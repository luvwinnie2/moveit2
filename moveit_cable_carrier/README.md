# moveit_cable_carrier

Collision support for **deformable cable carriers** (robot dresspacks / energy chains, e.g. igus
triflex R) that are mounted across two links of a manipulator.

## The gap this fills

MoveIt has no way to express collision geometry whose *shape* is a function of the joint
configuration:

* `AttachedCollisionObject` is rigid and attached to exactly one link.
* A dresspack spans two links, so it cannot be attached to either.
* Its shape changes with every wrist motion, so no single precomputed body describes it.

The usual workaround — wrapping the whole run in an oversized box — blocks paths the robot can
actually take. This package solves the carrier's quasi-static shape for each queried state and
injects it into the ordinary collision pipeline.

## How it hooks into MoveIt

A `CollisionEnv` that wraps FCL. For each query it copies the state, solves the carrier, attaches
the result as a normal `AttachedBody`, then delegates. Everything MoveIt already does well — the
ACM, link padding, distance queries, contact naming — keeps working. The attached body's
`touch_links` carry the links the carrier may rest against, so no ACM editing is required.

```yaml
collision_detector: CABLE_CARRIER   # in your move_group configuration
```

**Why not `PlanningScene::setStateFeasibilityPredicate`.** It is silently ignored under
`move_group`: `isStateFeasible` does not consult the parent scene, the diff constructor does not
copy the predicate, and `move_group` plans on a diff of the monitored scene.
`allocateCollisionDetector` does survive diffs, so the detector route is the only one that works.

## The shape solver

The unknowns are the **joint rotations of a discrete rod**, not node positions. Consequently:

| constraint | status |
|---|---|
| inextensible (constant arc length) | exact by construction |
| minimum bend radius `R_min` | exact, imposed by clamping each joint |
| moving bracket pose | driven to convergence; the residual is covered by `safety_margin` |

A position-based (PBD) formulation was tried first and fails structurally: the length and
curvature projections cancel each other and the iteration settles on a point satisfying neither.
Raising the iteration count from 200 to 1500 changed nothing, which is what identified it as a
stall rather than slow convergence.

A secondary "straighten out" objective is projected into the endpoint task's null space and
annealed away over the iterations, so the solve picks the minimum-bending-energy shape among those
that reach the bracket rather than one pinned against the bend limit.

Measured on a FANUC CRX-5iA, carrier spanning J4/J5/J6, 9³ = 729 wrist configurations:

```
carrier feasible : 98.9 %          shape solve : p50  57 us, mean 115 us
in collision     :  1.1 %          full check  : p50 411 us, mean 526 us
achieved minimum bend radius == R_min exactly, i.e. the constraint is active
```

Build with `-DCMAKE_BUILD_TYPE=Release`; an unoptimised build is ~600x slower.

## Tools

| tool | purpose |
|---|---|
| `carrier_benchmark` | sweeps the spanned joints; reports feasibility, timings, bracket chord range |
| `carrier_trajectory_check` | motion-cycle analysis: per-waypoint bend utilisation, stress, collisions, and which segment is worked hardest |
| `carrier_visualizer` | publishes the solved shape as markers so the geometry MoveIt plans against is visible in RViz |
| `scripts/make_carrier_urdf.py` | generates bracket frames into a URDF from the same YAML the collision model reads |
| `scripts/generate_reference_shapes.py` | reference shapes from MuJoCo's `elasticity.cable` or NVIDIA Newton's rod/VBD solver |
| `scripts/compare_reference.py` | turns the surrogate-vs-reference deviation into a `safety_margin` |

```bash
ros2 launch moveit_cable_carrier carrier_demo.launch.py   # move the wrist, watch it deform
```

## Supported run types

`config/carrier_types.yaml` is a type library. Every entry declares its `provenance`
(`measured` / `rule` / `template`); do not ship a `template` entry without replacing it with the
vendor's numbers.

| kind | what it is | binding limit |
|---|---|---|
| `articulated_carrier` | 3D dresspack (igus triflex R and equivalents), the standard for axes 3-6 | defined bend radius plus a torsion stop of about ±10° per link |
| `planar_chain` | planar link/linkless chain (Kunimori Silveyer KSL/KSH), **for linear axes** | one bend axis, back-stopped to one side |
| `bare_cable` | bare cable or hose | a multiple of its own OD: 10x for continuous flex, 7.5x for qualified high-flex |
| `corrugated_hose` | corrugated conduit | no defined bend radius and no torsion stop |

### The binding limit is usually the cable, not the carrier

Declaring `inner_cables` checks the carrier's own stop *and* what its contents will tolerate.
Measured over one pruning cycle on the CRX-5iA:

```
carrier (R_min 40 mm) utilisation : 0.32-0.74   comfortable
10 mm robot cable, needs 10x OD   : 5.4x-9.3x   violated on 15 of 24 waypoints
```

The carrier is relaxed at a 62 mm radius; the 10 mm cable inside wants 100 mm. That is the finding
the tool exists to produce.

## Reference-solver cross-check

| reference | status |
|---|---|
| **MuJoCo** `elasticity.cable` | works; the moving bracket is held by an equality `connect` constraint |
| **NVIDIA Newton** rod + VBD | implemented, but **cannot clamp both ends** — see below |

Against MuJoCo over 62 configurations, gravity disabled:

```
mean deviation   0.44 mm
worst deviation 21.7 mm   (p99 equals the worst, so one outlier sets it)
```

With gravity on the disagreement grows to 63.6 mm mean / 193 mm worst, but that is not model
error: this solver treats gravity as a heuristic sag term, so leaving it on compares two different
sag models rather than the elastic shape. `safety_margin: 0.024` comes from the measured 21.7 mm
plus 2 mm for bracket play.

### Why Newton cannot hold both brackets

A dresspack is clamped at two brackets. Newton's rigid path is articulation-based and an
articulation is a tree, so the second clamp is a loop closure the API does not expose. Verified
three ways on Newton 1.0.0 with `SolverVBD`, all reaching the same wall:

* `add_joint_fixed(-1, body)` on a body created by `add_rod` is silently ignored — with or without
  `parent_xform`, the rod settles to *exactly* the same place as with no anchor at all, because the
  body already has a parent joint.
* `add_rod(wrap_in_articulation=False)` then demands `add_articulation(joints)`, putting it back in
  a tree.
* Making the far end `add_body(is_kinematic=True)` is rejected outright: *"Only root bodies (whose
  joint parent is the world) can be kinematic."*

The generator therefore verifies after settling that the rod still starts at the bracket it was
pinned to, and drops any case that does not — a shape that merely fell under gravity looks
converged and would silently poison the comparison. That gate caught 64 of 64 cases.

Newton's solvers also need Python >= 3.11. They are lazily imported, so `import newton` and
`import newton.solvers` both succeed on 3.10 and only touching a solver class raises; Isaac Sim's
3.12 interpreter works.

## Known limitations

1. **The solver does not see obstacles.** It returns the free-space minimum-energy shape, which can
   pass through the arm. `touch_links` keeps that from being reported as a collision on the mount
   links, but the shape while resting on a link is not reproduced.
2. **No hysteresis.** A real carrier retains part of its previous shape. Newton's Dahl cable model
   covers this; it is not modelled here.
3. **Gravity is crude.** `gravity_sag` nudges the nodes each iteration rather than solving static
   equilibrium, which is what opens the 63.6 mm mean gap once gravity is enabled. Long unsupported
   runs need it fitted to measurement.
4. **No stress in pascals, deliberately.** Euler-Bernoulli beam stress on the shell is meaningless
   for a ball-and-socket chain that bends at its joints — it produced figures ten times the yield
   of any polymer such a carrier is made from. The reported quantities are bend utilisation and the
   industry's radius-to-OD ratio instead.

## Compatibility

Verified to build and pass its tests against ROS 2 Humble. The include paths use the `.h` form,
which exists both on Humble and as compatibility headers on `main`; migrating to `.hpp` is a
mechanical follow-up once Humble support is dropped.

See `README.ja.md` for the Japanese version, which additionally carries a glossary and a
troubleshooting table.
