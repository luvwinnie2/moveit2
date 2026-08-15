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
carrier feasible : 98.6 %          shape solve : p50 241 us
full check       : p50 397 us      achieved min bend radius == R_min exactly
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

## Known limitations

1. **The solver does not see obstacles.** It returns the free-space minimum-energy shape, which can
   pass through the arm. `touch_links` keeps that from being reported as a collision on the mount
   links, but the shape while resting on a link is not reproduced.
2. **No hysteresis.** A real carrier retains part of its previous shape. Newton's Dahl cable model
   covers this; it is not modelled here.
3. **`safety_margin` is not yet calibrated.** The MuJoCo cross-check disagrees by up to 193 mm, but
   that comparison is not valid as configured: MuJoCo's cable has no minimum-bend-radius constraint
   and its stiffness was not fitted to the real part, so it measures the difference between two
   different physical models rather than the surrogate's error. Calibrating the reference is
   required before the number means anything.
4. **Bending stress** is reported from Euler-Bernoulli beam theory, which suits a continuous cable
   but not an articulated ball-and-socket carrier shell. Treat `bend_utilisation` as the meaningful
   quantity for the carrier itself.

## Compatibility

Verified to build and pass its tests against ROS 2 Humble. The include paths use the `.h` form,
which exists both on Humble and as compatibility headers on `main`; migrating to `.hpp` is a
mechanical follow-up once Humble support is dropped.

See `README.ja.md` for the Japanese version, which additionally carries a glossary and a
troubleshooting table.
