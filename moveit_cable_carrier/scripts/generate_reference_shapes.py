#!/usr/bin/env python3
# Copyright 2026 Leow Chee Siang. Apache-2.0.
"""Generate ground-truth cable-carrier shapes with a full physics engine.

The C++ rod solver inside ``moveit_cable_carrier`` is deliberately cheap so it can run inside a
motion planner's inner loop.  Its *accuracy* therefore has to be established somewhere else.  This
script produces reference shapes with a real deformable solver, so that
``compare_reference.py`` can measure the surrogate's error and turn it into the
``safety_margin`` that keeps planning conservative.

Backends
--------
newton   NVIDIA Newton (https://github.com/newton-physics/newton), via ``ModelBuilder.add_rod``
         and ``newton.solvers.SolverVBD``.  Newton also carries a Dahl plasticity model for
         cable bending hysteresis -- the mechanism that makes a real dresspack keep part of its
         previous shape -- which ``--dahl`` switches on.

         .. warning::
            **Both ends cannot be clamped in Newton 1.0/1.5.**  A dresspack is clamped at two
            brackets, but Newton's rigid path is articulation-based and an articulation is a
            tree, so the second clamp is a loop closure that the API does not expose.  Verified
            three ways, all on Newton 1.0.0 with SolverVBD:

            * ``add_joint_fixed(-1, body)`` on a body created by ``add_rod`` is silently ignored
              -- with and without ``parent_xform``, the rod settles to exactly the same place as
              with no anchor at all, because the body already has a parent joint.
            * ``add_rod(wrap_in_articulation=False)`` then requires ``add_articulation(joints)``,
              which puts it back in a tree.
            * ``add_body(is_kinematic=True)`` for the far end is rejected outright:
              *"Only root bodies (whose joint parent is the world) can be kinematic."*

            The generator therefore checks after settling that the rod still starts at its
            bracket, and drops any case where it does not, rather than emitting a shape that
            merely fell under gravity.  Use the ``mujoco`` backend, whose equality ``connect``
            constraint does hold the far end, until Newton exposes a loop closure.

         Newton's solvers additionally need Python >= 3.11: they are lazily imported, so
         ``import newton`` and ``import newton.solvers`` both succeed on 3.10 and only touching
         a solver class raises.  Isaac Sim's interpreter (3.12) works.
mujoco   MuJoCo's ``elasticity.cable`` composite, with the moving bracket held by an equality
         ``connect`` constraint.  CPU only, and currently the only backend that can hold both
         ends.
mujoco   MuJoCo's ``elasticity.cable`` composite.  CPU-only and much easier to install; useful as
         an independent cross-check of the Newton numbers.

Both backends are optional.  Run with ``--list-backends`` to see what is importable here.

Input/output are plain JSON so this can run in a different environment (a GPU box, a Newton venv)
than the ROS workspace::

    generate_reference_shapes.py --backend newton cases.json reference.json

``cases.json``::

    {
      "carrier": {"length": 0.52, "bend_radius": 0.04, "num_segments": 24,
                  "outer_height": 0.016, "outer_width": 0.026},
      "cases": [{"base": {"xyz": [...], "quat_xyzw": [...]},
                 "tip":  {"xyz": [...], "quat_xyzw": [...]}}, ...]
    }
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from typing import Any, Sequence


# --------------------------------------------------------------------------------------------
# small quaternion helpers (kept dependency-free so --list-backends works anywhere)
# --------------------------------------------------------------------------------------------
def quat_rotate(q_xyzw: Sequence[float], v: Sequence[float]) -> list[float]:
    x, y, z, w = q_xyzw
    vx, vy, vz = v
    # t = 2 * (q_vec x v);  v' = v + w*t + q_vec x t
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return [
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    ]


def bracket_axis(pose: dict[str, Any]) -> list[float]:
    """The carrier travels along the bracket frame's local +X, base -> tip."""
    return quat_rotate(pose.get("quat_xyzw", [0.0, 0.0, 0.0, 1.0]), [1.0, 0.0, 0.0])


def available_backends() -> dict[str, bool]:
    found = {}
    for name, module in (("newton", "newton"), ("mujoco", "mujoco")):
        try:
            __import__(module)
            found[name] = True
        except Exception:
            found[name] = False
    return found


# --------------------------------------------------------------------------------------------
# Newton backend
# --------------------------------------------------------------------------------------------
def solve_newton(carrier: dict[str, Any], cases: list[dict[str, Any]], seeds: list[dict[str, Any]] | None,
                 *, dahl: bool, substeps: int, settle_steps: int) -> list[dict[str, Any]]:
    """Relax NVIDIA Newton's Cosserat rod between the two brackets.

    Uses ``ModelBuilder.add_rod``, which builds the rod straight from a centreline and gives direct
    control of stretch / bend / twist stiffness, and settles it with ``SolverVBD``. Newton also
    carries a Dahl plasticity model for cable bending hysteresis -- the mechanism that makes a real
    dresspack keep part of its previous shape -- which ``--dahl`` switches on.

    A seed centreline is required for the same reason as the MuJoCo backend: comparing the fast
    solver against a reference is only meaningful if both start from the same boundary conditions.
    """
    import inspect

    import newton
    import numpy as np
    import warp as wp
    from newton.solvers import SolverVBD

    n_seg = int(carrier["num_segments"])
    seg_len = float(carrier["length"]) / n_seg
    h = float(carrier["outer_height"])
    w = float(carrier["outer_width"])
    radius = 0.5 * math.hypot(h, w)

    # Starting stiffness values for a nylon section. They are only a starting point -- fitting them
    # to measured shapes is exactly what Warp's autodiff is for, and until that is done the
    # reference is not calibrated to any real part.
    e_modulus = float(carrier.get("youngs_modulus", 2.0e9))
    second_moment = math.pi * radius ** 4 / 4.0
    bend_k = float(carrier.get("bend_stiffness", e_modulus * second_moment / seg_len))
    twist_k = float(carrier.get("twist_stiffness", bend_k * 0.7))
    stretch_k = float(carrier.get("stretch_stiffness", bend_k * 1.0e4))

    results: list[dict[str, Any]] = []
    for index, case in enumerate(cases):
        if seeds is None or index >= len(seeds) or not seeds[index].get("nodes"):
            results.append({"nodes": [], "note": "no seed"})
            continue
        if not seeds[index].get("feasible", True):
            results.append({"nodes": [], "note": "seed infeasible"})
            continue
        seed_nodes = seeds[index]["nodes"]

        # Newton 1.0 takes gravity as a scalar along up_axis; 1.5 takes a vector. Passing the
        # wrong one is not caught at construction -- it blows up later inside finalize().
        probe = newton.ModelBuilder()
        gravity_is_vector = hasattr(getattr(probe, "gravity", None), "__len__")
        gravity_vec = carrier.get("gravity", [0.0, 0.0, -9.81])
        gravity_arg = gravity_vec if gravity_is_vector else -abs(float(gravity_vec[2]))

        builder = newton.ModelBuilder(gravity=gravity_arg)
        if dahl:
            # Custom attributes have to be registered before the model is built.
            SolverVBD.register_custom_attributes(builder, dahl_defaults_enabled=True)

        # add_rod's keyword set moved between Newton releases (1.0 has no twist/shear stiffness,
        # 1.5 does), so pass only what this build actually accepts rather than pinning a version.
        wanted = {
            "radius": radius,
            "stretch_stiffness": stretch_k,
            "bend_stiffness": bend_k,
            "twist_stiffness": twist_k,
            "shear_stiffness": twist_k,
            "label": "carrier",
        }
        accepted = set(inspect.signature(builder.add_rod).parameters)
        kwargs = {k: v for k, v in wanted.items() if k in accepted}
        dropped = sorted(set(wanted) - set(kwargs))

        created = builder.add_rod([wp.vec3(*[float(v) for v in p]) for p in seed_nodes], **kwargs)
        # Newton returns (body_ids, joint_ids); older builds returned nothing useful.
        body_ids = list(created[0]) if isinstance(created, tuple) and created else list(
            range(builder.body_count))

        # Pin two bodies at each end: two, not one, because a single fixed point would leave the
        # bracket angle free, and a bracket does not allow that.
        #
        # parent_xform must carry the body's *current* world pose. Without it the fixed joint
        # anchors the body to the world origin instead of to where the bracket is, the constraint
        # is effectively absent, and the rod simply falls under gravity -- which looks like a
        # converged shape and silently poisons any comparison made against it.
        body_q = getattr(builder, "body_q", None)
        for body in (body_ids[0], body_ids[1], body_ids[-2], body_ids[-1]):
            xform = body_q[body] if body_q is not None else None
            builder.add_joint_fixed(-1, body, parent_xform=xform)

        # SolverVBD solves coloured groups in parallel, so the bodies have to be coloured before
        # finalize(); without it the solver refuses the model outright.
        if hasattr(builder, "color"):
            builder.color()
        elif hasattr(builder, "set_coloring"):
            builder.set_coloring()

        model = builder.finalize()
        solver = SolverVBD(model, iterations=int(carrier.get("vbd_iterations", 20)))
        state_a = model.state()
        state_b = model.state()
        control = model.control()
        dt = float(carrier.get("dt", 1.0 / 240.0)) / max(1, substeps)

        contacts = None
        for _ in range(settle_steps):
            for _ in range(substeps):
                solver.step(state_a, state_b, control, contacts, dt)
                state_a, state_b = state_b, state_a

        settled_q = state_a.body_q.numpy()
        nodes = [[float(v) for v in settled_q[i][:3]] for i in range(len(settled_q))]
        finite = bool(np.all(np.isfinite(settled_q)))

        # Boundary-condition gate. A reference that quietly ignored its anchors is worse than no
        # reference at all, so check that the rod still starts at the bracket it was pinned to
        # before handing the shape on for comparison.
        anchor_error = float(np.linalg.norm(np.array(nodes[0]) - np.array(seed_nodes[0]))) if nodes else float("inf")
        anchored = anchor_error < float(carrier.get("anchor_tolerance", 0.01))

        note = ""
        if not finite:
            note = "diverged"
        elif not anchored:
            note = f"anchor not held: start moved {1000 * anchor_error:.0f} mm"

        results.append({"nodes": nodes if (finite and anchored) else [], "settled": finite,
                        "anchor_error": anchor_error, "unsupported_kwargs": dropped, "note": note})
    return results


# --------------------------------------------------------------------------------------------
# MuJoCo backend
# --------------------------------------------------------------------------------------------
def solve_mujoco(carrier: dict[str, Any], cases: list[dict[str, Any]], seeds: list[dict[str, Any]] | None,
                 *, settle_steps: int) -> list[dict[str, Any]]:
    """Relax MuJoCo's `elasticity.cable` composite between the two brackets.

    The composite is built from an explicit `vertex` list, which is why a seed centreline is
    required: MuJoCo needs the rod's rest shape, and starting from our solved shape means the two
    models are compared at the *same* boundary conditions rather than from unrelated states.

    The fixed bracket is the composite's parent body, so the start is pinned for free. The moving
    bracket is imposed with an equality `connect` constraint on the last link.
    """
    import mujoco
    import numpy as np

    radius = 0.5 * math.hypot(float(carrier["outer_height"]), float(carrier["outer_width"]))
    twist = float(carrier.get("mj_twist", 1.0e6))
    bend = float(carrier.get("mj_bend", 1.0e5))
    damping = float(carrier.get("mj_damping", 0.05))
    gravity_z = float(carrier.get("gravity", [0.0, 0.0, -9.81])[2])

    results: list[dict[str, Any]] = []
    for index, case in enumerate(cases):
        base = case["base"]["xyz"]
        tip = case["tip"]["xyz"]

        if seeds is None or index >= len(seeds) or not seeds[index].get("nodes"):
            results.append({"nodes": [], "note": "no seed"})
            continue
        seed_nodes = seeds[index]["nodes"]
        if not seeds[index].get("feasible", True):
            # Nothing to compare against: our solver already said this pose is unreachable.
            results.append({"nodes": [], "note": "seed infeasible"})
            continue

        verts = " ".join(f"{p[k] - base[k]:.6f}" for p in seed_nodes for k in range(3))
        xml = f"""
<mujoco model="carrier">
  <extension><plugin plugin="mujoco.elasticity.cable"/></extension>
  <option gravity="0 0 {gravity_z}" timestep="0.002"/>
  <worldbody>
    <body name="anchor" pos="{base[0]} {base[1]} {base[2]}">
      <composite type="cable" curve="s" initial="none" vertex="{verts}">
        <plugin plugin="mujoco.elasticity.cable">
          <config key="twist" value="{twist}"/>
          <config key="bend" value="{bend}"/>
          <config key="vmax" value="0"/>
        </plugin>
        <joint kind="main" damping="{damping}"/>
        <geom type="capsule" size="{radius}" rgba="0.2 0.8 0.2 1" condim="1"/>
      </composite>
    </body>
    <body name="tip_anchor" pos="{tip[0]} {tip[1]} {tip[2]}">
      <geom type="sphere" size="0.001" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <equality>
    <connect body1="B_last" body2="tip_anchor" anchor="0 0 0"/>
  </equality>
</mujoco>
"""
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        for _ in range(settle_steps):
            mujoco.mj_step(model, data)

        nodes = []
        for i in range(model.nbody):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ""
            if name.startswith("B_"):
                nodes.append([float(v) for v in data.xpos[i]])
        settled = bool(np.all(np.isfinite(data.qvel))) and float(np.abs(data.qvel).max()) < 1.0
        results.append({"nodes": nodes, "settled": settled,
                        "max_qvel": float(np.abs(data.qvel).max()) if data.qvel.size else 0.0})
    return results


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cases", nargs="?", help="input JSON with carrier params and bracket pose pairs")
    ap.add_argument("output", nargs="?", help="output JSON with reference node positions")
    ap.add_argument("--backend", choices=("newton", "mujoco"), default="newton")
    ap.add_argument("--seed", default="",
                    help="shapes.json from carrier_benchmark. Required by the mujoco backend, "
                         "which needs an explicit rest centreline for the cable composite.")
    ap.add_argument("--list-backends", action="store_true", help="report which engines are importable")
    ap.add_argument("--dahl", action="store_true",
                    help="Newton only: enable the Dahl cable-bending hysteresis model")
    ap.add_argument("--substeps", type=int, default=4)
    ap.add_argument("--gravity", type=float, default=-9.81,
                    help="gravity along Z. Pass 0 to compare pure elastic shape: the fast solver "
                         "treats gravity as a heuristic sag term, so leaving it on confounds "
                         "model error with a difference in how sag is modelled.")
    ap.add_argument("--settle-steps", type=int, default=400,
                    help="steps to run before reading the quasi-static shape")
    args = ap.parse_args()

    if args.list_backends:
        for name, ok in available_backends().items():
            print(f"{name:8s} {'available' if ok else 'NOT INSTALLED'}")
        return 0

    if not args.cases or not args.output:
        ap.error("cases and output are required unless --list-backends is given")

    with open(args.cases) as f:
        payload = json.load(f)
    carrier = payload["carrier"]
    carrier.setdefault("gravity", [0.0, 0.0, args.gravity])
    carrier["gravity"] = [0.0, 0.0, args.gravity]
    cases = payload["cases"]

    try:
        seeds = None
        if args.seed:
            with open(args.seed) as f:
                seeds = json.load(f)["cases"]

        if args.backend == "newton":
            shapes = solve_newton(carrier, cases, seeds, dahl=args.dahl, substeps=args.substeps,
                                  settle_steps=args.settle_steps)
        else:
            if seeds is None:
                print("--seed is required for the mujoco backend", file=sys.stderr)
                return 2
            shapes = solve_mujoco(carrier, cases, seeds, settle_steps=args.settle_steps)
    except ImportError as exc:
        print(f"backend '{args.backend}' is not installed: {exc}", file=sys.stderr)
        print("install with:  pip install newton-physics warp-lang   (or)   pip install mujoco", file=sys.stderr)
        return 2

    with open(args.output, "w") as f:
        json.dump({"backend": args.backend, "carrier": carrier, "cases": shapes}, f, indent=1)
    print(f"wrote {len(shapes)} reference shapes to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
