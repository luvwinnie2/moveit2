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
newton   NVIDIA Newton (https://github.com/newton-physics/newton).  Targets the 1.4-era API:
         ``newton.ModelBuilder.add_joint_cable`` for the Cosserat-style cable joints and
         ``newton.solvers.SolverVBD`` to settle them.  Newton additionally offers a Dahl
         plasticity model for cable bending hysteresis (``model.vbd.dahl_eps_max`` /
         ``model.vbd.dahl_tau``), which is the mechanism that makes a real drag chain keep some
         of its previous shape -- see ``--dahl``.
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
def solve_newton(carrier: dict[str, Any], cases: list[dict[str, Any]], *, dahl: bool,
                 substeps: int, settle_steps: int) -> list[dict[str, Any]]:
    import newton  # noqa: F401  (import errors are reported by the caller)
    import warp as wp
    from newton.solvers import SolverVBD

    n_seg = int(carrier["num_segments"])
    seg_len = float(carrier["length"]) / n_seg
    # Bending stiffness of a rectangular nylon section.  These are starting values: the whole
    # point of calibrate-against-reality is that they get fitted, see --stiffness overrides.
    e_modulus = float(carrier.get("youngs_modulus", 2.0e9))       # nylon, Pa
    h = float(carrier["outer_height"])
    w = float(carrier["outer_width"])
    second_moment = w * h ** 3 / 12.0
    bend_k = float(carrier.get("bend_stiffness", e_modulus * second_moment / seg_len))
    twist_k = float(carrier.get("twist_stiffness", bend_k * 2.0))
    stretch_k = float(carrier.get("stretch_stiffness", bend_k * 1.0e4))

    results = []
    for case in cases:
        builder = newton.ModelBuilder(gravity=carrier.get("gravity", [0.0, 0.0, -9.81]))
        if dahl:
            # Registering the custom attributes is required *before* the model is built.
            SolverVBD.register_custom_attributes(builder, dahl_defaults_enabled=True)

        base = case["base"]
        tip = case["tip"]
        b0 = base["xyz"]
        t0 = tip["xyz"]
        d_base = bracket_axis(base)
        d_tip = bracket_axis(tip)

        # Straight-line seed between the brackets that already respects both exit tangents;
        # the solver then relaxes it.  A poor seed makes VBD take far more iterations.
        nodes = []
        for i in range(n_seg + 1):
            s = i / n_seg
            blend = 3 * s * s - 2 * s * s * s
            p = [
                (1 - blend) * (b0[k] + d_base[k] * seg_len * i) + blend * (t0[k] - d_tip[k] * seg_len * (n_seg - i))
                for k in range(3)
            ]
            nodes.append(p)

        bodies = []
        for i, p in enumerate(nodes):
            body = builder.add_body(xform=wp.transform(p, wp.quat_identity()),
                                    mass=float(carrier.get("mass_per_node", 0.02)))
            builder.add_shape_capsule(body, radius=0.5 * math.hypot(h, w), half_height=0.5 * seg_len)
            bodies.append(body)
            # Clamp two nodes at each end: that is what encodes the bracket *orientation*.
            if i <= 1 or i >= n_seg - 1:
                builder.add_joint_fixed(-1, body)

        for i in range(n_seg):
            builder.add_joint_cable(
                bodies[i], bodies[i + 1],
                stretch_stiffness=stretch_k,
                bend_stiffness=bend_k,
                twist_stiffness=twist_k,
                label=f"seg{i}",
            )

        model = builder.finalize()
        solver = SolverVBD(model, iterations=int(carrier.get("vbd_iterations", 20)))
        state_a = model.state()
        state_b = model.state()
        control = model.control()
        dt = float(carrier.get("dt", 1.0 / 240.0)) / max(1, substeps)

        for _ in range(settle_steps):
            for _ in range(substeps):
                contacts = model.collide(state_a) if hasattr(model, "collide") else None
                solver.step(state_a, state_b, control, contacts, dt)
                state_a, state_b = state_b, state_a

        body_q = state_a.body_q.numpy()
        results.append({"nodes": [[float(v) for v in body_q[i][:3]] for i in range(n_seg + 1)]})
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
  <option gravity="0 0 -9.81" timestep="0.002"/>
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
