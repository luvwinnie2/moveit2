#!/usr/bin/env python3
# Copyright 2026 Leow Chee Siang. Apache-2.0.
"""Measure the fast rod solver's error against a physics-engine reference.

This closes the loop of the two-tier design:

  * the C++ ``RodSolver`` runs inside MoveIt's collision checks and must be microseconds fast,
  * a real deformable solver (Newton / MuJoCo, via ``generate_reference_shapes.py``) is the
    ground truth but is far too slow for a planner's inner loop.

The gap between them is model error, and model error has to be paid for with a conservative
inflation of the collision geometry.  This script reports the Hausdorff-style deviation between
the two centrelines and prints the ``safety_margin`` that would cover it.

    carrier_benchmark <urdf> <srdf> <carrier.yaml> 9 shapes.json     # ours
    generate_reference_shapes.py cases.json reference.json           # ground truth
    compare_reference.py shapes.json reference.json

Both files use the same schema: ``{"cases": [{"nodes": [[x, y, z], ...]}, ...]}``.
"""

from __future__ import annotations

import argparse
import json
import math
from typing import Sequence


def _point_segment_distance(p: Sequence[float], a: Sequence[float], b: Sequence[float]) -> float:
    ab = [b[i] - a[i] for i in range(3)]
    ap = [p[i] - a[i] for i in range(3)]
    denom = sum(v * v for v in ab)
    t = 0.0 if denom <= 1e-18 else max(0.0, min(1.0, sum(ap[i] * ab[i] for i in range(3)) / denom))
    return math.sqrt(sum((ap[i] - t * ab[i]) ** 2 for i in range(3)))


def polyline_distance(points: list[list[float]], polyline: list[list[float]]) -> list[float]:
    """Distance from each point to the closest place on `polyline`."""
    out = []
    for p in points:
        best = float("inf")
        for i in range(len(polyline) - 1):
            best = min(best, _point_segment_distance(p, polyline[i], polyline[i + 1]))
        out.append(best)
    return out


def symmetric_deviation(a: list[list[float]], b: list[list[float]]) -> tuple[float, float]:
    """(max, mean) two-sided deviation between two centrelines."""
    if len(a) < 2 or len(b) < 2:
        return float("nan"), float("nan")
    d = polyline_distance(a, b) + polyline_distance(b, a)
    return max(d), sum(d) / len(d)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ours", help="JSON dumped by carrier_benchmark")
    ap.add_argument("reference", help="JSON from generate_reference_shapes.py")
    ap.add_argument("--percentile", type=float, default=99.0,
                    help="percentile of the per-case max deviation used to size the margin")
    args = ap.parse_args()

    with open(args.ours) as f:
        ours = json.load(f)["cases"]
    with open(args.reference) as f:
        ref_payload = json.load(f)
    reference = ref_payload["cases"]

    if len(ours) != len(reference):
        print(f"case count differs: ours={len(ours)} reference={len(reference)}")
        return 1

    per_case_max: list[float] = []
    per_case_mean: list[float] = []
    skipped = 0
    for i, (o, r) in enumerate(zip(ours, reference)):
        if not o.get("feasible", True):
            skipped += 1
            continue
        mx, mn = symmetric_deviation(o["nodes"], r["nodes"])
        if math.isnan(mx):
            skipped += 1
            continue
        per_case_max.append(mx)
        per_case_mean.append(mn)

    if not per_case_max:
        print("no comparable cases")
        return 1

    per_case_max.sort()
    idx = min(len(per_case_max) - 1, int(args.percentile / 100.0 * (len(per_case_max) - 1)))
    p_val = per_case_max[idx]

    print(f"reference backend : {ref_payload.get('backend', '?')}")
    print(f"cases compared    : {len(per_case_max)} (skipped {skipped} infeasible/degenerate)")
    print(f"mean deviation    : {1000 * sum(per_case_mean) / len(per_case_mean):8.2f} mm")
    print(f"p{args.percentile:g} max deviation : {1000 * p_val:8.2f} mm")
    print(f"worst deviation   : {1000 * per_case_max[-1]:8.2f} mm")
    print()
    print("Set safety_margin to at least the worst deviation, plus an allowance for")
    print("manufacturing tolerance and bracket play, e.g.:")
    print(f"    safety_margin: {per_case_max[-1] + 0.002:.4f}   # {1000 * per_case_max[-1]:.1f} mm model error + 2 mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
