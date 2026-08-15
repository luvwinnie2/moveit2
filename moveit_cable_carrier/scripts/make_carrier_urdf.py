#!/usr/bin/env python3
# Copyright 2026 Leow Chee Siang. Apache-2.0.
"""Emit a *_with_carrier URDF variant from a base URDF plus a carrier YAML.

Why generate instead of hand-editing a copy: the bracket poses live in the carrier YAML because
that is what the MoveIt collision model reads. If the URDF carried its own hand-written copy of
those numbers the two would drift apart, and a carrier that is modelled 2 cm away from where it is
mounted is worse than no model at all. Generating keeps one source of truth.

What is added, per carrier:
  * ``<carrier>_base_mount``  -- fixed frame on the base link, at ``base_mount``
  * ``<carrier>_tip_mount``   -- fixed frame on the tip link, at ``tip_mount``
Each is a massless link with a small bracket plate as visual geometry, so the mounting shows up in
RViz and in TF and can be eyeballed against the real robot. Both frames follow the package's
convention that **local +X is the direction the cable travels**, base -> tip.

The carrier body itself is deliberately *not* put in the URDF: its shape depends on the joint
configuration, which URDF cannot express. That is exactly what ``moveit_cable_carrier`` computes at
runtime, and what ``carrier_visualizer`` draws.

    make_carrier_urdf.py crx5ia.urdf crx5ia_carrier.yaml crx5ia_with_carrier.urdf
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET


def _fmt(values) -> str:
    return " ".join(f"{float(v):.6g}" for v in values)


def load_carriers(path: str) -> list[dict]:
    try:
        import yaml
    except ImportError:
        print("PyYAML is required: pip install pyyaml", file=sys.stderr)
        raise
    with open(path) as handle:
        doc = yaml.safe_load(handle)
    carriers = doc.get("carriers", doc)
    if not isinstance(carriers, list):
        raise ValueError("carrier YAML must contain a 'carriers' sequence")
    return carriers


def add_mount(robot: ET.Element, name: str, parent: str, pose: dict, size, colour) -> None:
    xyz = pose.get("xyz", [0.0, 0.0, 0.0]) if pose else [0.0, 0.0, 0.0]
    rpy = pose.get("rpy", [0.0, 0.0, 0.0]) if pose else [0.0, 0.0, 0.0]

    link = ET.SubElement(robot, "link", {"name": name})
    visual = ET.SubElement(link, "visual")
    # The plate is drawn offset along -X so the frame origin sits on the plate's outer face,
    # which is where the carrier actually leaves the bracket.
    ET.SubElement(visual, "origin", {"xyz": _fmt([-size[0] / 2.0, 0.0, 0.0]), "rpy": "0 0 0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box", {"size": _fmt(size)})
    material = ET.SubElement(visual, "material", {"name": f"{name}_material"})
    ET.SubElement(material, "color", {"rgba": _fmt(colour)})

    joint = ET.SubElement(robot, "joint", {"name": f"{name}_joint", "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": name})
    ET.SubElement(joint, "origin", {"xyz": _fmt(xyz), "rpy": _fmt(rpy)})


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("urdf_in")
    ap.add_argument("carrier_yaml")
    ap.add_argument("urdf_out")
    ap.add_argument("--plate", type=float, nargs=3, default=(0.008, 0.05, 0.05),
                    metavar=("X", "Y", "Z"), help="bracket plate size in metres")
    args = ap.parse_args()

    tree = ET.parse(args.urdf_in)
    robot = tree.getroot()
    if robot.tag != "robot":
        print(f"{args.urdf_in} is not a URDF (root tag is <{robot.tag}>)", file=sys.stderr)
        return 1

    existing_links = {link.get("name") for link in robot.findall("link")}

    added = 0
    for carrier in load_carriers(args.carrier_yaml):
        name = carrier.get("name", "carrier")
        for suffix, parent_key, pose_key, colour in (
            ("base_mount", "base_link", "base_mount", (0.15, 0.55, 0.95, 1.0)),
            ("tip_mount", "tip_link", "tip_mount", (0.95, 0.55, 0.15, 1.0)),
        ):
            parent = carrier.get(parent_key)
            if not parent:
                print(f"carrier '{name}' has no {parent_key}", file=sys.stderr)
                return 1
            if parent not in existing_links:
                print(f"carrier '{name}': {parent_key} '{parent}' is not a link in {args.urdf_in}",
                      file=sys.stderr)
                return 1
            frame = f"{name}_{suffix}"
            if frame in existing_links:
                print(f"'{frame}' already exists in the input URDF; refusing to duplicate it",
                      file=sys.stderr)
                return 1
            add_mount(robot, frame, parent, carrier.get(pose_key), args.plate, colour)
            added += 1

    ET.indent(tree, space="  ")
    tree.write(args.urdf_out, encoding="utf-8", xml_declaration=True)
    print(f"wrote {args.urdf_out} (+{added} bracket frames)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
