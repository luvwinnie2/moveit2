#!/usr/bin/env python3
# Copyright 2026 Leow Chee Siang. Apache-2.0.
"""Make a URDF that MuJoCo will actually load, then compile it to MJCF.

MuJoCo reads URDF, but not the URDF ROS people write. Four things stop it every time, and all four
are fixed here rather than by hand-editing a file that is generated from CAD:

1. `package://pkg/path` means nothing to MuJoCo. Resolved to absolute paths via the ament index.
2. Mesh files must sit somewhere the compiler is told about, and Collada is not supported. Ours are
   already OBJ, so only the paths need rewriting.
3. A link with no <inertial> is massless. MuJoCo will not simulate a massless body that has a
   joint, and the failure is a solver that quietly does nothing rather than an error. Links without
   inertia get a small one, and it is logged so nobody mistakes it for measured data.
4. MuJoCo needs its own <mujoco> extension block for compiler settings. URDF from a robot vendor
   never has one.

    ros2 run moveit2_extended_mujoco urdf_to_mujoco.py --urdf <in.urdf> --out <out.xml>

The output is a real MJCF file, so it can be opened in the MuJoCo viewer and edited by hand for
things this script deliberately does not guess at -- contact parameters, actuator gains, scene
lighting.
"""

from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET

DEFAULT_MASS = 0.05  # kg, for links the URDF gives no inertia
DEFAULT_INERTIA = 1e-4  # kg m^2, diagonal


def resolve_package_uri(uri: str) -> str | None:
    """package://pkg/rest -> absolute path, or None if it cannot be found."""
    if not uri.startswith("package://"):
        return uri if os.path.exists(uri) else None
    package, _, relative = uri[len("package://") :].partition("/")
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory(package)
    except Exception:  # noqa: BLE001 - also used outside a sourced ROS environment
        return None
    path = os.path.join(share, relative)
    return path if os.path.exists(path) else None


def prepare(urdf_path: str, verbose: bool = True) -> tuple[str, list[str]]:
    """Return (urdf text MuJoCo can compile, notes about what had to be changed)."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    notes: list[str] = []

    # 1 + 2: meshes
    missing: list[str] = []
    for mesh in root.iter("mesh"):
        original = mesh.get("filename", "")
        resolved = resolve_package_uri(original)
        if resolved is None:
            missing.append(original)
            continue
        mesh.set("filename", resolved)
    if missing:
        notes.append(f"{len(missing)} mesh reference(s) could not be resolved: {sorted(set(missing))}")

    # 3: inertials. A jointed body with no mass is not simulated, and says nothing about it.
    invented: list[str] = []
    for link in root.findall("link"):
        if link.find("inertial") is not None:
            continue
        name = link.get("name", "?")
        # A link with neither inertia nor geometry is a frame marker (tool0, cutting_point); those
        # are fine massless because the compiler welds them into their parent.
        if link.find("visual") is None and link.find("collision") is None:
            continue
        invented.append(name)
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(inertial, "mass", {"value": str(DEFAULT_MASS)})
        ET.SubElement(
            inertial,
            "inertia",
            {
                "ixx": str(DEFAULT_INERTIA), "ixy": "0", "ixz": "0",
                "iyy": str(DEFAULT_INERTIA), "iyz": "0", "izz": str(DEFAULT_INERTIA),
            },
        )
    if invented:
        notes.append(
            f"invented inertia ({DEFAULT_MASS} kg) for {len(invented)} link(s) the URDF leaves "
            f"massless: {invented}. These are NOT measured values -- they exist so the solver runs."
        )

    # 4: the compiler block.
    #   balanceinertia    fixes inertia tensors that violate the triangle inequality, which CAD
    #                     exports do often enough that MuJoCo refuses to load without it.
    #   discardvisual     false, because the visual meshes are what makes the viewer legible.
    #   fusestatic        false, so fixed links keep their names and TF still lines up with ROS.
    #   strippath         false, since filenames are now absolute.
    for existing in root.findall("mujoco"):
        root.remove(existing)
    extension = ET.Element("mujoco")
    ET.SubElement(
        extension,
        "compiler",
        {
            "balanceinertia": "true",
            "discardvisual": "false",
            "fusestatic": "false",
            "strippath": "false",
            "angle": "radian",
        },
    )
    root.insert(0, extension)

    if verbose:
        for note in notes:
            print(f"note: {note}", file=sys.stderr)
    return ET.tostring(root, encoding="unicode"), notes


def compile_to_mjcf(urdf_text: str) -> str:
    """Compile with MuJoCo and hand back the MJCF it produced."""
    import mujoco

    model = mujoco.MjModel.from_xml_string(urdf_text)
    # save_last_xml writes what the compiler actually built, which is the thing worth keeping:
    # it has the resolved meshes, the balanced inertias, and MuJoCo's own defaults made explicit.
    path = "/tmp/_mjcf_out.xml"
    mujoco.mj_saveLastXML(path, model)
    with open(path, encoding="utf-8") as handle:
        return handle.read()


def measure_gravity_torque(mjcf: str, n_joints: int, samples: int = 12) -> list[float]:
    """Largest |gravity torque| each joint has to hold, swept over the reachable range.

    Sized from the model rather than guessed, because guessing is what produced the first version of
    this file: a flat kp=300 held the wrist fine and let the shoulder collapse under its own arm,
    all the way to the joint limit, at which point the simulation looked like a kinematics bug.
    """
    import itertools

    import mujoco
    import numpy as np

    model = mujoco.MjModel.from_xml_string(mjcf)
    data = mujoco.MjData(model)
    ranges = []
    for index in range(n_joints):
        low, high = model.jnt_range[index]
        if low == 0.0 and high == 0.0:  # unlimited
            low, high = -3.14, 3.14
        ranges.append(np.linspace(low, high, samples))

    # Only the first three joints move the arm's mass far enough to matter; sweeping all six would
    # be samples**6 evaluations for a result the wrist cannot change.
    worst = np.zeros(model.nv)
    sweep = list(itertools.product(*ranges[: min(3, n_joints)]))
    for combination in sweep:
        data.qpos[:] = 0.0
        for index, value in enumerate(combination):
            data.qpos[index] = value
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        worst = np.maximum(worst, np.abs(data.qfrc_bias[: model.nv]))
    return [float(v) for v in worst[:n_joints]]


def add_actuators_and_keyframes(mjcf: str, joints: list[str], tolerance: float, armature: float,
                                damping_ratio: float, keyframes: dict[str, list[float]],
                                verbose: bool = True) -> str:
    """Give the model position actuators sized to hold it up, plus named poses to reset to.

    A URDF converted to MJCF has no actuators at all, so the arm is a ragdoll: joints and mass and
    nothing to hold them. It also has no armature and no joint damping, because URDF cannot express
    either -- and a real industrial arm gets most of its stiffness from the reflected rotor inertia
    of its gearboxes. Both are added here so the model behaves like a geared arm rather than a
    frictionless direct-drive one.

    kp is per joint, computed as (worst gravity torque) / tolerance. kv is critically damped for the
    joint's own inertia. These are honest starting values derived from the model's own mass
    properties -- they are NOT identified against the real robot, and anything that depends on
    contact forces should be calibrated before it is believed.
    """
    root = ET.fromstring(mjcf)
    torques = measure_gravity_torque(mjcf, len(joints))

    # Armature and damping go on the joints, before the actuators are sized against them.
    for element in root.iter("joint"):
        if element.get("name") in joints:
            element.set("armature", str(armature))
            element.set("damping", str(round(armature * 2.0, 4)))

    for existing in root.findall("actuator"):
        root.remove(existing)
    actuator = ET.SubElement(root, "actuator")
    gains = []
    for index, joint in enumerate(joints):
        kp = max(torques[index] / tolerance, 50.0)
        # Critical damping for a unit-inertia joint plus its armature; enough to stop the ringing a
        # stiff position actuator otherwise produces.
        kv = damping_ratio * 2.0 * (kp * max(armature, 1e-3)) ** 0.5
        gains.append((joint, round(kp), round(kv, 1), round(torques[index], 2)))
        ET.SubElement(actuator, "position", {
            "name": f"{joint}_pos",
            "joint": joint,
            "kp": f"{kp:.1f}",
            "kv": f"{kv:.2f}",
        })

    if keyframes:
        for existing in root.findall("keyframe"):
            root.remove(existing)
        keyframe = ET.SubElement(root, "keyframe")
        for name, values in keyframes.items():
            ET.SubElement(keyframe, "key", {
                "name": name,
                "qpos": " ".join(f"{v:.6f}" for v in values),
                # Start the actuators holding the same pose, or the arm falls on the first step.
                "ctrl": " ".join(f"{v:.6f}" for v in values),
            })

    if verbose:
        print(f"actuator gains (tolerance {tolerance} rad, armature {armature}):", file=sys.stderr)
        for joint, kp, kv, torque in gains:
            print(f"  {joint:4s} gravity {torque:6.2f} Nm -> kp {kp:6d}  kv {kv:6.1f}", file=sys.stderr)

    return ET.tostring(root, encoding="unicode")


def write_ros2_control_urdf(urdf_path: str, mjcf_path: str, joints: list[str],
                            keyframe: str, out_path: str) -> None:
    """Emit the robot's URDF with a <ros2_control> block bolted on.

    controller_manager needs one robot_description that describes both the robot and the hardware
    that drives it, so this is the original URDF plus the tag -- generated rather than hand-edited,
    because the URDF itself comes out of CAD and any edit would be lost on the next export.

    Command interfaces are position only. The MJCF's actuators are position actuators, so offering
    a velocity command interface would advertise something the model cannot honour.
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    for existing in root.findall("ros2_control"):
        root.remove(existing)

    control = ET.SubElement(root, "ros2_control", {"name": "MujocoSystem", "type": "system"})
    hardware = ET.SubElement(control, "hardware")
    ET.SubElement(hardware, "plugin").text = "moveit2_extended_mujoco/MujocoSystem"
    ET.SubElement(hardware, "param", {"name": "mjcf"}).text = mjcf_path
    if keyframe:
        ET.SubElement(hardware, "param", {"name": "initial_keyframe"}).text = keyframe

    limits = {j.get("name"): j.find("limit") for j in root.findall("joint")}
    for name in joints:
        joint = ET.SubElement(control, "joint", {"name": name})
        command = ET.SubElement(joint, "command_interface", {"name": "position"})
        limit = limits.get(name)
        if limit is not None and limit.get("lower") and limit.get("upper"):
            # Repeat the URDF's limits here so the controller manager enforces them too; a command
            # that only MuJoCo rejects would be silently clamped with nothing said about it.
            ET.SubElement(command, "param", {"name": "min"}).text = limit.get("lower")
            ET.SubElement(command, "param", {"name": "max"}).text = limit.get("upper")
        ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(joint, "state_interface", {"name": "velocity"})

    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write(ET.tostring(root, encoding="unicode"))


def add_collision_excludes(mjcf: str, srdf_path: str, verbose: bool = True) -> str:
    """Turn the SRDF's disable_collisions list into MJCF <contact><exclude> pairs.

    The URDF's collision geometry overlaps at every joint -- that is normal, because MoveIt never
    checks adjacent links against each other; the SRDF says which pairs to skip. MuJoCo has no idea
    about any of that, so it sees a robot permanently intersecting itself and applies contact forces
    to push it apart. Measured on this arm at its home pose: J4 into J2 by 106 mm, base into J2 by
    56 mm. The result is an arm that flies away from the pose it was told to hold and then sits
    against a joint limit, which reads as a kinematics or gain bug and is neither.

    So the SRDF is the source of truth for both, rather than a second hand-maintained list here.
    """
    import xml.etree.ElementTree as SrdfET

    root = ET.fromstring(mjcf)
    pairs: list[tuple[str, str]] = []
    try:
        srdf = SrdfET.parse(srdf_path).getroot()
    except (OSError, SrdfET.ParseError) as error:
        if verbose:
            print(f"warning: cannot read {srdf_path} ({error}); self-collision left ON, which will "
                  f"almost certainly throw the arm around", file=sys.stderr)
        return mjcf

    for element in srdf.iter("disable_collisions"):
        a, b = element.get("link1"), element.get("link2")
        if a and b:
            pairs.append((a, b))

    for existing in root.findall("contact"):
        root.remove(existing)
    if not pairs:
        return ET.tostring(root, encoding="unicode")

    bodies = {b.get("name") for b in root.iter("body")}
    contact = ET.SubElement(root, "contact")
    written, skipped = 0, []
    for a, b in pairs:
        # A pair naming a link MuJoCo welded away is not an error; it just has nothing to exclude.
        if a in bodies and b in bodies:
            ET.SubElement(contact, "exclude", {"body1": a, "body2": b})
            written += 1
        else:
            skipped.append(f"{a}/{b}")
    if verbose:
        print(f"self-collision: excluded {written} pair(s) from the SRDF"
              + (f"; {len(skipped)} name links not in the model ({skipped})" if skipped else ""),
              file=sys.stderr)
    return ET.tostring(root, encoding="unicode")


def add_scene(mjcf: str, floor: bool = True) -> str:
    """Give the model a floor, lights and a camera pointed at the robot.

    A URDF describes a robot, not a room, so the compiled model has no ground plane, no light and
    no camera -- MuJoCo's viewer then shows a grey arm floating in a black void, lit only by the
    default headlight. None of this affects the physics of a fixed-base arm; it is what makes the
    window worth looking at, and the floor gives depth cues that make a pose readable.

    The floor is contype/conaffinity 0 -- visual only. A real collidable floor would be a lie about
    where this arm is mounted, and would let a planned motion be blocked by scenery the planner on
    the ROS side knows nothing about.
    """
    root = ET.fromstring(mjcf)

    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "headlight", {"ambient": "0.4 0.4 0.4", "diffuse": "0.7 0.7 0.7",
                                        "specular": "0.1 0.1 0.1"})
    ET.SubElement(visual, "rgba", {"haze": "0.15 0.25 0.35 1"})

    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", {"name": "grid", "type": "2d", "builtin": "checker",
                                     "rgb1": "0.22 0.24 0.28", "rgb2": "0.16 0.18 0.21",
                                     "width": "300", "height": "300"})
    ET.SubElement(asset, "material", {"name": "grid_mat", "texture": "grid",
                                      "texrepeat": "6 6", "reflectance": "0.05"})

    world = root.find("worldbody")
    if world is None:
        world = ET.SubElement(root, "worldbody")

    ET.SubElement(world, "light", {"pos": "1.2 -1.2 2.2", "dir": "-0.4 0.4 -1",
                                   "directional": "true", "diffuse": "0.5 0.5 0.5"})
    if floor:
        ET.SubElement(world, "geom", {
            "name": "floor", "type": "plane", "size": "3 3 0.05", "pos": "0 0 0",
            "material": "grid_mat", "contype": "0", "conaffinity": "0",
        })
    # Framed on the workspace rather than the origin: the arm reaches forward in +X, so a camera at
    # the origin looks at the base and misses the interesting half.
    ET.SubElement(world, "camera", {
        "name": "overview", "pos": "1.4 -1.3 1.1", "xyaxes": "0.68 0.73 0 -0.28 0.26 0.92",
    })
    ET.SubElement(world, "camera", {
        "name": "wrist_view", "pos": "0.9 -0.9 0.9", "xyaxes": "0.7 0.7 0 -0.35 0.35 0.87",
    })
    return ET.tostring(root, encoding="unicode")


def revolute_joint_names(urdf_path: str) -> list[str]:
    root = ET.parse(urdf_path).getroot()
    return [j.get("name", "") for j in root.findall("joint") if j.get("type") in ("revolute", "continuous")]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--out", help="write MJCF here; default is stdout")
    parser.add_argument("--urdf-out", help="also write the prepared URDF, for debugging")
    parser.add_argument("--tolerance", type=float, default=0.02,
                        help="rad of sag to allow under gravity; kp is sized from this")
    parser.add_argument("--armature", type=float, default=0.1,
                        help="reflected rotor inertia per joint (kg m^2). A geared arm has this; "
                             "URDF cannot express it and MuJoCo defaults it to zero")
    parser.add_argument("--damping-ratio", type=float, default=1.0)
    parser.add_argument("--no-scene", action="store_true",
                        help="skip the floor, lights and cameras (physics is unaffected either way)")
    parser.add_argument("--srdf",
                        help="SRDF whose disable_collisions list becomes MJCF contact excludes. "
                             "Without it the arm self-collides at every pose")
    parser.add_argument("--ros2-control-out",
                        help="write a URDF with a <ros2_control> block for controller_manager")
    parser.add_argument("--keyframe", action="append", default=[],
                        help="name=q1,q2,... a named pose to reset to. Repeatable.")
    args = parser.parse_args()

    urdf_text, _ = prepare(args.urdf)
    if args.urdf_out:
        with open(args.urdf_out, "w", encoding="utf-8") as handle:
            handle.write(urdf_text)

    mjcf = compile_to_mjcf(urdf_text)

    joints = revolute_joint_names(args.urdf)
    keyframes: dict[str, list[float]] = {}
    for spec in args.keyframe:
        name, _, values = spec.partition("=")
        try:
            parsed = [float(v) for v in values.split(",") if v.strip()]
        except ValueError:
            print(f"keyframe '{name}': not a list of numbers", file=sys.stderr)
            return 1
        if len(parsed) != len(joints):
            print(f"keyframe '{name}': {len(parsed)} values for {len(joints)} joints {joints}",
                  file=sys.stderr)
            return 1
        keyframes[name] = parsed
    if args.srdf:
        mjcf = add_collision_excludes(mjcf, args.srdf)
    if not args.no_scene:
        mjcf = add_scene(mjcf)
    mjcf = add_actuators_and_keyframes(mjcf, joints, args.tolerance, args.armature,
                                       args.damping_ratio, keyframes)
    print(f"keyframes: {list(keyframes) or 'none'}", file=sys.stderr)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            handle.write(mjcf)
        print(f"wrote {args.out} ({len(mjcf)} bytes)")

    if args.ros2_control_out:
        if not args.out:
            print("--ros2-control-out needs --out: the tag has to point at the compiled model",
                  file=sys.stderr)
            return 1
        write_ros2_control_urdf(args.urdf, os.path.abspath(args.out), joints,
                                next(iter(keyframes), ""), args.ros2_control_out)
        print(f"wrote {args.ros2_control_out}")
    else:
        print(mjcf)
    return 0


if __name__ == "__main__":
    sys.exit(main())
