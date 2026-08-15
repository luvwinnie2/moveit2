#!/usr/bin/env python3
# Copyright 2026 Leow Chee Siang. Apache-2.0.
"""Stream this robot into Rerun, so the dresspack can be scrubbed along the trajectory's timeline.

Why Rerun rather than more RViz: RViz shows the present. The question this stack actually has to
answer is "where along that trajectory did the carrier get tight, and what did the arm look like at
that moment" -- which needs a timeline you can drag, with the 3D scene and the scalar plots moving
together. Rerun gives that, in a browser, without a GPU session per viewer.

What is logged
    world/<link chain>       the arm, as one Asset3D per visual mesh, posed from /tf
    world/carrier/…          the dresspack, from carrier_visualizer's MarkerArray
    plots/joints/<joint>     joint positions
    plots/carrier/<field>    bend utilisation, twist, achieved radius, cable strain
    events                   Objective state changes and Behavior failures

Nothing here is required for the robot to work: this is an observer. Every subscription is
optional, so a topic that is not being published simply produces an empty panel rather than an
error -- carrier_visualizer in particular is not started by studio.launch.py.

Run it:
    ros2 run moveit2_extended_studio rerun_bridge
then open the URL it prints.
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Tuple

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

import rerun as rr

try:
    from moveit2_extended_msgs.msg import BehaviorStatus, BehaviorTreeLog, ObjectiveState
except ImportError:  # pragma: no cover - lets the bridge run against a bare robot
    BehaviorTreeLog = None
    ObjectiveState = None
    BehaviorStatus = None


LATCHED = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

TIMELINE = "ros_time"

STATE_NAMES = {0: "IDLE", 1: "RUNNING", 2: "SUCCEEDED", 3: "FAILED", 4: "CANCELED"}
STATUS_NAMES = {0: "IDLE", 1: "RUNNING", 2: "SUCCESS", 3: "FAILURE", 4: "SKIPPED"}


def resolve_package_uri(uri: str) -> Optional[str]:
    """package://pkg/rest -> absolute path, or None if the package is not installed."""
    if not uri.startswith("package://"):
        return uri if os.path.exists(uri) else None
    remainder = uri[len("package://") :]
    package, _, relative = remainder.partition("/")
    try:
        share = get_package_share_directory(package)
    except (PackageNotFoundError, KeyError):
        return None
    path = os.path.join(share, relative)
    return path if os.path.exists(path) else None


def parse_xyz(text: Optional[str]) -> Tuple[float, float, float]:
    if not text:
        return (0.0, 0.0, 0.0)
    parts = [float(value) for value in text.split()]
    while len(parts) < 3:
        parts.append(0.0)
    return (parts[0], parts[1], parts[2])


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """URDF fixed-axis RPY -> xyzw. Written out rather than pulled from tf_transformations,
    which is not installed in this container and would be a dependency for nine lines."""
    import math

    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class RobotLayout:
    """Where each link sits in the Rerun entity tree, and which mesh to draw there.

    Rerun composes a Transform3D with its entity's ancestors, which is the same rule TF uses -- so
    if the entity path mirrors the URDF's link tree, logging each /tf message at its child's path
    is all the posing that is needed. Get the hierarchy wrong and every link is drawn relative to
    the world, which looks like an exploded diagram.
    """

    def __init__(self, urdf_text: str, root_entity: str) -> None:
        self.root_entity = root_entity
        self.parent_of: Dict[str, str] = {}
        self.visuals: Dict[str, List[Tuple[str, Tuple[float, float, float], Tuple[float, float, float, float], Tuple[float, float, float]]]] = {}
        self.missing_meshes: List[str] = []

        tree = ET.fromstring(urdf_text)
        for joint in tree.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is not None and child is not None:
                self.parent_of[child.get("link", "")] = parent.get("link", "")

        for link in tree.findall("link"):
            name = link.get("name", "")
            for visual in link.findall("visual"):
                mesh = visual.find("geometry/mesh")
                if mesh is None:
                    continue
                path = resolve_package_uri(mesh.get("filename", ""))
                if path is None:
                    self.missing_meshes.append(mesh.get("filename", ""))
                    continue
                origin = visual.find("origin")
                xyz = parse_xyz(origin.get("xyz") if origin is not None else None)
                rpy = parse_xyz(origin.get("rpy") if origin is not None else None)
                scale = parse_xyz(mesh.get("scale")) if mesh.get("scale") else (1.0, 1.0, 1.0)
                self.visuals.setdefault(name, []).append((path, xyz, rpy_to_quaternion(*rpy), scale))

        self._entity_cache: Dict[str, str] = {}

    def entity(self, link: str) -> str:
        """Full entity path for a link, walking up the URDF joint tree."""
        cached = self._entity_cache.get(link)
        if cached is not None:
            return cached

        chain: List[str] = []
        seen = set()
        current = link
        while current and current not in seen:
            seen.add(current)
            chain.append(current)
            current = self.parent_of.get(current, "")
        chain.reverse()
        path = "/".join([self.root_entity] + chain)
        self._entity_cache[link] = path
        return path


class RerunBridge(Node):
    def __init__(self) -> None:
        super().__init__("rerun_bridge")

        self.declare_parameter("application_id", "moveit2_extended")
        self.declare_parameter("root_entity", "world")
        self.declare_parameter("serve_web_viewer", True)
        self.declare_parameter("web_port", 9090)
        self.declare_parameter("grpc_port", 9876)

        self.root_entity = self.get_parameter("root_entity").value
        self.layout: Optional[RobotLayout] = None
        self.logged_meshes = False

        rr.init(self.get_parameter("application_id").value, spawn=False)
        url = rr.serve_grpc(grpc_port=int(self.get_parameter("grpc_port").value))
        if self.get_parameter("serve_web_viewer").value:
            rr.serve_web_viewer(
                web_port=int(self.get_parameter("web_port").value),
                open_browser=False,
                connect_to=url,
            )
            self.get_logger().info(
                f"Rerun viewer on http://localhost:{self.get_parameter('web_port').value} "
                f"(data on {url})"
            )
        else:
            self.get_logger().info(f"Rerun gRPC endpoint: {url}")

        rr.send_blueprint(self._blueprint())

        # The URDF is latched, so this arrives once, shortly after start-up.
        self.create_subscription(String, "/robot_description", self.on_robot_description, LATCHED)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        self.create_subscription(TFMessage, "/tf_static", self.on_tf_static, LATCHED)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.create_subscription(MarkerArray, "/cable_carrier_markers", self.on_markers, 1)
        self.create_subscription(String, "/cable_carrier_status", self.on_carrier_status, LATCHED)

        if ObjectiveState is not None:
            self.create_subscription(
                ObjectiveState, "/objective_server/objective_state", self.on_objective_state, LATCHED
            )
            self.create_subscription(
                BehaviorTreeLog, "/objective_server/behavior_tree_log", self.on_tree_log, 20
            )
        else:
            self.get_logger().warning(
                "moveit2_extended_msgs not importable: Objective events will not be logged"
            )

    # -- blueprint ------------------------------------------------------------------------------

    def _blueprint(self):
        """The layout, fixed in code so every operator opens the same view.

        3D on the left because it is what you look at; the carrier plots directly under the joint
        plots on the right, because the question is always "which joint was moving when the bend
        went up".
        """
        import rerun.blueprint as rrb

        return rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(name="Robot", origin=f"/{self.root_entity}"),
                rrb.Vertical(
                    rrb.TimeSeriesView(name="Carrier", origin="/plots/carrier"),
                    rrb.TimeSeriesView(name="Joints", origin="/plots/joints"),
                    rrb.TextLogView(name="Events", origin="/events"),
                    row_shares=[2, 2, 1],
                ),
                column_shares=[3, 2],
            ),
            collapse_panels=True,
        )

    # -- time -----------------------------------------------------------------------------------

    def stamp(self, header_stamp) -> None:
        """Put everything on one ROS timeline so the 3D view and the plots scrub together."""
        rr.set_time(TIMELINE, timestamp=header_stamp.sec + header_stamp.nanosec * 1e-9)

    def stamp_now(self) -> None:
        now = self.get_clock().now().to_msg()
        rr.set_time(TIMELINE, timestamp=now.sec + now.nanosec * 1e-9)

    # -- robot ----------------------------------------------------------------------------------

    def on_robot_description(self, message: String) -> None:
        if self.logged_meshes:
            return
        try:
            self.layout = RobotLayout(message.data, self.root_entity)
        except ET.ParseError as error:
            self.get_logger().error(f"robot_description is not parseable: {error}")
            return

        for link, visuals in self.layout.visuals.items():
            base = self.layout.entity(link)
            for index, (path, xyz, quat, scale) in enumerate(visuals):
                entity = f"{base}/visual_{index}"
                # The visual's own origin is a child entity rather than folded into the link's
                # transform: /tf owns the link transform and would overwrite it on the next tick.
                rr.log(entity, rr.Transform3D(translation=xyz, quaternion=rr.Quaternion(xyzw=quat), scale=scale), static=True)
                rr.log(entity, rr.Asset3D(path=path), static=True)

        self.logged_meshes = True
        self.get_logger().info(
            f"logged {sum(len(v) for v in self.layout.visuals.values())} meshes for "
            f"{len(self.layout.visuals)} links"
        )
        for missing in self.layout.missing_meshes:
            self.get_logger().warning(f"mesh not found, skipped: {missing}")

    def _log_transform(self, transform: TransformStamped) -> None:
        if self.layout is None:
            return
        entity = self.layout.entity(transform.child_frame_id)
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rr.log(
            entity,
            rr.Transform3D(
                translation=[translation.x, translation.y, translation.z],
                quaternion=rr.Quaternion(xyzw=[rotation.x, rotation.y, rotation.z, rotation.w]),
            ),
        )

    def on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            self.stamp(transform.header.stamp)
            self._log_transform(transform)

    def on_tf_static(self, message: TFMessage) -> None:
        for transform in message.transforms:
            self.stamp(transform.header.stamp)
            self._log_transform(transform)

    def on_joint_states(self, message: JointState) -> None:
        self.stamp(message.header.stamp)
        for name, position in zip(message.name, message.position):
            rr.log(f"plots/joints/{name}", rr.Scalars(position))

    # -- carrier --------------------------------------------------------------------------------

    def on_markers(self, message: MarkerArray) -> None:
        for marker in message.markers:
            self.stamp(marker.header.stamp)
            entity = f"{self.root_entity}/carrier/{marker.ns or 'default'}/{marker.id}"

            if marker.action == Marker.DELETE or marker.action == Marker.DELETEALL:
                rr.log(entity, rr.Clear(recursive=True))
                continue

            colour = [
                int(marker.color.r * 255),
                int(marker.color.g * 255),
                int(marker.color.b * 255),
                int(marker.color.a * 255),
            ]
            points = [[p.x, p.y, p.z] for p in marker.points]

            if marker.type in (Marker.LINE_STRIP, Marker.LINE_LIST) and points:
                strips = [points] if marker.type == Marker.LINE_STRIP else [
                    points[i : i + 2] for i in range(0, len(points) - 1, 2)
                ]
                rr.log(entity, rr.LineStrips3D(strips, colors=[colour], radii=max(marker.scale.x, 1e-4) * 0.5))
            elif marker.type in (Marker.SPHERE_LIST, Marker.POINTS, Marker.CUBE_LIST) and points:
                rr.log(entity, rr.Points3D(points, colors=[colour], radii=max(marker.scale.x, 1e-4) * 0.5))
            elif marker.type == Marker.SPHERE:
                position = marker.pose.position
                rr.log(
                    entity,
                    rr.Points3D([[position.x, position.y, position.z]], colors=[colour],
                                radii=max(marker.scale.x, 1e-4) * 0.5),
                )
            # Other marker types are skipped rather than approximated: a cylinder drawn as a point
            # would be quietly wrong about where the carrier is.

    def on_carrier_status(self, message: String) -> None:
        """carrier_visualizer publishes "key=value;key=value". These four are the ones that decide
        whether a design works, so they get the time series."""
        self.stamp_now()
        for field in message.data.split(";"):
            key, _, value = field.partition("=")
            if not key or not value:
                continue
            try:
                rr.log(f"plots/carrier/{key}", rr.Scalars(float(value)))
            except ValueError:
                continue  # a text field such as the cable's name

    # -- objectives -----------------------------------------------------------------------------

    def on_objective_state(self, message) -> None:
        self.stamp(message.stamp)
        state = STATE_NAMES.get(message.state, str(message.state))
        level = "ERROR" if state == "FAILED" else "INFO"
        detail = f" [{message.current_behavior}]" if message.current_behavior else ""
        rr.log("events", rr.TextLog(f"{state}  {message.objective_name}{detail}", level=level))

    def on_tree_log(self, message) -> None:
        self.stamp(message.stamp)
        for event in message.event_log:
            # Only failures. Every transition is thousands of lines a minute and would bury the one
            # line that says why the Objective went down its recovery branch.
            if event.current_status != BehaviorStatus.FAILURE:
                continue
            rr.log(
                "events",
                rr.TextLog(
                    f"FAILURE  {event.instance_name} ({event.registration_name})",
                    level="WARN",
                ),
            )


def main() -> None:
    rclpy.init()
    node = RerunBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
