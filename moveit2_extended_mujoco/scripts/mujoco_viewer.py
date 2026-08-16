#!/usr/bin/env python3
# Copyright 2026 Leow Chee Siang. Apache-2.0.
"""Show the simulated arm in MuJoCo's own viewer, and optionally publish what its camera sees.

    ros2 run moveit2_extended_mujoco mujoco_viewer.py --ros-args -p mjcf:=/tmp/crx5ia.mjcf.xml

WHAT THIS IS, precisely: the physics runs inside the ros2_control hardware interface, in another
process. This viewer loads the same model and drives it from /joint_states, so what you see is
MuJoCo rendering the real simulated state -- but it is a MIRROR, not the simulation itself.
Contacts and forces shown here are recomputed from the pose, so they agree with the running sim
about where the arm is and should not be trusted about what it is touching.

Doing it this way rather than rendering inside the hardware interface is deliberate: a render loop
in the control thread would tie the simulation rate to the frame rate, and a stalled window would
stall the robot.

With --camera it also renders offscreen and publishes sensor_msgs/Image, which is the part of
MuJoCo worth having beyond the picture: a camera that sees the simulated scene, on a machine with
no GPU to spare.
"""

from __future__ import annotations

import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState


class MujocoViewer(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_viewer")

        self.declare_parameter("mjcf", "")
        self.declare_parameter("camera", "")
        self.declare_parameter("camera_topic", "~/image_raw")
        self.declare_parameter("camera_rate", 10.0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)

        path = self.get_parameter("mjcf").value
        if not path:
            raise RuntimeError("set the 'mjcf' parameter to a compiled MuJoCo model")

        import mujoco

        self._mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        self._lock = threading.Lock()

        # Joint name -> index into qpos, so a JointState in any order lands in the right place.
        self.qpos_index: dict[str, int] = {}
        for index in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, index)
            if name:
                self.qpos_index[name] = self.model.jnt_qposadr[index]

        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        self._renderer = None
        camera = self.get_parameter("camera").value
        if camera:
            self._start_camera(camera)

        self.get_logger().info(
            f"mirroring {path}: {self.model.njnt} joints, {self.model.ngeom} geoms"
            + (f", camera '{camera}'" if camera else "")
        )

    def _on_joint_states(self, message: JointState) -> None:
        with self._lock:
            for name, position in zip(message.name, message.position):
                index = self.qpos_index.get(name)
                if index is not None:
                    self.data.qpos[index] = position
            # forward, not step: the pose comes from the running simulation, so integrating here
            # would fight it and drift.
            self._mujoco.mj_forward(self.model, self.data)

    def _start_camera(self, camera: str) -> None:
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        self._renderer = self._mujoco.Renderer(self.model, height=height, width=width)
        publisher = self.create_publisher(Image, self.get_parameter("camera_topic").value, 1)
        period = 1.0 / max(float(self.get_parameter("camera_rate").value), 0.1)

        def publish() -> None:
            with self._lock:
                try:
                    self._renderer.update_scene(self.data, camera=camera)
                except Exception as error:  # noqa: BLE001
                    self.get_logger().error(f"camera '{camera}': {error}", once=True)
                    return
                frame = self._renderer.render()
            message = Image()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = camera
            message.height, message.width = frame.shape[0], frame.shape[1]
            message.encoding = "rgb8"
            message.step = frame.shape[1] * 3
            message.data = frame.tobytes()
            publisher.publish(message)

        self.create_timer(period, publish)

    def run_viewer(self) -> None:
        """Block in MuJoCo's interactive window until it is closed."""
        import mujoco.viewer

        with mujoco.viewer.launch_passive(self.model, self.data,
                                          show_left_ui=True, show_right_ui=False) as viewer:
            while viewer.is_running() and rclpy.ok():
                with self._lock:
                    viewer.sync()
                time.sleep(1.0 / 60.0)


def main() -> int:
    rclpy.init()
    try:
        node = MujocoViewer()
    except Exception as error:  # noqa: BLE001
        print(f"mujoco_viewer: {error}", file=sys.stderr)
        rclpy.shutdown()
        return 1

    # ROS spins on a thread; the viewer must own the main thread, because the windowing library
    # will not create a context anywhere else.
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        node.run_viewer()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
