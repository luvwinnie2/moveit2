#!/usr/bin/env python3
# Copyright 2026 Leow Chee Siang. Apache-2.0.
"""FollowJointTrajectory server that replays a plan onto /joint_states.

Without a controller, MoveIt can plan but "Execute" has nothing to talk to, so a carrier-aware
plan can never be watched actually running. This node stands in for one in the offline demo: it
owns /joint_states, holds the current pose, and on an execution goal walks the trajectory in real
time. The carrier follows because it tracks /joint_states, so what you see moving is the arm going
through the motion that was planned *with the carrier in collision*.

It is not a controller. There is no feedback, no tolerance checking and no dynamics -- it replays
positions. On the real robot, and in Isaac, crx5ia_traj_bridge.py does this job properly against
the actual hardware interface, and this node must not be running at the same time: two publishers
on /joint_states fight and the arm jitters between them.

    ros2 run moveit_cable_carrier demo_trajectory_executor.py --ros-args \
        -p joints:="[J1,J2,J3,J4,J5,J6]"
"""

from __future__ import annotations

import threading

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState


class DemoTrajectoryExecutor(Node):
    def __init__(self) -> None:
        super().__init__("demo_trajectory_executor")

        self.joints: list[str] = self.declare_parameter(
            "joints", ["J1", "J2", "J3", "J4", "J5", "J6"]
        ).value
        initial = self.declare_parameter("initial_positions", [0.0] * len(self.joints)).value
        action_name = self.declare_parameter(
            "action_name", "/arm_controller/follow_joint_trajectory"
        ).value
        self.rate = float(self.declare_parameter("publish_rate", 30.0).value)

        if len(initial) != len(self.joints):
            initial = [0.0] * len(self.joints)
        self._lock = threading.Lock()
        self._q = list(float(v) for v in initial)

        self._pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(1.0 / max(1.0, self.rate), self._publish)

        # Reentrant so the publish timer keeps running while a goal is executing; otherwise the
        # arm would freeze for the duration of the motion and nothing would be visible.
        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(
            f"demo executor ready on '{action_name}' for {len(self.joints)} joints"
        )

    def _publish(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joints)
        with self._lock:
            msg.position = list(self._q)
        self._pub.publish(msg)

    def _execute(self, goal_handle):
        traj = goal_handle.request.trajectory
        points = traj.points
        result = FollowJointTrajectory.Result()

        if not points:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory has no points"
            return result

        # Map the goal's joint order onto ours; MoveIt does not promise the same ordering.
        try:
            order = [traj.joint_names.index(name) for name in self.joints]
        except ValueError as exc:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = f"goal is missing a joint: {exc}"
            return result

        self.get_logger().info(f"executing {len(points)} waypoints")
        start = self.get_clock().now()
        for point in points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "cancelled"
                return result

            target = float(point.time_from_start.sec) + point.time_from_start.nanosec * 1e-9
            while True:
                elapsed = (self.get_clock().now() - start).nanoseconds * 1e-9
                if elapsed >= target:
                    break
                self._sleep(min(0.01, target - elapsed))

            with self._lock:
                self._q = [float(point.positions[i]) for i in order]

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def _sleep(self, seconds: float) -> None:
        # rclpy has no blocking sleep that plays well inside an action callback on a
        # MultiThreadedExecutor; a rate object here would deadlock against the publish timer.
        import time

        time.sleep(max(0.0, seconds))


def main() -> None:
    rclpy.init()
    node = DemoTrajectoryExecutor()
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
