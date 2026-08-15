#!/usr/bin/env python3
# Copyright 2026 Leow Chee Siang. Apache-2.0.
"""cuRobo behind two ROS services, so the Behavior layer can use the GPU without being Python.

cuRobo is Python-only and the Behaviors are C++, so the seam has to be a service. That is not
purely a cost: it also means the GPU is optional at run time. A site without one simply does not
start this node, the GPU Behaviors report that plainly, and the Objective's Fallback drops to the
OMPL path.

    ros2 run moveit2_extended_gpu curobo_service.py

What it answers
    ~/solve_ik_batch   many IK queries at once -- the query shape a GPU is actually good at
    ~/plan_to_goal     one trajectory, joint or pose goal, as a plain moveit_msgs/RobotTrajectory

TWO THINGS THAT WILL BITE, both measured on this robot rather than assumed:

1. cuRobo's `success` flag is not "the answer is good enough". It requires the OPTIMISER to have
   converged, and a solution 0.67 mm and 0.0016 rad from the goal is still reported as a failure --
   with `success_requires_convergence=False` set, it is STILL reported as a failure. So this node
   ignores that flag entirely and judges on position_error / rotation_error against the requested
   tolerances, which is the question the caller actually asked.

2. Batch size is fixed when the solver is built (`max_batch_size`, default 1), not per call. A
   larger batch raises ValueError rather than being split, so the solver is rebuilt when a request
   exceeds the current size -- expensive, and logged when it happens, because rebuilding on every
   call would be far slower than doing IK on the CPU.
"""

from __future__ import annotations

import os
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState, RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from moveit2_extended_msgs.srv import PlanWithGpu, SolveIKBatch


class CuroboService(Node):
    def __init__(self) -> None:
        super().__init__("curobo_service")

        self.declare_parameter("robot_config", "")
        self.declare_parameter("tool_frame", "tool0")
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("position_tolerance", 0.005)
        self.declare_parameter("orientation_tolerance", 0.05)
        self.declare_parameter("num_seeds", 32)
        self.declare_parameter("initial_batch_size", 128)
        self.declare_parameter("plan_timeout", 10.0)

        self.robot_config = self.get_parameter("robot_config").value
        if not self.robot_config or not os.path.exists(self.robot_config):
            raise RuntimeError(
                f"robot_config must point at a cuRobo robot YAML; got '{self.robot_config}'"
            )
        self.tool_frame = self.get_parameter("tool_frame").value

        # Everything below touches CUDA. One lock, and services on one callback group, because two
        # threads sharing a cuRobo solver is undefined behaviour rather than a slowdown.
        self._lock = threading.Lock()
        self._group = MutuallyExclusiveCallbackGroup()

        self._ik = None
        self._ik_batch_size = 0
        self._planner = None
        self._joint_names: List[str] = []

        self._current_state: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        self.create_service(SolveIKBatch, "~/solve_ik_batch", self._on_solve_ik,
                            callback_group=self._group)
        self.create_service(PlanWithGpu, "~/plan_to_goal", self._on_plan,
                            callback_group=self._group)

        self.get_logger().info(f"loading cuRobo from {self.robot_config}")
        self._load()
        self.get_logger().info(
            f"cuRobo ready: joints {self._joint_names}, tool frame '{self.tool_frame}'"
        )

    # -- cuRobo lifecycle -------------------------------------------------------------------

    def _load(self) -> None:
        # Imported here rather than at module scope so the node's own error message is what a user
        # sees when torch or cuRobo is missing, instead of an ImportError traceback from rclpy.
        import torch  # noqa: F401
        from curobo.kinematics import Kinematics, KinematicsCfg

        kinematics = Kinematics(KinematicsCfg.from_robot_yaml_file(self.robot_config))
        self._joint_names = list(kinematics.joint_names)
        if self.tool_frame not in kinematics.tool_frames:
            raise RuntimeError(
                f"tool_frame '{self.tool_frame}' is not one of the robot's tool frames "
                f"{kinematics.tool_frames}"
            )
        self._ensure_ik(int(self.get_parameter("initial_batch_size").value))

    def _ensure_ik(self, batch_size: int) -> None:
        """Build (or rebuild) the IK solver big enough for `batch_size`."""
        if self._ik is not None and batch_size <= self._ik_batch_size:
            return
        from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg

        # Grow in steps rather than to the exact size, so a caller that creeps up by one does not
        # pay a rebuild every time.
        size = max(batch_size, self._ik_batch_size * 2, 32)
        if self._ik is not None:
            self.get_logger().warning(
                f"rebuilding the IK solver for a batch of {batch_size} (was sized {self._ik_batch_size}); "
                f"set initial_batch_size to avoid this"
            )
        self._ik = InverseKinematics(
            InverseKinematicsCfg.create(
                robot=self.robot_config,
                num_seeds=int(self.get_parameter("num_seeds").value),
                position_tolerance=float(self.get_parameter("position_tolerance").value),
                orientation_tolerance=float(self.get_parameter("orientation_tolerance").value),
                max_batch_size=size,
                success_requires_convergence=False,
            )
        )
        self._ik_batch_size = size

    def _ensure_planner(self):
        if self._planner is None:
            from curobo.motion_planner import MotionPlanner, MotionPlannerCfg

            self.get_logger().info("building the motion planner (the first plan compiles kernels)")
            self._planner = MotionPlanner(MotionPlannerCfg.create(robot=self.robot_config))
            # NOT warmup(). It captures a CUDA graph for the shape it warms with, and the first
            # real plan then dies with "CUDA graph reset is not available". The cost of skipping it
            # is that the first plan pays the compile instead: measured 6.2 s once, then 177 ms.
        return self._planner

    # -- state ------------------------------------------------------------------------------

    def _on_joint_states(self, message: JointState) -> None:
        self._current_state = message

    def _seed_positions(self, seed: JointState):
        """Joint values in cuRobo's order, from the request's seed or the live robot."""
        source = seed if seed.name else self._current_state
        if source is None or not source.name:
            return None
        by_name = dict(zip(source.name, source.position))
        if not all(name in by_name for name in self._joint_names):
            return None
        return [float(by_name[name]) for name in self._joint_names]

    # -- IK ---------------------------------------------------------------------------------

    def _on_solve_ik(self, request: SolveIKBatch.Request, response: SolveIKBatch.Response):
        if not request.poses:
            response.success = False
            response.message = "no poses given"
            return response

        position_tolerance = request.position_tolerance or float(
            self.get_parameter("position_tolerance").value
        )
        orientation_tolerance = request.orientation_tolerance or float(
            self.get_parameter("orientation_tolerance").value
        )

        try:
            import torch
            from curobo.types import GoalToolPose, Pose

            with self._lock:
                self._ensure_ik(len(request.poses))

                count = len(request.poses)
                positions = torch.zeros(count, 3, device="cuda:0")
                quaternions = torch.zeros(count, 4, device="cuda:0")
                for index, stamped in enumerate(request.poses):
                    p = stamped.pose.position
                    q = stamped.pose.orientation
                    positions[index] = torch.tensor([p.x, p.y, p.z], device="cuda:0")
                    # cuRobo orders quaternions wxyz; ROS orders them xyzw. Getting this backwards
                    # produces answers that look plausible and are rotated by a right angle.
                    quaternions[index] = torch.tensor([q.w, q.x, q.y, q.z], device="cuda:0")

                goal = GoalToolPose.from_poses(
                    {self.tool_frame: Pose(position=positions, quaternion=quaternions)}
                )
                started = time.time()
                result = self._ik.solve_pose(goal_tool_poses=goal)
                torch.cuda.synchronize()
                elapsed = time.time() - started

                solution = result.solution.detach().cpu().numpy().reshape(count, -1)
                position_error = result.position_error.detach().cpu().numpy().reshape(-1)
                rotation_error = result.rotation_error.detach().cpu().numpy().reshape(-1)
        except Exception as error:  # noqa: BLE001 - a service must answer, not die
            response.success = False
            response.message = f"cuRobo IK failed: {error}"
            self.get_logger().error(response.message)
            return response

        for index in range(count):
            state = JointState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.name = list(self._joint_names)
            state.position = [float(value) for value in solution[index]]
            response.solutions.append(state)
            response.position_error.append(float(position_error[index]))
            response.orientation_error.append(float(rotation_error[index]))
            # Judged here, not by cuRobo's success flag -- see the module docstring.
            response.solved.append(
                bool(position_error[index] <= position_tolerance
                     and rotation_error[index] <= orientation_tolerance)
            )

        response.success = True
        response.solve_time = elapsed
        solved = sum(1 for ok in response.solved if ok)
        response.message = (
            f"{solved}/{count} within {1000 * position_tolerance:.1f} mm and "
            f"{orientation_tolerance:.3f} rad, in {1000 * elapsed:.1f} ms"
        )
        return response

    # -- planning ---------------------------------------------------------------------------

    def _on_plan(self, request: PlanWithGpu.Request, response: PlanWithGpu.Response):
        start = self._seed_positions(request.start_state.joint_state)
        if start is None:
            response.success = False
            response.message = "no start state: give one, or wait for /joint_states"
            return response

        try:
            import torch
            from curobo.types import GoalToolPose, JointState as CuJointState, Pose

            with self._lock:
                planner = self._ensure_planner()
                start_state = CuJointState.from_position(
                    torch.tensor([start], device="cuda:0"), joint_names=self._joint_names
                )
                started = time.time()

                if request.goal_type == PlanWithGpu.Request.JOINT_GOAL:
                    by_name = dict(zip(request.joint_goal.name, request.joint_goal.position))
                    missing = [n for n in self._joint_names if n not in by_name]
                    if missing:
                        raise ValueError(f"joint_goal is missing {missing}")
                    goal_state = CuJointState.from_position(
                        torch.tensor([[float(by_name[n]) for n in self._joint_names]], device="cuda:0"),
                        joint_names=self._joint_names,
                    )
                    # GOAL FIRST. cuRobo 0.8's signature is
                    # plan_cspace(goal_state, current_state) -- the opposite way round from every
                    # other planner API, including plan_pose below. Getting it backwards does not
                    # raise: it returns a perfectly valid trajectory that runs the arm from the
                    # goal to where it already is. Caught by checking the first point, below.
                    result = planner.plan_cspace(goal_state, start_state)
                else:
                    p = request.pose_goal.pose.position
                    q = request.pose_goal.pose.orientation
                    goal = GoalToolPose.from_poses({
                        self.tool_frame: Pose(
                            position=torch.tensor([[p.x, p.y, p.z]], device="cuda:0"),
                            quaternion=torch.tensor([[q.w, q.x, q.y, q.z]], device="cuda:0"),
                        )
                    })
                    # Goal first here too -- plan_pose(goal_tool_poses, current_state).
                    result = planner.plan_pose(goal, start_state)

                torch.cuda.synchronize()
                elapsed = time.time() - started
                trajectory = _extract_trajectory(result)
        except Exception as error:  # noqa: BLE001
            response.success = False
            response.message = f"cuRobo planning failed: {error}"
            self.get_logger().error(response.message)
            return response

        if trajectory is None:
            response.success = False
            response.message = "cuRobo returned no trajectory"
            response.planning_time = elapsed
            return response

        positions, times = trajectory

        # The trajectory must START where the arm is. This is not paranoia: cuRobo's plan_cspace
        # takes the goal as its FIRST argument, so passing (start, goal) -- which is what every
        # other planner wants -- silently returns the path reversed, and handing that to a
        # controller drives the arm backwards at full speed. A reversed plan is indistinguishable
        # from a correct one by every other check, so it is checked here explicitly.
        drift = max(abs(float(a) - float(b)) for a, b in zip(positions[0], start))
        if drift > 0.05:
            response.success = False
            response.message = (
                f"cuRobo returned a trajectory starting {drift:.3f} rad from the requested start "
                f"state -- refusing it, because running it would move the arm somewhere nobody asked"
            )
            response.planning_time = elapsed
            self.get_logger().error(response.message)
            return response

        message = RobotTrajectory()
        message.joint_trajectory.joint_names = list(self._joint_names)
        for values, stamp in zip(positions, times):
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in values]
            point.time_from_start.sec = int(stamp)
            point.time_from_start.nanosec = int((stamp - int(stamp)) * 1e9)
            message.joint_trajectory.points.append(point)

        response.success = True
        response.trajectory = message
        response.planning_time = elapsed
        response.message = f"{len(positions)} points in {1000 * elapsed:.1f} ms"

        end = RobotState()
        end.joint_state.name = list(self._joint_names)
        end.joint_state.position = [float(value) for value in positions[-1]]
        response.end_state = end
        return response


def _extract_trajectory(result):
    """cuRobo's plan result, as (positions, times).

    get_interpolated_plan() is the one to read: `solution` is the optimiser's 16 knot points and
    `interpolated_trajectory` is a fixed 5000-sample buffer that is mostly padding. The interpolated
    plan is the actual path, and motion_time() is how long it takes -- which is what turns it into a
    time-parameterised ROS trajectory rather than a list of poses.
    """
    if not bool(result.success.detach().cpu().reshape(-1)[0]):
        return None

    plan = result.get_interpolated_plan()
    positions = plan.position.detach().cpu().numpy()
    while positions.ndim > 2:  # (1, 1, steps, dof) -> (steps, dof)
        positions = positions[0]
    if len(positions) < 2:
        return None

    motion_time = result.motion_time
    if callable(motion_time):
        motion_time = motion_time()
    try:
        duration = float(motion_time.detach().cpu().reshape(-1)[0])
    except AttributeError:
        duration = float(motion_time)
    if duration <= 0.0:
        # Untimed would be refused by ExecuteTrajectory, and rightly so. Better to say the plan is
        # unusable than to hand over something the controller will run as fast as it can.
        return None

    step = duration / (len(positions) - 1)
    return positions, [index * step for index in range(len(positions))]


def main() -> None:
    rclpy.init()
    try:
        node = CuroboService()
    except Exception as error:  # noqa: BLE001
        print(f"curobo_service: {error}")
        rclpy.shutdown()
        raise SystemExit(1)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
