// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_core/bt_compat.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <string>

namespace moveit2_extended::behaviors
{

/** Express a goal given for `target_link` as a goal for `ik_link`.
 *
 *  THIS IS NOT OPTIONAL ON THIS ROBOT, and it is the single easiest way to be badly wrong here.
 *
 *  The CRX-5iA's SRDF names `cutting_point` as the planning group's tip, but
 *  crx_kinematics/CRXKinematicsPlugin always solves to `flange` and ignores both the SRDF tip and
 *  the ik_link_name field of /compute_ik. Its own kinematics.yaml says so. So a Behavior that
 *  constrains `cutting_point` directly gets `flange` put there instead -- the whole arm lands
 *  (0.150, 0, 0.028) away from the requested pose, and nothing reports an error, because from the
 *  solver's point of view it succeeded.
 *
 *  The fix is arithmetic, not configuration: take the fixed transform from ik_link to target_link
 *  out of the robot model and pre-multiply it away. Both links must be in the model and rigidly
 *  connected; if a joint between them can move, this is meaningless and the function says so.
 *
 *  Returns the pose `ik_link` must reach so that `target_link` ends up at `target_pose`. */
BtExpected<geometry_msgs::msg::PoseStamped> retargetPose(const moveit::core::RobotModelConstPtr& model,
                                                         const geometry_msgs::msg::PoseStamped& target_pose,
                                                         const std::string& target_link,
                                                         const std::string& ik_link);

/** Resolve a target robot state from either an SRDF named state or an explicit JointState.
 *
 *  `named_state` wins when both are given, because an Objective that says `named_state="home"`
 *  means it. The result is seeded from `reference`, so joints the caller did not mention keep
 *  their current values rather than snapping to zero -- on this arm the zero pose is a
 *  singularity, which is exactly why the SRDF defines `home` separately. */
BtExpected<moveit::core::RobotState> resolveTargetState(const moveit::core::RobotModelConstPtr& model,
                                                        const moveit::core::RobotState& reference,
                                                        const std::string& group, const std::string& named_state,
                                                        const sensor_msgs::msg::JointState& joint_state);

/** Joint-space goal constraints for `group` at `goal_state`. */
BtExpected<moveit_msgs::msg::Constraints> makeJointGoal(const moveit::core::RobotState& goal_state,
                                                        const std::string& group, double tolerance);

/** Pose goal constraints on `ik_link`. The pose must already have been retargeted. */
moveit_msgs::msg::Constraints makePoseGoal(const geometry_msgs::msg::PoseStamped& pose, const std::string& ik_link,
                                           double position_tolerance, double orientation_tolerance);

/** The last waypoint of a trajectory, as a RobotState message.
 *
 *  This is what makes plan-ahead chaining work without MoveIt Task Constructor: feed stage n's
 *  end_state into stage n+1's start_state and the whole approach/cut/retreat is planned before
 *  anything executes. MotionPlanRequest honours start_state, so no extra machinery is needed. */
BtExpected<moveit_msgs::msg::RobotState> finalStateOf(const moveit::core::RobotModelConstPtr& model,
                                                      const moveit::core::RobotState& reference,
                                                      const moveit_msgs::msg::RobotTrajectory& trajectory);

}  // namespace moveit2_extended::behaviors
