// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

// Motion Behaviors, built on move_group's action and service interfaces rather than on
// MoveGroupInterface.
//
// MoveGroupInterface::move() blocks and cannot be cancelled from another thread, so a Behavior
// built on it would have a halt() that cannot halt. Talking to /move_action and
// /execute_trajectory directly gives real cancellation, real feedback, and one uniform base class.
//
// TWO PORTS APPEAR ON EVERY CARTESIAN BEHAVIOR AND ARE NOT DECORATION:
//
//   target_link  the frame the caller means, e.g. cutting_point
//   ik_link      the frame the solver can actually constrain, e.g. flange
//
// On the CRX-5iA these differ, and crx_kinematics silently ignores any request to change the link
// it solves for. See retargetPose() in moveit_goal_helpers.hpp.

#include <moveit2_extended_behaviors/moveit_goal_helpers.hpp>
#include <moveit2_extended_behaviors/port_conversions.hpp>

#include <moveit2_extended_core/action_client_behavior_base.hpp>
#include <moveit2_extended_core/service_client_behavior_base.hpp>
#include <moveit2_extended_core/send_message_to_topic_behavior_base.hpp>

#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <std_msgs/msg/string.hpp>

namespace moveit2_extended::behaviors
{

/** Ports every planning Behavior shares. Kept in one place so an Objective can move a port from
 *  one Behavior to another without the names changing under it. */
BT::PortsList commonPlanningPorts();

/** Everything two /move_action Behaviors share: assembling the request from the common ports, and
 *  reading the result back out. Only how the goal constraints are built differs between them. */
class MoveGroupBehaviorBase : public ActionClientBehaviorBase<moveit_msgs::action::MoveGroup>
{
public:
  MoveGroupBehaviorBase(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);

protected:
  /** Fill a goal from the common ports, with `constraints` as the goal. */
  BtExpected<Goal> assembleGoal(const moveit_msgs::msg::Constraints& constraints);
  BtStatus processResult(const WrappedResult& result) override;
};

/** Plan (and optionally execute) a joint-space motion.
 *
 *  `start_state` left unset means "from wherever the robot is". Setting it to a previous stage's
 *  `end_state` is what lets a whole approach/cut/retreat be planned before anything moves. */
class MoveToJointState : public MoveGroupBehaviorBase
{
public:
  MoveToJointState(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

protected:
  BtExpected<Goal> createGoal() override;
};

/** Plan (and optionally execute) to a Cartesian pose. */
class MoveToPose : public MoveGroupBehaviorBase
{
public:
  MoveToPose(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

protected:
  BtExpected<Goal> createGoal() override;
};

/** Run an already-planned trajectory on the controller. */
class ExecuteTrajectory : public ActionClientBehaviorBase<moveit_msgs::action::ExecuteTrajectory>
{
public:
  ExecuteTrajectory(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

protected:
  /** SUCCESS, without contacting move_group, when there is nothing to run. */
  std::optional<BtStatus> shortCircuit() override;
  BtExpected<Goal> createGoal() override;
  BtStatus processResult(const WrappedResult& result) override;
};

/** Straight-line Cartesian path through a list of waypoints.
 *
 *  Returns an UNTIMED trajectory: /compute_cartesian_path does not time-parameterise. Follow it
 *  with RetimeTrajectory or the controller receives zero-duration points and refuses (or, worse,
 *  runs them as fast as it can). */
class PlanCartesianPath : public ServiceClientBehaviorBase<moveit_msgs::srv::GetCartesianPath>
{
public:
  PlanCartesianPath(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

protected:
  BtExpected<std::shared_ptr<Request>> createRequest() override;
  BtStatus processResponse(const std::shared_ptr<Response>& response) override;
};

/** Time-parameterise a trajectory. */
class RetimeTrajectory : public SyncBehaviorBase
{
public:
  RetimeTrajectory(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

private:
  BtStatus tick() override;
};

/** Solve IK for a pose, with the target_link/ik_link correction applied. */
class ComputeInverseKinematics : public ServiceClientBehaviorBase<moveit_msgs::srv::GetPositionIK>
{
public:
  ComputeInverseKinematics(const std::string& name, const NodeConfig& config,
                           BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

protected:
  BtExpected<std::shared_ptr<Request>> createRequest() override;
  BtStatus processResponse(const std::shared_ptr<Response>& response) override;
};

/** Ask move_group to stop executing.
 *
 *  Deliberately NOT an emergency stop. It publishes on /trajectory_execution_event, which
 *  move_group honours between controller updates; it does not cut power and it does not bypass
 *  the controller. Anything safety-rated has to be wired to hardware, not to a Behavior. */
class StopMotion : public SendMessageToTopicBehaviorBase<std_msgs::msg::String>
{
public:
  StopMotion(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

protected:
  BtExpected<std_msgs::msg::String> createMessage() override;
};

}  // namespace moveit2_extended::behaviors
