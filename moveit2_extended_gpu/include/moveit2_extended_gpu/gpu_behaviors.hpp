// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_core/service_client_behavior_base.hpp>

#include <moveit2_extended_msgs/srv/plan_with_gpu.hpp>
#include <moveit2_extended_msgs/srv/solve_ik_batch.hpp>

#include <string>

namespace moveit2_extended::gpu
{

/** Solve many IK queries at once on the GPU.
 *
 *  The reason this Behavior exists and a "SolveIK" one does not: one IK query is not worth a GPU.
 *  Choosing where to cut a grape cluster means proposing dozens of candidate poses around it and
 *  asking which are reachable, and that batch is the shape a GPU is good at. Measured on this
 *  robot, 128 poses take 223 ms -- 1.74 ms each, including the round trip.
 *
 *  Ports mirror the CPU ComputeInverseKinematics where they can, so an Objective can be switched
 *  between them by changing the node name in the XML. */
class SolveIKQueriesGPU : public ServiceClientBehaviorBase<moveit2_extended_msgs::srv::SolveIKBatch>
{
public:
  SolveIKQueriesGPU(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

protected:
  BtExpected<std::shared_ptr<Request>> createRequest() override;
  BtStatus processResponse(const std::shared_ptr<Response>& response) override;
};

/** Plan a trajectory to a joint goal on the GPU.
 *
 *  Same ports as MoveToJointState so an Objective can swap planners by changing the node name, and
 *  the answer is a plain moveit_msgs/RobotTrajectory so the carrier Behaviors judge a GPU plan
 *  exactly as they judge an OMPL one. Unlike MoveToJointState there is no plan_only port: this
 *  never executes. Follow it with ExecuteTrajectory, which is the order the carrier gate needs
 *  anyway -- plan, judge, then move. */
class PlanToJointGoalGPU : public ServiceClientBehaviorBase<moveit2_extended_msgs::srv::PlanWithGpu>
{
public:
  PlanToJointGoalGPU(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

protected:
  BtExpected<std::shared_ptr<Request>> createRequest() override;
  BtStatus processResponse(const std::shared_ptr<Response>& response) override;
};

/** Plan a trajectory to a Cartesian goal on the GPU. See PlanToJointGoalGPU. */
class PlanToPoseGPU : public ServiceClientBehaviorBase<moveit2_extended_msgs::srv::PlanWithGpu>
{
public:
  PlanToPoseGPU(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

protected:
  BtExpected<std::shared_ptr<Request>> createRequest() override;
  BtStatus processResponse(const std::shared_ptr<Response>& response) override;
};

}  // namespace moveit2_extended::gpu
