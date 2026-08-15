// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

// Small synchronous Behaviors: build a value, move a value, look a value up. Individually trivial;
// collectively they are what stops every Objective needing a bespoke C++ node to compute an offset.

#include <moveit2_extended_behaviors/port_conversions.hpp>

#include <moveit2_extended_core/shared_resources_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace moveit2_extended::behaviors
{

/** Assemble a PoseStamped from numbers, so an Objective can compute an offset target without a
 *  dedicated Behavior for each one. */
class CreatePoseStamped : public SyncBehaviorBase
{
public:
  CreatePoseStamped(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

private:
  BtStatus tick() override;
};

/** Apply a translation and rotation to a pose, in either the pose's own frame or its parent frame.
 *
 *  "Back off 100 mm along the tool axis" is `local` and "raise 100 mm" is not, and confusing the
 *  two is a common way to drive a tool into what it was retreating from -- hence the explicit
 *  port rather than a convention. */
class TransformPose : public SyncBehaviorBase
{
public:
  TransformPose(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

private:
  BtStatus tick() override;
};

/** Look a pose up in TF. */
class GetLatestTransform : public SyncBehaviorBase
{
public:
  GetLatestTransform(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

private:
  BtStatus tick() override;
};

/** The robot's current joint values. */
class GetJointState : public SyncBehaviorBase
{
public:
  GetJointState(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

private:
  BtStatus tick() override;
};

/** Read a YAML file onto the blackboard, so an Objective's numbers live in a file a person can
 *  edit rather than inside the tree. */
class LoadObjectiveParameters : public SyncBehaviorBase
{
public:
  LoadObjectiveParameters(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

private:
  BtStatus tick() override;
};

/** One element of a semicolon-separated list, for iterating over taught targets. */
class GetElementOfVector : public SyncBehaviorBase
{
public:
  GetElementOfVector(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

private:
  BtStatus tick() override;
};

/** Succeed when a pose is within tolerance of the identity -- i.e. two frames coincide. */
class IsPoseNearIdentity : public ConditionBehaviorBase
{
public:
  IsPoseNearIdentity(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

private:
  BtStatus tick() override;
};

}  // namespace moveit2_extended::behaviors
