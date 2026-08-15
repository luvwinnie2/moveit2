// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

// Planning-scene Behaviors.
//
// Every one of them supports the two-mode pattern MoveIt Pro uses for SetCollisionRule, and it is
// worth stating once here because it applies throughout:
//
//   planning_scene port NOT wired   the Behavior reads and writes the LIVE scene over
//                                   /get_planning_scene and /apply_planning_scene
//   planning_scene port wired       the Behavior reads the scene from the port, modifies it, and
//                                   writes it back to the same port. No service calls at all.
//
// The second mode is what lets a tree assemble a hypothetical scene -- "what if this object were
// here" -- and hand it to a validation Behavior without ever touching the robot's real world.

#include <moveit2_extended_behaviors/port_conversions.hpp>

#include <moveit2_extended_core/service_client_behavior_base.hpp>
#include <moveit2_extended_core/shared_resources_node.hpp>

#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>

namespace moveit2_extended::behaviors
{

/** Fetch the current planning scene, for a Behavior that wants to reason about it without
 *  changing it. */
class GetCurrentPlanningScene : public ServiceClientBehaviorBase<moveit_msgs::srv::GetPlanningScene>
{
public:
  GetCurrentPlanningScene(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

protected:
  BtExpected<std::shared_ptr<Request>> createRequest() override;
  BtStatus processResponse(const std::shared_ptr<Response>& response) override;
};

/** Add a primitive or mesh collision object to the scene.
 *
 *  Named after MoveIt Pro's Behavior of the same name so an Objective written against Pro ports
 *  over unchanged. */
class AddVirtualObjectToPlanningScene : public ServiceClientBehaviorBase<moveit_msgs::srv::ApplyPlanningScene>
{
public:
  AddVirtualObjectToPlanningScene(const std::string& name, const NodeConfig& config,
                                  BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

protected:
  BtExpected<std::shared_ptr<Request>> createRequest() override;
  BtStatus processResponse(const std::shared_ptr<Response>& response) override;
};

/** Move, remove, attach or detach an object already in the scene. */
class ModifyObjectInPlanningScene : public ServiceClientBehaviorBase<moveit_msgs::srv::ApplyPlanningScene>
{
public:
  ModifyObjectInPlanningScene(const std::string& name, const NodeConfig& config,
                              BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

protected:
  BtExpected<std::shared_ptr<Request>> createRequest() override;
  BtStatus processResponse(const std::shared_ptr<Response>& response) override;
};

/** Remove named objects, or everything the tree put there. */
class ClearSceneObjects : public ServiceClientBehaviorBase<moveit_msgs::srv::ApplyPlanningScene>
{
public:
  ClearSceneObjects(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

protected:
  BtExpected<std::shared_ptr<Request>> createRequest() override;
  BtStatus processResponse(const std::shared_ptr<Response>& response) override;
};

}  // namespace moveit2_extended::behaviors
