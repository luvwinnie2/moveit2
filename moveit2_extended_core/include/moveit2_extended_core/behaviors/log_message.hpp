// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_core/shared_resources_node.hpp>

namespace moveit2_extended::behaviors
{

/** Write a line to the ROS log. Always succeeds.
 *
 *  The `message` port goes through the blackboard, so "cut failed at waypoint {bad_index}"
 *  substitutes the value at run time -- which is what makes a Fallback's recovery branch able to
 *  say why it was taken. */
class LogMessage : public SyncBehaviorBase
{
public:
  LogMessage(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);

  static BT::PortsList providedPorts();

private:
  BtStatus tick() override;
};

}  // namespace moveit2_extended::behaviors
