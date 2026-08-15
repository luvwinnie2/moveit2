// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_core/shared_resources_node.hpp>

#include <chrono>

namespace moveit2_extended::behaviors
{

/** Return RUNNING until `duration` seconds have passed, then SUCCESS.
 *
 *  BehaviorTree.CPP's <Delay> decorator exists but needs a child; this is the leaf form, and it is
 *  what the smoke test uses to prove the tick loop and halt path work without any robot. */
class WaitForDuration : public SharedResourcesNode<BT::StatefulActionNode>
{
public:
  WaitForDuration(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);

  static BT::PortsList providedPorts();

private:
  BtStatus onStart() override;
  BtStatus onRunning() override;
  void onHalted() override;

  std::chrono::steady_clock::time_point deadline_;
};

}  // namespace moveit2_extended::behaviors
