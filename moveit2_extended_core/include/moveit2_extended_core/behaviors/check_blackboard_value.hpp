// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_core/shared_resources_node.hpp>

namespace moveit2_extended::behaviors
{

/** Compare a blackboard entry against a literal, as strings.
 *
 *  String comparison on purpose: BehaviorTree.CPP stores whatever type the writer used, and this
 *  Behavior has to work for entries whose type it has never heard of. Numeric comparison is a
 *  separate concern and belongs in a numeric Behavior with its own tolerance handling. */
class CheckBlackboardValue : public ConditionBehaviorBase
{
public:
  CheckBlackboardValue(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);

  static BT::PortsList providedPorts();

private:
  BtStatus tick() override;
};

}  // namespace moveit2_extended::behaviors
