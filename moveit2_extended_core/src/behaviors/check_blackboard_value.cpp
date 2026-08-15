// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_core/behaviors/check_blackboard_value.hpp>

namespace moveit2_extended::behaviors
{

CheckBlackboardValue::CheckBlackboardValue(const std::string& name, const NodeConfig& config,
                                           BehaviorContextPtr shared_resources)
  : ConditionBehaviorBase(name, config, std::move(shared_resources))
{
}

BT::PortsList CheckBlackboardValue::providedPorts()
{
  return {
    BT::InputPort<std::string>("value", "", "the value to test; usually {a_blackboard_key}"),
    BT::InputPort<std::string>("equals", "", "succeed when value matches this"),
    BT::InputPort<bool>("invert", false, "succeed when it does NOT match"),
  };
}

BtStatus CheckBlackboardValue::tick()
{
  const auto value = getInputOr<std::string>("value", "");
  const auto expected = getInputOr<std::string>("equals", "");
  const bool invert = getInputOr<bool>("invert", false);

  const bool matches = (value == expected);
  return (matches != invert) ? BtStatus::SUCCESS : BtStatus::FAILURE;
}

}  // namespace moveit2_extended::behaviors
