// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_core/behaviors/log_message.hpp>

#include <moveit2_extended_core/blackboard_text.hpp>

namespace moveit2_extended::behaviors
{

LogMessage::LogMessage(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources)
  : SyncBehaviorBase(name, config, std::move(shared_resources))
{
}

BT::PortsList LogMessage::providedPorts()
{
  return {
    BT::InputPort<std::string>("message", "", "text to log; {blackboard} references are substituted"),
    BT::InputPort<std::string>("level", "info", "debug | info | warn | error"),
  };
}

BtStatus LogMessage::tick()
{
  // BT itself only substitutes a port whose value is *nothing but* one {reference}, so
  // "rejected at waypoint {bad_index}" arrives with the braces intact. Expand them here, which is
  // what the port description has always promised.
  const auto message = expandBlackboardReferences(config().blackboard, getInputOr<std::string>("message", ""));
  const auto level = getInputOr<std::string>("level", std::string("info"));

  if (level == "debug")
  {
    RCLCPP_DEBUG(getLogger(), "%s", message.c_str());
  }
  else if (level == "warn")
  {
    RCLCPP_WARN(getLogger(), "%s", message.c_str());
  }
  else if (level == "error")
  {
    RCLCPP_ERROR(getLogger(), "%s", message.c_str());
  }
  else
  {
    RCLCPP_INFO(getLogger(), "%s", message.c_str());
  }
  return BtStatus::SUCCESS;
}

}  // namespace moveit2_extended::behaviors
