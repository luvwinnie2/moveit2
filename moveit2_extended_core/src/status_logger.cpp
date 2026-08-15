// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_core/status_logger.hpp>
#include <moveit2_extended_core/tree_introspection.hpp>

namespace moveit2_extended
{

ObjectiveStatusLogger::ObjectiveStatusLogger(const BtTree& tree, rclcpp::Clock::SharedPtr clock)
  : BT::StatusChangeLogger(tree.rootNode()), clock_(std::move(clock))
{
}

void ObjectiveStatusLogger::callback(BT::Duration /*timestamp*/, const BT::TreeNode& node, BtStatus prev,
                                     BtStatus status)
{
  moveit2_extended_msgs::msg::BehaviorStatus event;
  // The BT timestamp is a steady-clock duration since the logger was constructed, which is not
  // comparable with anything else on the graph. Stamp with the node clock instead so a UI can line
  // these up against topics.
  event.stamp = clock_ ? clock_->now() : rclcpp::Time();
  event.uid = node.UID();
  event.instance_name = node.name();
  event.registration_name = node.registrationName();
  event.previous_status = statusToMsg(prev);
  event.current_status = statusToMsg(status);

  std::lock_guard<std::mutex> lock(mutex_);
  events_.push_back(std::move(event));
}

std::vector<moveit2_extended_msgs::msg::BehaviorStatus> ObjectiveStatusLogger::drain()
{
  std::vector<moveit2_extended_msgs::msg::BehaviorStatus> out;
  std::lock_guard<std::mutex> lock(mutex_);
  out.swap(events_);
  return out;
}

bool ObjectiveStatusLogger::empty() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return events_.empty();
}

}  // namespace moveit2_extended
