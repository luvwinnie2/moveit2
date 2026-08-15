// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_core/bt_compat.hpp>

#include <moveit2_extended_msgs/msg/behavior_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <vector>

namespace moveit2_extended
{

/** Buffers BehaviorTree status changes for the ticking thread to drain.
 *
 *  Deliberately does NOT publish from callback(). BehaviorTree.CPP invokes the logger from inside
 *  executeTick(), so publishing there would fold DDS latency into the tick period. The server
 *  drains this once per tick and publishes on its own terms. */
class ObjectiveStatusLogger : public BT::StatusChangeLogger
{
public:
  explicit ObjectiveStatusLogger(const BtTree& tree, rclcpp::Clock::SharedPtr clock);

  void callback(BT::Duration timestamp, const BT::TreeNode& node, BtStatus prev, BtStatus status) override;
  void flush() override
  {
  }

  /** Take everything buffered since the last call. */
  std::vector<moveit2_extended_msgs::msg::BehaviorStatus> drain();

  /** True when anything has been buffered, without taking it. */
  bool empty() const;

private:
  rclcpp::Clock::SharedPtr clock_;
  mutable std::mutex mutex_;
  std::vector<moveit2_extended_msgs::msg::BehaviorStatus> events_;
};

}  // namespace moveit2_extended
