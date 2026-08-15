// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_core/objective_server.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // MultiThreaded so services stay answerable while an Objective is running. The tree itself is
  // ticked on its own worker thread and never from an executor callback -- see the threading
  // contract in behavior_context.hpp.
  rclcpp::executors::MultiThreadedExecutor executor;
  auto server = std::make_shared<moveit2_extended::ObjectiveServer>(rclcpp::NodeOptions{});
  executor.add_node(server->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
