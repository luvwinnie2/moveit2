// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_core/behavior_context.hpp>
#include <moveit2_extended_core/behavior_loader_base.hpp>
#include <moveit2_extended_core/bt_compat.hpp>
#include <moveit2_extended_core/objective_library.hpp>

#include <moveit2_extended_msgs/action/execute_objective.hpp>
#include <moveit2_extended_msgs/msg/behavior_tree_log.hpp>
#include <moveit2_extended_msgs/msg/objective_state.hpp>
#include <moveit2_extended_msgs/msg/tree_structure.hpp>
#include <moveit2_extended_msgs/srv/get_objective.hpp>
#include <moveit2_extended_msgs/srv/list_behaviors.hpp>
#include <moveit2_extended_msgs/srv/list_objectives.hpp>
#include <moveit2_extended_msgs/srv/reload_objectives.hpp>
#include <moveit2_extended_msgs/srv/save_objective_xml.hpp>
#include <moveit2_extended_msgs/srv/validate_objective_xml.hpp>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace moveit2_extended
{

/** Loads Behavior plugins, registers Objectives, and runs one on request. */
class ObjectiveServer
{
public:
  explicit ObjectiveServer(const rclcpp::NodeOptions& options);
  ~ObjectiveServer();

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;
  const rclcpp::Node::SharedPtr& node() const
  {
    return node_;
  }

  /** Behaviors the factory can build. Exposed for tests. */
  std::vector<std::string> registeredBehaviors() const;
  std::vector<std::string> registeredObjectives() const;

private:
  using ExecuteObjective = moveit2_extended_msgs::action::ExecuteObjective;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteObjective>;

  void declareParameters();
  void createContext();
  void loadBehaviorLoaders();
  void loadObjectives();
  void createInterfaces();

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid,
                                         std::shared_ptr<const ExecuteObjective::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>& goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandle>& goal_handle);
  /** Worker thread body. Owns the Tree for its whole lifetime; nothing else may touch it. */
  void runObjective(const std::shared_ptr<GoalHandle>& goal_handle);

  void onListObjectives(const std::shared_ptr<moveit2_extended_msgs::srv::ListObjectives::Request> request,
                        std::shared_ptr<moveit2_extended_msgs::srv::ListObjectives::Response> response);
  void onGetObjective(const std::shared_ptr<moveit2_extended_msgs::srv::GetObjective::Request> request,
                      std::shared_ptr<moveit2_extended_msgs::srv::GetObjective::Response> response);
  void onListBehaviors(const std::shared_ptr<moveit2_extended_msgs::srv::ListBehaviors::Request> request,
                       std::shared_ptr<moveit2_extended_msgs::srv::ListBehaviors::Response> response);
  void onReloadObjectives(const std::shared_ptr<moveit2_extended_msgs::srv::ReloadObjectives::Request> request,
                          std::shared_ptr<moveit2_extended_msgs::srv::ReloadObjectives::Response> response);
  void onValidateObjectiveXml(
      const std::shared_ptr<moveit2_extended_msgs::srv::ValidateObjectiveXml::Request> request,
      std::shared_ptr<moveit2_extended_msgs::srv::ValidateObjectiveXml::Response> response);
  void onSaveObjectiveXml(const std::shared_ptr<moveit2_extended_msgs::srv::SaveObjectiveXml::Request> request,
                          std::shared_ptr<moveit2_extended_msgs::srv::SaveObjectiveXml::Response> response);

  /** Run the four validation checks. Shared by ValidateObjectiveXml and SaveObjectiveXml so the
   *  two can never disagree about what is acceptable. */
  bool validateXml(const std::string& xml, std::string& xml_error, std::vector<std::string>& missing,
                   std::vector<std::string>& unknown_ports, std::vector<std::string>& invalid_values,
                   std::vector<std::string>& tree_ids) const;

  /** Goal parameters -> blackboard, as strings. BehaviorTree.CPP converts a string entry to the
   *  port's declared type on read, so this needs no type registry. */
  static void seedBlackboard(const BtBlackboard::Ptr& blackboard,
                             const std::vector<rcl_interfaces::msg::Parameter>& parameters);
  static std::string parameterToString(const rcl_interfaces::msg::ParameterValue& value);

  void publishState(uint8_t state, const std::string& objective_name, const std::string& instance_id,
                    const std::string& current_behavior, uint32_t tick_count);

  rclcpp::Node::SharedPtr node_;

  // DECLARATION ORDER IS LOAD-BEARING.
  //
  // Members are destroyed in reverse declaration order. The factory's builders are lambdas that
  // live inside the loader plugins' shared libraries, and pluginlib::ClassLoader owns the dlopen
  // handle. If the class loader is destroyed first, ~BehaviorTreeFactory runs against unmapped
  // memory and the process segfaults on shutdown. Keep class_loader_ first.
  std::shared_ptr<pluginlib::ClassLoader<BehaviorLoaderBase>> class_loader_;
  std::vector<std::shared_ptr<BehaviorLoaderBase>> loaders_;
  BehaviorContextPtr context_;
  std::unique_ptr<BtFactory> factory_;
  ObjectiveLibrary library_;
  /** registration name -> package whose loader registered it. Makes a collision diagnosable. */
  std::unordered_map<std::string, std::string> loader_of_behavior_;

  rclcpp_action::Server<ExecuteObjective>::SharedPtr action_server_;
  rclcpp::Publisher<moveit2_extended_msgs::msg::BehaviorTreeLog>::SharedPtr log_pub_;
  rclcpp::Publisher<moveit2_extended_msgs::msg::ObjectiveState>::SharedPtr state_pub_;
  rclcpp::Publisher<moveit2_extended_msgs::msg::TreeStructure>::SharedPtr structure_pub_;
  rclcpp::Service<moveit2_extended_msgs::srv::ListObjectives>::SharedPtr list_objectives_srv_;
  rclcpp::Service<moveit2_extended_msgs::srv::GetObjective>::SharedPtr get_objective_srv_;
  rclcpp::Service<moveit2_extended_msgs::srv::ListBehaviors>::SharedPtr list_behaviors_srv_;
  rclcpp::Service<moveit2_extended_msgs::srv::ReloadObjectives>::SharedPtr reload_srv_;
  rclcpp::Service<moveit2_extended_msgs::srv::ValidateObjectiveXml>::SharedPtr validate_srv_;
  rclcpp::Service<moveit2_extended_msgs::srv::SaveObjectiveXml>::SharedPtr save_srv_;

  mutable std::mutex library_mutex_;
  std::atomic_bool busy_{ false };
  std::thread worker_;
  std::chrono::milliseconds tick_period_{ 20 };
  double default_timeout_ = 0.0;
  uint64_t instance_counter_ = 0;
};

}  // namespace moveit2_extended
