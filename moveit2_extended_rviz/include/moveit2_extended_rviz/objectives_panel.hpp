// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_msgs/action/execute_objective.hpp>
#include <moveit2_extended_msgs/msg/arm_waypoint.hpp>
#include <moveit2_extended_msgs/msg/behavior_tree_log.hpp>
#include <moveit2_extended_msgs/msg/objective_state.hpp>
#include <moveit2_extended_msgs/msg/tool_info.hpp>
#include <moveit2_extended_msgs/msg/tree_structure.hpp>
#include <moveit2_extended_msgs/msg/user_prompt.hpp>
#include <moveit2_extended_msgs/msg/user_response.hpp>
#include <moveit2_extended_msgs/srv/delete_waypoint.hpp>
#include <moveit2_extended_msgs/srv/list_objectives.hpp>
#include <moveit2_extended_msgs/srv/list_tools.hpp>
#include <moveit2_extended_msgs/srv/list_waypoints.hpp>
#include <moveit2_extended_msgs/srv/save_waypoint.hpp>
#include <moveit2_extended_msgs/srv/switch_tool.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>

#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QTreeWidget>

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

// Forward declared at global scope: an elaborated `class QHBoxLayout*` written inside the namespace
// below would declare moveit2_extended::rviz_plugin::QHBoxLayout, not Qt's.
class QHBoxLayout;
class QTabWidget;
class QWidget;

namespace moveit2_extended::rviz_plugin
{

/** RViz panel for running and watching Objectives.
 *
 *  The whole point of an Objective server is that the operator drives tasks, not joints, so this
 *  panel is deliberately the same set of controls as the web Studio: pick an Objective, fill in its
 *  parameters, run it, watch the tree light up, and answer it when a Behavior stops to ask
 *  something. RViz is where the robot already is, so for anyone with RViz open this saves opening
 *  a browser at all.
 *
 *  THREADING. Every ROS callback here does one thing: take the mutex and store a value. Qt widgets
 *  are touched only from refresh(), which a QTimer runs on the GUI thread. Touching a widget from
 *  the executor thread is undefined behaviour that usually survives testing and crashes in front of
 *  the customer. For the same reason nothing here waits on a future: RViz's executor is the thread
 *  that would have to deliver the response, so blocking on it deadlocks the whole application. */
class ObjectivesPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ObjectivesPanel(QWidget* parent = nullptr);
  ~ObjectivesPanel() override;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config& config) override;

private Q_SLOTS:
  void refresh();
  void runObjective();
  void cancelObjective();
  void objectiveSelected(int index);
  void reloadObjectives();
  void reloadWaypoints();
  void teachWaypoint();
  void deleteWaypoint();
  void switchTool();
  void answerPrompt();

private:
  // --- construction helpers ---
  QWidget* buildRunTab();
  QWidget* buildTreeTab();
  QWidget* buildTeachTab();

  // --- painting, all on the GUI thread ---
  void repaintState();
  void repaintTree();
  void repaintPrompt();
  void repaintLog();
  void repaintWaypoints();
  void repaintTools();
  void appendLog(const QString& line);

  using ExecuteObjective = moveit2_extended_msgs::action::ExecuteObjective;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteObjective>;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp_action::Client<ExecuteObjective>::SharedPtr execute_client_;
  rclcpp::Client<moveit2_extended_msgs::srv::ListObjectives>::SharedPtr list_objectives_;
  rclcpp::Client<moveit2_extended_msgs::srv::ListWaypoints>::SharedPtr list_waypoints_;
  rclcpp::Client<moveit2_extended_msgs::srv::SaveWaypoint>::SharedPtr save_waypoint_;
  rclcpp::Client<moveit2_extended_msgs::srv::DeleteWaypoint>::SharedPtr delete_waypoint_;
  rclcpp::Client<moveit2_extended_msgs::srv::ListTools>::SharedPtr list_tools_;
  rclcpp::Client<moveit2_extended_msgs::srv::SwitchTool>::SharedPtr switch_tool_;

  rclcpp::Subscription<moveit2_extended_msgs::msg::ObjectiveState>::SharedPtr state_sub_;
  rclcpp::Subscription<moveit2_extended_msgs::msg::TreeStructure>::SharedPtr structure_sub_;
  rclcpp::Subscription<moveit2_extended_msgs::msg::BehaviorTreeLog>::SharedPtr log_sub_;
  rclcpp::Subscription<moveit2_extended_msgs::msg::UserPrompt>::SharedPtr prompt_sub_;
  rclcpp::Publisher<moveit2_extended_msgs::msg::UserResponse>::SharedPtr response_pub_;

  // --- shared between the executor thread and the GUI thread; mutex_ guards all of it ---
  mutable std::mutex mutex_;
  std::optional<moveit2_extended_msgs::msg::ObjectiveState> state_;
  std::optional<moveit2_extended_msgs::msg::TreeStructure> structure_;
  std::map<uint16_t, uint8_t> node_status_;
  std::optional<moveit2_extended_msgs::msg::UserPrompt> prompt_;
  std::vector<moveit2_extended_msgs::msg::ObjectiveInfo> objectives_;
  std::vector<moveit2_extended_msgs::msg::ArmWaypoint> waypoints_;
  std::vector<moveit2_extended_msgs::msg::ToolInfo> tools_;
  std::vector<QString> pending_log_;
  bool objectives_dirty_ = false;
  bool waypoints_dirty_ = false;
  bool tools_dirty_ = false;
  std::string running_goal_note_;

  // --- GUI-thread-only state ---
  /** The structure currently laid out in tree_. Rebuilding the QTreeWidget on every tick would
   *  fight the user's scroll position and collapse the branches they opened, so the widget is
   *  rebuilt only when the server says a different tree instance is running. */
  std::string laid_out_tree_id_;
  std::map<uint16_t, QTreeWidgetItem*> tree_items_;
  QString shown_prompt_id_;
  QString last_result_;

  std::shared_ptr<GoalHandle> goal_handle_;

  QLabel* status_ = nullptr;
  QLabel* current_ = nullptr;
  QWidget* prompt_box_ = nullptr;
  QLabel* prompt_message_ = nullptr;
  QHBoxLayout* prompt_buttons_ = nullptr;

  QTabWidget* tabs_ = nullptr;
  QComboBox* objective_list_ = nullptr;
  QLabel* objective_description_ = nullptr;
  QTableWidget* parameters_ = nullptr;
  QPushButton* run_ = nullptr;
  QPushButton* cancel_ = nullptr;
  QLabel* result_ = nullptr;

  QTreeWidget* tree_ = nullptr;

  QListWidget* waypoint_list_ = nullptr;
  QLineEdit* waypoint_name_ = nullptr;
  QComboBox* tool_list_ = nullptr;
  QLabel* teach_status_ = nullptr;

  QPlainTextEdit* log_ = nullptr;
};

}  // namespace moveit2_extended::rviz_plugin
