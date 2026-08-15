// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_rviz/objectives_panel.hpp>

#include <rviz_common/display_context.hpp>

#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>

#include <QFont>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>

namespace moveit2_extended::rviz_plugin
{
namespace
{
using ObjectiveState = moveit2_extended_msgs::msg::ObjectiveState;
using BehaviorStatus = moveit2_extended_msgs::msg::BehaviorStatus;
using TreeNodeInfo = moveit2_extended_msgs::msg::TreeNodeInfo;

constexpr const char* kServer = "/objective_server";
constexpr const char* kWaypoints = "/arm_waypoint_manager";
constexpr const char* kTools = "/tool_manager";
constexpr const char* kPromptTopic = "/objective/user_prompt";
constexpr const char* kResponseTopic = "/objective/user_response";

/** Latched, so a panel opened mid-run sees the current value rather than waiting for the next
 *  change -- which for an idle server never comes. */
rclcpp::QoS latchedQos()
{
  return rclcpp::QoS(1).transient_local().reliable();
}

QString stateName(uint8_t state)
{
  switch (state)
  {
    case ObjectiveState::IDLE:
      return "IDLE";
    case ObjectiveState::RUNNING:
      return "RUNNING";
    case ObjectiveState::SUCCEEDED:
      return "SUCCEEDED";
    case ObjectiveState::FAILED:
      return "FAILED";
    case ObjectiveState::CANCELED:
      return "CANCELED";
    default:
      return "?";
  }
}

QString stateColour(uint8_t state)
{
  switch (state)
  {
    case ObjectiveState::RUNNING:
      return "#2471a3";
    case ObjectiveState::SUCCEEDED:
      return "#1e8449";
    case ObjectiveState::FAILED:
      return "#c0392b";
    case ObjectiveState::CANCELED:
      return "#b9770e";
    default:
      return "#566573";
  }
}

/** Same palette as the web Studio on purpose: the same colour has to mean the same thing whichever
 *  of the two an operator happens to be looking at. */
QColor statusColour(uint8_t status)
{
  switch (status)
  {
    case BehaviorStatus::RUNNING:
      return QColor("#26456f");
    case BehaviorStatus::SUCCESS:
      return QColor("#1f4a33");
    case BehaviorStatus::FAILURE:
      return QColor("#56221c");
    default:
      return QColor(Qt::transparent);
  }
}

QString nodeTypeName(uint8_t type)
{
  switch (type)
  {
    case TreeNodeInfo::ACTION:
      return "action";
    case TreeNodeInfo::CONDITION:
      return "condition";
    case TreeNodeInfo::CONTROL:
      return "control";
    case TreeNodeInfo::DECORATOR:
      return "decorator";
    case TreeNodeInfo::SUBTREE:
      return "subtree";
    default:
      return "";
  }
}

QString errorCodeName(uint8_t code)
{
  using Result = moveit2_extended_msgs::action::ExecuteObjective::Result;
  switch (code)
  {
    case Result::SUCCESS:
      return "SUCCESS";
    case Result::UNKNOWN_OBJECTIVE:
      return "UNKNOWN_OBJECTIVE";
    case Result::XML_PARSE_ERROR:
      return "XML_PARSE_ERROR";
    case Result::MISSING_BEHAVIOR:
      return "MISSING_BEHAVIOR";
    case Result::BEHAVIOR_FAILURE:
      return "BEHAVIOR_FAILURE";
    case Result::BEHAVIOR_EXCEPTION:
      return "BEHAVIOR_EXCEPTION";
    case Result::CANCELED:
      return "CANCELED";
    case Result::TIMEOUT:
      return "TIMEOUT";
    case Result::SERVER_BUSY:
      return "SERVER_BUSY";
    case Result::INVALID_PARAMETER:
      return "INVALID_PARAMETER";
    default:
      return "?";
  }
}
}  // namespace

ObjectivesPanel::ObjectivesPanel(QWidget* parent) : rviz_common::Panel(parent)
{
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(4, 4, 4, 4);

  status_ = new QLabel("waiting for the objective server…", this);
  QFont bold = status_->font();
  bold.setBold(true);
  status_->setFont(bold);
  status_->setWordWrap(true);
  root->addWidget(status_);

  current_ = new QLabel(this);
  current_->setWordWrap(true);
  current_->setTextFormat(Qt::PlainText);
  root->addWidget(current_);

  // The prompt sits above the tabs and outside them because a blocked Objective must be visible
  // whichever tab is open. A question nobody notices looks exactly like a hang.
  prompt_box_ = new QGroupBox("The robot is asking", this);
  auto* prompt_layout = new QVBoxLayout(prompt_box_);
  prompt_message_ = new QLabel(prompt_box_);
  prompt_message_->setWordWrap(true);
  prompt_layout->addWidget(prompt_message_);
  prompt_buttons_ = new QHBoxLayout();
  prompt_layout->addLayout(prompt_buttons_);
  prompt_box_->setVisible(false);
  root->addWidget(prompt_box_);

  tabs_ = new QTabWidget(this);
  tabs_->addTab(buildRunTab(), "Run");
  tabs_->addTab(buildTreeTab(), "Tree");
  tabs_->addTab(buildTeachTab(), "Teach");
  root->addWidget(tabs_, 1);

  log_ = new QPlainTextEdit(this);
  log_->setReadOnly(true);
  log_->setMaximumBlockCount(500);
  log_->setMaximumHeight(110);
  root->addWidget(log_);
}

ObjectivesPanel::~ObjectivesPanel() = default;

QWidget* ObjectivesPanel::buildRunTab()
{
  auto* page = new QWidget(this);
  auto* layout = new QVBoxLayout(page);

  auto* pick = new QHBoxLayout();
  objective_list_ = new QComboBox(page);
  objective_list_->setToolTip("Objectives the server has loaded. One that references a Behavior no "
                              "plugin registered is listed but cannot be run.");
  pick->addWidget(objective_list_, 1);
  auto* reload = new QPushButton("↻", page);
  reload->setToolTip("Re-read the Objective list from the server");
  reload->setMaximumWidth(32);
  pick->addWidget(reload);
  layout->addLayout(pick);

  objective_description_ = new QLabel(page);
  objective_description_->setWordWrap(true);
  objective_description_->setTextFormat(Qt::PlainText);
  layout->addWidget(objective_description_);

  parameters_ = new QTableWidget(0, 2, page);
  parameters_->setHorizontalHeaderLabels({ "parameter", "value" });
  parameters_->horizontalHeader()->setStretchLastSection(true);
  parameters_->verticalHeader()->setVisible(false);
  parameters_->setToolTip("Written to the tree's blackboard before the first tick. Everything goes "
                          "in as a string; BehaviorTree.CPP converts on read.");
  layout->addWidget(parameters_, 1);

  auto* buttons = new QHBoxLayout();
  run_ = new QPushButton("Run", page);
  cancel_ = new QPushButton("Cancel", page);
  cancel_->setEnabled(false);
  buttons->addWidget(run_);
  buttons->addWidget(cancel_);
  layout->addLayout(buttons);

  result_ = new QLabel(page);
  result_->setWordWrap(true);
  result_->setTextFormat(Qt::PlainText);
  layout->addWidget(result_);

  connect(reload, &QPushButton::clicked, this, &ObjectivesPanel::reloadObjectives);
  connect(run_, &QPushButton::clicked, this, &ObjectivesPanel::runObjective);
  connect(cancel_, &QPushButton::clicked, this, &ObjectivesPanel::cancelObjective);
  connect(objective_list_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &ObjectivesPanel::objectiveSelected);
  return page;
}

QWidget* ObjectivesPanel::buildTreeTab()
{
  auto* page = new QWidget(this);
  auto* layout = new QVBoxLayout(page);
  tree_ = new QTreeWidget(page);
  tree_->setColumnCount(3);
  tree_->setHeaderLabels({ "behavior", "type", "status" });
  tree_->setToolTip("The tree the server is running, coloured by node status. Rebuilt only when a "
                    "different tree starts, so branches you opened stay open.");
  layout->addWidget(tree_);
  return page;
}

QWidget* ObjectivesPanel::buildTeachTab()
{
  auto* page = new QWidget(this);
  auto* layout = new QVBoxLayout(page);

  layout->addWidget(new QLabel("Waypoints", page));
  waypoint_list_ = new QListWidget(page);
  waypoint_list_->setToolTip("Taught arm targets. A pose is stored with the frame and the link it "
                             "describes, because on a mobile base a robot_base pose is relative.");
  layout->addWidget(waypoint_list_, 1);

  auto* teach_row = new QHBoxLayout();
  waypoint_name_ = new QLineEdit(page);
  waypoint_name_->setPlaceholderText("name");
  auto* teach = new QPushButton("Teach here", page);
  auto* remove = new QPushButton("Delete", page);
  teach_row->addWidget(waypoint_name_, 1);
  teach_row->addWidget(teach);
  teach_row->addWidget(remove);
  layout->addLayout(teach_row);

  layout->addWidget(new QLabel("Tool", page));
  auto* tool_row = new QHBoxLayout();
  tool_list_ = new QComboBox(page);
  auto* fit = new QPushButton("Fit", page);
  fit->setToolTip("Attach this end-effector. The dresspack configuration that goes with it is "
                  "switched at the same time -- a heavier tool works the carrier harder.");
  tool_row->addWidget(tool_list_, 1);
  tool_row->addWidget(fit);
  layout->addLayout(tool_row);

  teach_status_ = new QLabel(page);
  teach_status_->setWordWrap(true);
  teach_status_->setTextFormat(Qt::PlainText);
  layout->addWidget(teach_status_);

  connect(teach, &QPushButton::clicked, this, &ObjectivesPanel::teachWaypoint);
  connect(remove, &QPushButton::clicked, this, &ObjectivesPanel::deleteWaypoint);
  connect(fit, &QPushButton::clicked, this, &ObjectivesPanel::switchTool);
  return page;
}

void ObjectivesPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  execute_client_ = rclcpp_action::create_client<ExecuteObjective>(node_, std::string(kServer) + "/execute_objective");
  list_objectives_ =
      node_->create_client<moveit2_extended_msgs::srv::ListObjectives>(std::string(kServer) + "/list_objectives");
  list_waypoints_ = node_->create_client<moveit2_extended_msgs::srv::ListWaypoints>(std::string(kWaypoints) + "/list");
  save_waypoint_ = node_->create_client<moveit2_extended_msgs::srv::SaveWaypoint>(std::string(kWaypoints) + "/save");
  delete_waypoint_ =
      node_->create_client<moveit2_extended_msgs::srv::DeleteWaypoint>(std::string(kWaypoints) + "/delete");
  list_tools_ = node_->create_client<moveit2_extended_msgs::srv::ListTools>(std::string(kTools) + "/list");
  switch_tool_ = node_->create_client<moveit2_extended_msgs::srv::SwitchTool>(std::string(kTools) + "/switch");

  response_pub_ = node_->create_publisher<moveit2_extended_msgs::msg::UserResponse>(kResponseTopic, rclcpp::QoS(10));

  state_sub_ = node_->create_subscription<ObjectiveState>(
      std::string(kServer) + "/objective_state", latchedQos(), [this](const ObjectiveState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = *msg;
      });

  structure_sub_ = node_->create_subscription<moveit2_extended_msgs::msg::TreeStructure>(
      std::string(kServer) + "/tree_structure", latchedQos(),
      [this](const moveit2_extended_msgs::msg::TreeStructure::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        structure_ = *msg;
        node_status_.clear();
      });

  log_sub_ = node_->create_subscription<moveit2_extended_msgs::msg::BehaviorTreeLog>(
      std::string(kServer) + "/behavior_tree_log", rclcpp::QoS(20),
      [this](const moveit2_extended_msgs::msg::BehaviorTreeLog::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& event : msg->event_log)
        {
          node_status_[event.uid] = event.current_status;
          if (event.current_status == BehaviorStatus::FAILURE)
          {
            // Only failures go to the log. Every transition would be thousands of lines a minute
            // and would bury the one line that matters.
            pending_log_.push_back(QString("FAILURE  %1 (%2)")
                                       .arg(QString::fromStdString(event.instance_name))
                                       .arg(QString::fromStdString(event.registration_name)));
          }
        }
      });

  prompt_sub_ = node_->create_subscription<moveit2_extended_msgs::msg::UserPrompt>(
      kPromptTopic, latchedQos(), [this](const moveit2_extended_msgs::msg::UserPrompt::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        prompt_ = *msg;
      });

  auto* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &ObjectivesPanel::refresh);
  timer->start(200);

  // The servers are usually still coming up when RViz finishes loading its config, so the first
  // query is deferred rather than fired into a void and reported as "not running".
  QTimer::singleShot(2000, this, &ObjectivesPanel::reloadObjectives);
  QTimer::singleShot(2200, this, &ObjectivesPanel::reloadWaypoints);
  QTimer::singleShot(2400, this, [this]() {
    if (!list_tools_ || !list_tools_->service_is_ready())
    {
      return;
    }
    list_tools_->async_send_request(
        std::make_shared<moveit2_extended_msgs::srv::ListTools::Request>(),
        [this](rclcpp::Client<moveit2_extended_msgs::srv::ListTools>::SharedFuture future) {
          std::lock_guard<std::mutex> lock(mutex_);
          tools_ = future.get()->tools;
          tools_dirty_ = true;
        });
  });
}

// ---------------------------------------------------------------------------------------------
// queries
// ---------------------------------------------------------------------------------------------

void ObjectivesPanel::reloadObjectives()
{
  if (!list_objectives_ || !list_objectives_->service_is_ready())
  {
    appendLog("objective server is not up");
    return;
  }
  auto request = std::make_shared<moveit2_extended_msgs::srv::ListObjectives::Request>();
  list_objectives_->async_send_request(
      request, [this](rclcpp::Client<moveit2_extended_msgs::srv::ListObjectives>::SharedFuture future) {
        // Executor thread: cache only, never touch a widget from here.
        std::lock_guard<std::mutex> lock(mutex_);
        objectives_ = future.get()->objectives;
        objectives_dirty_ = true;
      });
}

void ObjectivesPanel::reloadWaypoints()
{
  if (!list_waypoints_ || !list_waypoints_->service_is_ready())
  {
    return;
  }
  list_waypoints_->async_send_request(
      std::make_shared<moveit2_extended_msgs::srv::ListWaypoints::Request>(),
      [this](rclcpp::Client<moveit2_extended_msgs::srv::ListWaypoints>::SharedFuture future) {
        std::lock_guard<std::mutex> lock(mutex_);
        waypoints_ = future.get()->waypoints;
        waypoints_dirty_ = true;
      });
}

// ---------------------------------------------------------------------------------------------
// running
// ---------------------------------------------------------------------------------------------

void ObjectivesPanel::objectiveSelected(int index)
{
  std::vector<moveit2_extended_msgs::msg::ObjectiveInfo> objectives;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    objectives = objectives_;
  }
  if (index < 0 || index >= static_cast<int>(objectives.size()))
  {
    return;
  }
  const auto& info = objectives[static_cast<size_t>(index)];

  QString description = QString::fromStdString(info.description);
  if (!info.missing_behaviors.empty())
  {
    QStringList missing;
    for (const auto& name : info.missing_behaviors)
    {
      missing << QString::fromStdString(name);
    }
    description += QString("\n\nCANNOT RUN — no plugin registered: %1").arg(missing.join(", "));
  }
  objective_description_->setText(description);
  run_->setEnabled(info.missing_behaviors.empty());

  // Pre-fill one row per declared parameter so the operator does not have to know the port names.
  // Required ones are marked; leaving one blank is what makes the run fail with INVALID_PARAMETER
  // rather than something obscure deep in the tree.
  parameters_->setRowCount(0);
  const auto addRow = [this](const std::string& name, bool required) {
    const int row = parameters_->rowCount();
    parameters_->insertRow(row);
    auto* key = new QTableWidgetItem(QString::fromStdString(name) + (required ? " *" : ""));
    key->setFlags(key->flags() & ~Qt::ItemIsEditable);
    parameters_->setItem(row, 0, key);
    parameters_->setItem(row, 1, new QTableWidgetItem(""));
  };
  for (const auto& name : info.required_parameters)
  {
    addRow(name, true);
  }
  for (const auto& name : info.optional_parameters)
  {
    addRow(name, false);
  }
}

void ObjectivesPanel::runObjective()
{
  if (!execute_client_ || !execute_client_->action_server_is_ready())
  {
    appendLog("objective server's action is not available");
    return;
  }
  const int index = objective_list_->currentIndex();
  if (index < 0)
  {
    return;
  }

  ExecuteObjective::Goal goal;
  goal.objective_name = objective_list_->itemData(index).toString().toStdString();

  for (int row = 0; row < parameters_->rowCount(); ++row)
  {
    auto* key_item = parameters_->item(row, 0);
    auto* value_item = parameters_->item(row, 1);
    if (key_item == nullptr || value_item == nullptr || value_item->text().isEmpty())
    {
      continue;
    }
    QString key = key_item->text();
    if (key.endsWith(" *"))
    {
      key.chop(2);
    }
    rcl_interfaces::msg::Parameter parameter;
    parameter.name = key.toStdString();
    // Always a string. BehaviorTree.CPP converts a string blackboard entry to the port's declared
    // type on read, so this works for ports this panel has never heard of.
    parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    parameter.value.string_value = value_item->text().toStdString();
    goal.parameters.push_back(parameter);
  }

  rclcpp_action::Client<ExecuteObjective>::SendGoalOptions options;
  options.goal_response_callback = [this](std::shared_ptr<GoalHandle> handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_ = handle;
    pending_log_.push_back(handle ? "goal accepted" : "goal REJECTED by the server");
  };
  options.feedback_callback = [this](std::shared_ptr<GoalHandle>,
                                     const std::shared_ptr<const ExecuteObjective::Feedback> feedback) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& event : feedback->changed)
    {
      node_status_[event.uid] = event.current_status;
    }
  };
  options.result_callback = [this](const GoalHandle::WrappedResult& wrapped) {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_.reset();
    const auto& result = wrapped.result;
    QString line = QString("%1 — %2").arg(errorCodeName(result->error_code),
                                          QString::fromStdString(result->error_message));
    if (!result->failed_behavior_name.empty())
    {
      line += QString("\nfailed at: %1 (%2)")
                  .arg(QString::fromStdString(result->failed_behavior_name),
                       QString::fromStdString(result->failed_behavior_registration));
    }
    running_goal_note_ = line.toStdString();
    pending_log_.push_back(line);
  };

  execute_client_->async_send_goal(goal, options);
  result_->setText("");
  appendLog(QString("running %1").arg(QString::fromStdString(goal.objective_name)));
}

void ObjectivesPanel::cancelObjective()
{
  std::shared_ptr<GoalHandle> handle;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handle = goal_handle_;
  }
  if (!handle)
  {
    appendLog("nothing to cancel");
    return;
  }
  // Fire and forget. Waiting for the cancel response would block RViz's GUI thread on RViz's own
  // executor, which is the thread that has to deliver it.
  execute_client_->async_cancel_goal(handle);
  appendLog("cancel requested");
}

// ---------------------------------------------------------------------------------------------
// teaching and tools
// ---------------------------------------------------------------------------------------------

void ObjectivesPanel::teachWaypoint()
{
  const QString name = waypoint_name_->text().trimmed();
  if (name.isEmpty())
  {
    teach_status_->setText("give the waypoint a name first");
    return;
  }
  if (!save_waypoint_ || !save_waypoint_->service_is_ready())
  {
    teach_status_->setText("arm_waypoint_manager is not up");
    return;
  }
  auto request = std::make_shared<moveit2_extended_msgs::srv::SaveWaypoint::Request>();
  request->name = name.toStdString();
  // BOTH: joints reproduce the elbow the operator taught, the pose is what a Cartesian Behavior
  // needs. Storing one of the two throws away information that cannot be recovered later.
  request->capture = moveit2_extended_msgs::srv::SaveWaypoint::Request::CAPTURE_CURRENT_BOTH;
  save_waypoint_->async_send_request(
      request, [this](rclcpp::Client<moveit2_extended_msgs::srv::SaveWaypoint>::SharedFuture future) {
        const auto response = future.get();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          pending_log_.push_back(response->success ? "waypoint saved" :
                                                     QString("save failed: %1")
                                                         .arg(QString::fromStdString(response->message)));
        }
        reloadWaypoints();
      });
}

void ObjectivesPanel::deleteWaypoint()
{
  auto* item = waypoint_list_->currentItem();
  if (item == nullptr)
  {
    teach_status_->setText("select a waypoint to delete");
    return;
  }
  if (!delete_waypoint_ || !delete_waypoint_->service_is_ready())
  {
    teach_status_->setText("arm_waypoint_manager is not up");
    return;
  }
  auto request = std::make_shared<moveit2_extended_msgs::srv::DeleteWaypoint::Request>();
  request->name = item->data(Qt::UserRole).toString().toStdString();
  delete_waypoint_->async_send_request(
      request, [this](rclcpp::Client<moveit2_extended_msgs::srv::DeleteWaypoint>::SharedFuture future) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          pending_log_.push_back(future.get()->success ? "waypoint deleted" : "delete failed");
        }
        reloadWaypoints();
      });
}

void ObjectivesPanel::switchTool()
{
  if (!switch_tool_ || !switch_tool_->service_is_ready())
  {
    teach_status_->setText("tool_manager is not up");
    return;
  }
  auto request = std::make_shared<moveit2_extended_msgs::srv::SwitchTool::Request>();
  request->name = tool_list_->currentText().toStdString();
  switch_tool_->async_send_request(
      request, [this](rclcpp::Client<moveit2_extended_msgs::srv::SwitchTool>::SharedFuture future) {
        const auto response = future.get();
        std::lock_guard<std::mutex> lock(mutex_);
        pending_log_.push_back(response->success ?
                                   QString("fitted %1").arg(QString::fromStdString(response->active_tool.name)) :
                                   QString("tool change failed: %1").arg(QString::fromStdString(response->message)));
        tools_dirty_ = true;
      });
}

void ObjectivesPanel::answerPrompt()
{
  auto* button = qobject_cast<QPushButton*>(sender());
  if (button == nullptr || shown_prompt_id_.isEmpty())
  {
    return;
  }
  moveit2_extended_msgs::msg::UserResponse response;
  response.prompt_id = shown_prompt_id_.toStdString();
  response.choice = button->text().toStdString();
  response_pub_->publish(response);
  appendLog(QString("answered '%1'").arg(button->text()));
}

// ---------------------------------------------------------------------------------------------
// painting -- GUI thread only
// ---------------------------------------------------------------------------------------------

void ObjectivesPanel::refresh()
{
  repaintState();
  repaintTree();
  repaintPrompt();
  repaintWaypoints();
  repaintTools();
  repaintLog();
}

void ObjectivesPanel::repaintState()
{
  std::optional<ObjectiveState> state;
  bool dirty = false;
  std::vector<moveit2_extended_msgs::msg::ObjectiveInfo> objectives;
  QString note;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = state_;
    dirty = objectives_dirty_;
    objectives_dirty_ = false;
    objectives = objectives_;
    note = QString::fromStdString(running_goal_note_);
    running_goal_note_.clear();
  }

  if (dirty)
  {
    const QString previous = objective_list_->currentData().toString();
    objective_list_->blockSignals(true);
    objective_list_->clear();
    for (const auto& info : objectives)
    {
      const QString name = QString::fromStdString(info.name);
      objective_list_->addItem(info.missing_behaviors.empty() ? name : name + "  (unavailable)", name);
    }
    const int restored = objective_list_->findData(previous);
    objective_list_->setCurrentIndex(restored >= 0 ? restored : (objective_list_->count() ? 0 : -1));
    objective_list_->blockSignals(false);
    objectiveSelected(objective_list_->currentIndex());
    appendLog(QString("%1 objective(s) loaded").arg(objectives.size()));
  }

  if (!note.isEmpty())
  {
    result_->setText(note);
  }

  if (!state)
  {
    return;
  }
  status_->setText(QString("%1   %2").arg(stateName(state->state), QString::fromStdString(state->objective_name)));
  status_->setStyleSheet(QString("color: %1;").arg(stateColour(state->state)));

  const bool running = state->state == ObjectiveState::RUNNING;
  cancel_->setEnabled(running);
  current_->setText(running ? QString("running: %1   (tick %2)")
                                  .arg(QString::fromStdString(state->current_behavior))
                                  .arg(state->tick_count) :
                              QString());
}

void ObjectivesPanel::repaintTree()
{
  std::optional<moveit2_extended_msgs::msg::TreeStructure> structure;
  std::map<uint16_t, uint8_t> statuses;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    structure = structure_;
    statuses = node_status_;
  }
  if (!structure)
  {
    return;
  }

  if (structure->tree_instance_id != laid_out_tree_id_)
  {
    tree_->clear();
    tree_items_.clear();
    // The structure arrives depth-first with every parent ahead of its children, so a single pass
    // can attach each node to a parent that is guaranteed to exist already.
    for (const auto& info : structure->nodes)
    {
      auto* item = new QTreeWidgetItem();
      item->setText(0, QString::fromStdString(info.instance_name.empty() ? info.registration_name :
                                                                          info.instance_name));
      item->setText(1, nodeTypeName(info.node_type));
      item->setToolTip(0, QString::fromStdString(info.registration_name));

      const auto parent = tree_items_.find(info.parent_uid);
      if (info.uid != info.parent_uid && parent != tree_items_.end())
      {
        parent->second->addChild(item);
      }
      else
      {
        tree_->addTopLevelItem(item);
      }
      tree_items_[info.uid] = item;
    }
    tree_->expandAll();
    for (int column = 0; column < tree_->columnCount(); ++column)
    {
      tree_->resizeColumnToContents(column);
    }
    laid_out_tree_id_ = structure->tree_instance_id;
  }

  for (const auto& [uid, status] : statuses)
  {
    const auto found = tree_items_.find(uid);
    if (found == tree_items_.end())
    {
      continue;
    }
    const QColor colour = statusColour(status);
    for (int column = 0; column < tree_->columnCount(); ++column)
    {
      found->second->setBackground(column, colour);
    }
    found->second->setText(2, status == BehaviorStatus::RUNNING  ? "running" :
                              status == BehaviorStatus::SUCCESS  ? "success" :
                              status == BehaviorStatus::FAILURE  ? "failure" :
                                                                   "");
  }
}

void ObjectivesPanel::repaintPrompt()
{
  std::optional<moveit2_extended_msgs::msg::UserPrompt> prompt;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    prompt = prompt_;
  }
  const QString id = prompt ? QString::fromStdString(prompt->prompt_id) : QString();
  if (id == shown_prompt_id_)
  {
    return;
  }
  shown_prompt_id_ = id;

  while (QLayoutItem* item = prompt_buttons_->takeAt(0))
  {
    delete item->widget();
    delete item;
  }

  // An empty prompt_id is the server withdrawing the question, so the buttons go away rather than
  // sitting there offering to answer something nobody is listening for any more.
  if (id.isEmpty())
  {
    prompt_box_->setVisible(false);
    return;
  }

  prompt_message_->setText(QString::fromStdString(prompt->message));
  for (const auto& choice : prompt->choices)
  {
    auto* button = new QPushButton(QString::fromStdString(choice), prompt_box_);
    connect(button, &QPushButton::clicked, this, &ObjectivesPanel::answerPrompt);
    prompt_buttons_->addWidget(button);
  }
  prompt_box_->setVisible(true);
  tabs_->setCurrentIndex(0);
}

void ObjectivesPanel::repaintWaypoints()
{
  std::vector<moveit2_extended_msgs::msg::ArmWaypoint> waypoints;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!waypoints_dirty_)
    {
      return;
    }
    waypoints_dirty_ = false;
    waypoints = waypoints_;
  }
  const QString selected = waypoint_list_->currentItem() ?
                               waypoint_list_->currentItem()->data(Qt::UserRole).toString() :
                               QString();
  waypoint_list_->clear();
  for (const auto& waypoint : waypoints)
  {
    const QString name = QString::fromStdString(waypoint.name);
    QStringList parts;
    if (waypoint.has_joint_state)
    {
      parts << "joints";
    }
    if (waypoint.has_pose)
    {
      parts << QString::fromStdString(waypoint.pose_link);
    }
    auto* item = new QListWidgetItem(QString("%1   [%2]").arg(name, parts.join(", ")));
    item->setData(Qt::UserRole, name);
    if (!waypoint.description.empty())
    {
      item->setToolTip(QString::fromStdString(waypoint.description));
    }
    waypoint_list_->addItem(item);
    if (name == selected)
    {
      waypoint_list_->setCurrentItem(item);
    }
  }
}

void ObjectivesPanel::repaintTools()
{
  std::vector<moveit2_extended_msgs::msg::ToolInfo> tools;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!tools_dirty_)
    {
      return;
    }
    tools_dirty_ = false;
    tools = tools_;
  }
  if (tools.empty())
  {
    return;
  }
  tool_list_->clear();
  QString fitted;
  for (const auto& tool : tools)
  {
    tool_list_->addItem(QString::fromStdString(tool.name));
    if (tool.attached)
    {
      fitted = QString::fromStdString(tool.name);
    }
  }
  if (!fitted.isEmpty())
  {
    tool_list_->setCurrentText(fitted);
    teach_status_->setText(QString("fitted: %1").arg(fitted));
  }
}

void ObjectivesPanel::repaintLog()
{
  std::vector<QString> lines;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    lines.swap(pending_log_);
  }
  for (const auto& line : lines)
  {
    log_->appendPlainText(line);
  }
}

void ObjectivesPanel::appendLog(const QString& line)
{
  log_->appendPlainText(line);
}

// ---------------------------------------------------------------------------------------------

void ObjectivesPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("objective", objective_list_ ? objective_list_->currentData().toString() : QString());
  config.mapSetValue("tab", tabs_ ? tabs_->currentIndex() : 0);
}

void ObjectivesPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
  int tab = 0;
  if (config.mapGetInt("tab", &tab) && tabs_ != nullptr)
  {
    tabs_->setCurrentIndex(tab);
  }
  // The Objective itself is restored in repaintState(), which is the first place the list is
  // populated -- selecting a name before the list exists would silently do nothing.
}

}  // namespace moveit2_extended::rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(moveit2_extended::rviz_plugin::ObjectivesPanel, rviz_common::Panel)
