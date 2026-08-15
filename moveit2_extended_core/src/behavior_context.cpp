// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_core/behavior_context.hpp>

namespace moveit2_extended
{

void BehaviorContext::CallbackIsland::pump(std::chrono::nanoseconds max_duration) const
{
  if (executor)
  {
    executor->spin_some(max_duration);
  }
}

BehaviorContext::BehaviorContext(rclcpp::Node::SharedPtr node, BehaviorContextConfig config,
                                 BehaviorParameterMap behavior_parameters)
  : node_(std::move(node)), config_(std::move(config)), behavior_parameters_(std::move(behavior_parameters))
{
  if (!node_)
  {
    throw std::invalid_argument("BehaviorContext requires a node");
  }

  // One TF buffer and one listener in the process. The listener goes on node_, which is spun.
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);

  // Created up front but never added to an executor: MoveGroupInterface spins it itself.
  moveit_client_node_ = rclcpp::Node::make_shared(std::string(node_->get_name()) + "_moveit_client",
                                                  node_->get_namespace());
}

BehaviorContext::~BehaviorContext() = default;

BehaviorContext::CallbackIsland BehaviorContext::makeCallbackIsland()
{
  CallbackIsland island;
  // false = do not add this group to whatever executor already owns the node. That flag is the
  // whole point: it is what keeps these callbacks out of the server's MultiThreadedExecutor.
  island.group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  island.executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  island.executor->add_callback_group(island.group, node_->get_node_base_interface());
  return island;
}

void BehaviorContext::ensureMoveItStarted()
{
  if (moveit_start_attempted_)
  {
    return;
  }
  moveit_start_attempted_ = true;

  if (!config_.start_planning_scene_monitor)
  {
    RCLCPP_INFO(logger(), "planning scene monitor disabled by configuration; MoveIt-backed "
                          "Behaviors will report that MoveIt is unavailable");
    return;
  }

  try
  {
    // Note: the overload taking a tf2_ros::Buffer is deprecated in MoveIt 2.5 and ignores the
    // buffer anyway, so it is not passed here.
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, config_.robot_description_param,
                                                                          "moveit2_extended_psm");
  }
  catch (const std::exception& exc)
  {
    RCLCPP_ERROR(logger(), "cannot create the planning scene monitor: %s", exc.what());
    psm_.reset();
    return;
  }

  if (!psm_->getPlanningScene())
  {
    RCLCPP_ERROR(logger(), "planning scene monitor has no scene; is '%s' published?",
                 config_.robot_description_param.c_str());
    psm_.reset();
    return;
  }

  psm_->startSceneMonitor(config_.monitored_planning_scene_topic);
  psm_->startStateMonitor(config_.joint_states_topic, config_.attached_object_topic);
  // Deliberately NOT startWorldGeometryMonitor(): it spins up octomap updaters, which is a real
  // cost and this robot has no depth sensors.
  psm_->requestPlanningSceneState(config_.planning_scene_service);

  RCLCPP_INFO(logger(), "planning scene monitor started for robot model '%s'",
              psm_->getRobotModel() ? psm_->getRobotModel()->getName().c_str() : "<none>");
}

planning_scene_monitor::PlanningSceneMonitorPtr BehaviorContext::planningSceneMonitor()
{
  std::lock_guard<std::mutex> lock(moveit_mutex_);
  ensureMoveItStarted();
  return psm_;
}

moveit::core::RobotModelConstPtr BehaviorContext::robotModel()
{
  const auto psm = planningSceneMonitor();
  return psm ? psm->getRobotModel() : nullptr;
}

moveit::core::RobotStatePtr BehaviorContext::currentState()
{
  const auto psm = planningSceneMonitor();
  if (!psm)
  {
    return nullptr;
  }
  planning_scene_monitor::LockedPlanningSceneRO scene(psm);
  return std::make_shared<moveit::core::RobotState>(scene->getCurrentState());
}

std::shared_ptr<moveit::planning_interface::MoveGroupInterface> BehaviorContext::moveGroup(const std::string& group)
{
  const std::string name = group.empty() ? config_.default_planning_group : group;

  std::lock_guard<std::mutex> lock(moveit_mutex_);
  const auto it = move_groups_.find(name);
  if (it != move_groups_.end())
  {
    return it->second;
  }

  moveit::planning_interface::MoveGroupInterface::Options options(name, config_.robot_description_param,
                                                                  node_->get_namespace());
  try
  {
    auto mgi = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        moveit_client_node_, options, tf_buffer_, rclcpp::Duration::from_seconds(config_.moveit_connect_timeout));
    move_groups_[name] = mgi;
    return mgi;
  }
  catch (const std::exception& exc)
  {
    // Cached as null so a tree that keeps asking does not pay the connect timeout every tick.
    RCLCPP_ERROR(logger(), "cannot create MoveGroupInterface for group '%s': %s", name.c_str(), exc.what());
    move_groups_[name] = nullptr;
    return nullptr;
  }
}

BehaviorParameters BehaviorContext::parametersFor(const std::string& registration_name) const
{
  return behavior_parameters_.forBehavior(registration_name);
}

}  // namespace moveit2_extended
