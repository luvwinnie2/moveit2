// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_core/behavior_parameters.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace moveit2_extended
{

struct BehaviorContextConfig
{
  std::string default_planning_group = "arm";
  std::string robot_description_param = "robot_description";
  std::string move_group_action = "move_action";
  std::string execute_trajectory_action = "execute_trajectory";
  std::string monitored_planning_scene_topic = "/monitored_planning_scene";
  std::string joint_states_topic = "/joint_states";
  std::string attached_object_topic = "/attached_collision_object";
  std::string planning_scene_service = "/get_planning_scene";
  /** Seconds MoveGroupInterface waits for move_group's servers on first use. */
  double moveit_connect_timeout = 10.0;
  /** When false, planningSceneMonitor() stays null. Lets the Objective server come up, list its
   *  Objectives and run non-MoveIt Behaviors on a machine where move_group is not running. */
  bool start_planning_scene_monitor = true;
};

/** Shared resources handed to every Behavior, and the only sanctioned route from a Behavior to
 *  ROS, TF or MoveIt.
 *
 *  THREADING CONTRACT -- read this before adding a member:
 *
 *   - node() is spun by the Objective server's MultiThreadedExecutor. Anything created on it with
 *     the default callback group runs concurrently with a tick. Behaviors must therefore use
 *     makeCallbackIsland() instead, and pump it from their own tick.
 *   - moveGroup() lives on a node that no external executor owns, because MoveGroupInterface spins
 *     it on a thread it creates itself. Handing that node to our executor as well would deliver
 *     every callback twice.
 *   - planningSceneMonitor() runs on its own private node and threads. Never hold
 *     LockedPlanningSceneRO across a spin or an action wait.
 *   - MoveIt is started lazily on first use, so the server does not have to come up after
 *     move_group. The first call therefore may block for up to moveit_connect_timeout. */
class BehaviorContext
{
public:
  BehaviorContext(rclcpp::Node::SharedPtr node, BehaviorContextConfig config,
                  BehaviorParameterMap behavior_parameters);
  ~BehaviorContext();

  BehaviorContext(const BehaviorContext&) = delete;
  BehaviorContext& operator=(const BehaviorContext&) = delete;

  // ---- always available ----------------------------------------------------------------------
  const rclcpp::Node::SharedPtr& node() const
  {
    return node_;
  }
  rclcpp::Logger logger() const
  {
    return node_->get_logger();
  }
  rclcpp::Clock::SharedPtr clock() const
  {
    return node_->get_clock();
  }
  const std::shared_ptr<tf2_ros::Buffer>& tf() const
  {
    return tf_buffer_;
  }
  const BehaviorContextConfig& config() const
  {
    return config_;
  }

  // ---- MoveIt: any of these may be null when move_group is not running ------------------------
  planning_scene_monitor::PlanningSceneMonitorPtr planningSceneMonitor();
  moveit::core::RobotModelConstPtr robotModel();
  /** A copy of the current state. Takes and releases the scene lock internally, so the caller
   *  never holds it. Null when the monitor is unavailable. */
  moveit::core::RobotStatePtr currentState();

  /** MoveGroupInterface for `group` (empty = the configured default), created on first use and
   *  cached. Null when move_group did not answer within the timeout.
   *
   *  Only for cheap queries -- named targets, end-effector link, planning frame. Motion goes
   *  through the move_group action clients instead, because MoveGroupInterface::move() blocks and
   *  cannot be cancelled from another thread, and a halt() that cannot halt is worse than none. */
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroup(const std::string& group = "");

  // ---- per-Behavior ROS isolation --------------------------------------------------------------
  /** A callback group deliberately kept out of the node's executor, plus a private executor that
   *  owns it. A Behavior pumps this from its own tick, so its subscriptions and action callbacks
   *  run on the tick thread and can touch the Behavior's own state without a lock.
   *
   *  This is the pattern nav2_behavior_tree::BtActionNode uses, and it is what keeps ROS
   *  callbacks off the blackboard. */
  struct CallbackIsland
  {
    rclcpp::CallbackGroup::SharedPtr group;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

    /** Bounded pump. Returns immediately when nothing is ready. */
    void pump(std::chrono::nanoseconds max_duration = std::chrono::milliseconds(1)) const;
    bool valid() const
    {
      return group && executor;
    }
  };
  CallbackIsland makeCallbackIsland();

  // ---- Behavior Configuration Parameters --------------------------------------------------------
  BehaviorParameters parametersFor(const std::string& registration_name) const;
  const BehaviorParameterMap& allBehaviorParameters() const
  {
    return behavior_parameters_;
  }

private:
  /** Idempotent, mutex-guarded. Only ever attempted once: if move_group is not up, retrying on
   *  every tick would stall the tree for the timeout each time. */
  void ensureMoveItStarted();

  rclcpp::Node::SharedPtr node_;
  /** Never handed to any executor. See the threading contract above. */
  rclcpp::Node::SharedPtr moveit_client_node_;
  BehaviorContextConfig config_;
  BehaviorParameterMap behavior_parameters_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex moveit_mutex_;
  bool moveit_start_attempted_ = false;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  std::unordered_map<std::string, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>> move_groups_;
};

using BehaviorContextPtr = std::shared_ptr<BehaviorContext>;

}  // namespace moveit2_extended
