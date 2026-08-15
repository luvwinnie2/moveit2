// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// Owns the arm's taught targets and serves them.
//
// Uses a PlanningSceneMonitor with only the state monitor started, rather than a
// MoveGroupInterface: MGI spins a node on its own thread, and this node needs nothing from
// move_group except the current joint values and forward kinematics -- both of which the monitor
// already provides, without a second executor in the process.

#include <moveit2_extended_waypoints/waypoint_store.hpp>

#include <moveit2_extended_msgs/srv/delete_waypoint.hpp>
#include <moveit2_extended_msgs/srv/get_waypoint.hpp>
#include <moveit2_extended_msgs/srv/list_waypoints.hpp>
#include <moveit2_extended_msgs/srv/rename_waypoint.hpp>
#include <moveit2_extended_msgs/srv/save_waypoint.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <filesystem>
#include <memory>

namespace moveit2_extended::waypoints
{
namespace srvs = moveit2_extended_msgs::srv;

class WaypointManager
{
public:
  explicit WaypointManager(const rclcpp::NodeOptions& options)
    : node_(std::make_shared<rclcpp::Node>("arm_waypoint_manager", options))
  {
    node_->declare_parameter<std::string>("waypoint_file", defaultFile());
    node_->declare_parameter<std::string>("seed_file", "");
    node_->declare_parameter<std::string>("planning_group", "arm");
    node_->declare_parameter<std::string>("default_frame_id", "");
    node_->declare_parameter<std::string>("default_pose_link", "");
    node_->declare_parameter<std::string>("robot_description_param", "robot_description");
    node_->declare_parameter<bool>("autosave", true);
    node_->declare_parameter<bool>("publish_markers", true);
    node_->declare_parameter<double>("marker_rate", 1.0);

    file_ = node_->get_parameter("waypoint_file").as_string();
    autosave_ = node_->get_parameter("autosave").as_bool();
    store_.default_group = node_->get_parameter("planning_group").as_string();
    store_.default_frame_id = node_->get_parameter("default_frame_id").as_string();
    store_.default_pose_link = node_->get_parameter("default_pose_link").as_string();

    loadOrSeed();
    startSceneMonitor();
    createInterfaces();

    RCLCPP_INFO(node_->get_logger(), "arm waypoint manager ready: %zu waypoint(s) in %s", store_.size(),
                file_.c_str());
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const
  {
    return node_->get_node_base_interface();
  }

private:
  static std::string defaultFile()
  {
    const char* home = std::getenv("HOME");
    return std::string(home ? home : "/tmp") + "/.ros/arm_waypoints.yaml";
  }

  void loadOrSeed()
  {
    std::error_code ec;
    const std::string seed = node_->get_parameter("seed_file").as_string();
    // A seed file is copied in only when there is nothing yet, so a deployment can ship a starting
    // set of targets without overwriting what an operator has since taught.
    if (!seed.empty() && !std::filesystem::exists(file_, ec))
    {
      std::string error;
      if (store_.loadFromFile(seed, &error))
      {
        RCLCPP_INFO(node_->get_logger(), "seeded %zu waypoint(s) from %s", store_.size(), seed.c_str());
        save();
        return;
      }
      RCLCPP_ERROR(node_->get_logger(), "cannot read the seed file %s: %s", seed.c_str(), error.c_str());
    }

    std::string error;
    if (!store_.loadFromFile(file_, &error))
    {
      RCLCPP_ERROR(node_->get_logger(), "cannot read %s: %s -- starting empty, and NOT overwriting it",
                   file_.c_str(), error.c_str());
      // Autosave is disabled so a parse error in a file full of taught targets cannot be turned
      // into an empty file by the next save.
      autosave_ = false;
    }
  }

  void startSceneMonitor()
  {
    try
    {
      psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
          node_, node_->get_parameter("robot_description_param").as_string(), "waypoint_psm");
    }
    catch (const std::exception& exc)
    {
      RCLCPP_ERROR(node_->get_logger(), "cannot start the planning scene monitor: %s", exc.what());
      return;
    }
    if (!psm_->getPlanningScene())
    {
      RCLCPP_ERROR(node_->get_logger(), "no robot model; teaching from the current state will not work");
      psm_.reset();
      return;
    }
    // State only: this node never needs the world, and startWorldGeometryMonitor would spin up
    // octomap updaters for nothing.
    psm_->startStateMonitor();
  }

  void createInterfaces()
  {
    using namespace std::placeholders;
    save_srv_ = node_->create_service<srvs::SaveWaypoint>("~/save", std::bind(&WaypointManager::onSave, this, _1, _2));
    list_srv_ = node_->create_service<srvs::ListWaypoints>("~/list", std::bind(&WaypointManager::onList, this, _1, _2));
    get_srv_ = node_->create_service<srvs::GetWaypoint>("~/get", std::bind(&WaypointManager::onGet, this, _1, _2));
    delete_srv_ =
        node_->create_service<srvs::DeleteWaypoint>("~/delete", std::bind(&WaypointManager::onDelete, this, _1, _2));
    rename_srv_ =
        node_->create_service<srvs::RenameWaypoint>("~/rename", std::bind(&WaypointManager::onRename, this, _1, _2));

    // Latched, so a UI that connects later sees the current set without asking.
    list_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/markers", rclcpp::QoS(1).transient_local().reliable());

    if (node_->get_parameter("publish_markers").as_bool())
    {
      const double rate = std::max(0.1, node_->get_parameter("marker_rate").as_double());
      marker_timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / rate),
                                               [this]() { publishMarkers(); });
    }
  }

  bool save()
  {
    std::string error;
    if (!store_.saveToFile(file_, &error))
    {
      RCLCPP_ERROR(node_->get_logger(), "cannot write %s: %s", file_.c_str(), error.c_str());
      return false;
    }
    return true;
  }

  void onSave(const std::shared_ptr<srvs::SaveWaypoint::Request> request,
              std::shared_ptr<srvs::SaveWaypoint::Response> response)
  {
    if (request->name.empty())
    {
      response->success = false;
      response->message = "a waypoint needs a name";
      return;
    }

    ArmWaypoint waypoint;
    waypoint.name = request->name;
    waypoint.group = request->group.empty() ? store_.default_group : request->group;
    waypoint.description = request->description;
    waypoint.tags = request->tags;
    waypoint.created = node_->get_clock()->now();
    waypoint.modified = waypoint.created;

    if (request->capture == srvs::SaveWaypoint::Request::CAPTURE_GIVEN)
    {
      waypoint.has_joint_state = !request->joint_state.name.empty();
      waypoint.joint_state = request->joint_state;
      waypoint.has_pose = !request->pose.header.frame_id.empty();
      waypoint.pose = request->pose;
      waypoint.pose_link = request->pose_link.empty() ? store_.default_pose_link : request->pose_link;
    }
    else
    {
      if (!psm_)
      {
        response->success = false;
        response->message = "no robot state available; is move_group running?";
        return;
      }
      moveit::core::RobotState state(psm_->getRobotModel());
      {
        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        state = scene->getCurrentState();
      }
      state.update();

      const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(waypoint.group);
      if (!jmg)
      {
        response->success = false;
        response->message = "no planning group named '" + waypoint.group + "'";
        return;
      }

      const bool want_joints = request->capture != srvs::SaveWaypoint::Request::CAPTURE_CURRENT_POSE;
      const bool want_pose = request->capture != srvs::SaveWaypoint::Request::CAPTURE_CURRENT_JOINTS;

      if (want_joints)
      {
        waypoint.has_joint_state = true;
        waypoint.joint_state.header.stamp = node_->get_clock()->now();
        for (const auto& joint : jmg->getActiveJointModelNames())
        {
          waypoint.joint_state.name.push_back(joint);
          waypoint.joint_state.position.push_back(state.getVariablePosition(joint));
        }
      }

      if (want_pose)
      {
        std::string link = request->pose_link.empty() ? store_.default_pose_link : request->pose_link;
        if (link.empty())
        {
          link = jmg->getLinkModelNames().empty() ? "" : jmg->getLinkModelNames().back();
        }
        if (link.empty() || !state.getRobotModel()->hasLinkModel(link))
        {
          response->success = false;
          response->message = "cannot work out which link to record a pose for";
          return;
        }
        waypoint.has_pose = true;
        waypoint.pose_link = link;
        // The frame is recorded explicitly rather than defaulted: on a mobile base a pose in
        // robot_base is robot-relative, and replaying it after the base has driven would send the
        // arm somewhere else entirely.
        waypoint.pose.header.frame_id = state.getRobotModel()->getModelFrame();
        waypoint.pose.header.stamp = node_->get_clock()->now();
        waypoint.pose.pose = tf2::toMsg(state.getGlobalLinkTransform(link));
      }
    }

    std::string error;
    if (!store_.add(waypoint, request->overwrite, &error))
    {
      response->success = false;
      response->message = error;
      return;
    }
    if (autosave_ && !save())
    {
      response->success = false;
      response->message = "saved in memory but could not write " + file_;
      return;
    }

    response->success = true;
    response->message = "saved";
    response->waypoint = waypoint;
    publishMarkers();
    RCLCPP_INFO(node_->get_logger(), "taught '%s'", waypoint.name.c_str());
  }

  void onList(const std::shared_ptr<srvs::ListWaypoints::Request> request,
              std::shared_ptr<srvs::ListWaypoints::Response> response)
  {
    response->waypoints = store_.list(request->tag_filter);
    response->success = true;
  }

  void onGet(const std::shared_ptr<srvs::GetWaypoint::Request> request,
             std::shared_ptr<srvs::GetWaypoint::Response> response)
  {
    const ArmWaypoint* waypoint = store_.find(request->name);
    if (!waypoint)
    {
      response->success = false;
      response->message = "no waypoint named '" + request->name + "'";
      return;
    }
    response->success = true;
    response->waypoint = *waypoint;
  }

  void onDelete(const std::shared_ptr<srvs::DeleteWaypoint::Request> request,
                std::shared_ptr<srvs::DeleteWaypoint::Response> response)
  {
    std::string error;
    response->success = store_.remove(request->name, &error);
    response->message = response->success ? "deleted" : error;
    if (response->success && autosave_)
    {
      save();
      publishMarkers();
    }
  }

  void onRename(const std::shared_ptr<srvs::RenameWaypoint::Request> request,
                std::shared_ptr<srvs::RenameWaypoint::Response> response)
  {
    std::string error;
    response->success = store_.rename(request->name, request->new_name, request->overwrite, &error);
    response->message = response->success ? "renamed" : error;
    if (response->success && autosave_)
    {
      save();
      publishMarkers();
    }
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int id = 0;
    for (const auto& waypoint : store_.list())
    {
      if (!waypoint.has_pose)
      {
        continue;  // nothing to draw for a joint-only target
      }
      visualization_msgs::msg::Marker arrow;
      arrow.header = waypoint.pose.header;
      arrow.header.stamp = node_->get_clock()->now();
      arrow.ns = "arm_waypoints";
      arrow.id = id++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.pose = waypoint.pose.pose;
      arrow.scale.x = 0.08;
      arrow.scale.y = 0.008;
      arrow.scale.z = 0.008;
      arrow.color.r = 0.2f;
      arrow.color.g = 0.7f;
      arrow.color.b = 1.0f;
      arrow.color.a = 0.9f;
      markers.markers.push_back(arrow);

      visualization_msgs::msg::Marker label = arrow;
      label.id = id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.text = waypoint.name;
      label.scale.z = 0.03;
      label.pose.position.z += 0.04;
      markers.markers.push_back(label);
    }
    list_pub_->publish(markers);
  }

  rclcpp::Node::SharedPtr node_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  WaypointStore store_;
  std::string file_;
  bool autosave_ = true;

  rclcpp::Service<srvs::SaveWaypoint>::SharedPtr save_srv_;
  rclcpp::Service<srvs::ListWaypoints>::SharedPtr list_srv_;
  rclcpp::Service<srvs::GetWaypoint>::SharedPtr get_srv_;
  rclcpp::Service<srvs::DeleteWaypoint>::SharedPtr delete_srv_;
  rclcpp::Service<srvs::RenameWaypoint>::SharedPtr rename_srv_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr list_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
};

}  // namespace moveit2_extended::waypoints

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto manager = std::make_shared<moveit2_extended::waypoints::WaypointManager>(rclcpp::NodeOptions{});
  executor.add_node(manager->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
