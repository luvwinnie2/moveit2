// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// The Behaviors an Objective uses to reach a taught target. Names follow MoveIt Pro's.
#include <moveit2_extended_core/behavior_loader_base.hpp>
#include <moveit2_extended_core/service_client_behavior_base.hpp>

#include <moveit2_extended_msgs/srv/get_waypoint.hpp>
#include <moveit2_extended_msgs/srv/list_waypoints.hpp>
#include <moveit2_extended_msgs/srv/save_waypoint.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>

namespace moveit2_extended::waypoints
{
namespace srvs = moveit2_extended_msgs::srv;

/** Fetch a taught target by name.
 *
 *  Fails when the name is unknown -- which is what lets a Fallback recover by teaching one, or by
 *  telling the operator which names DO exist. Silently returning an empty target would send the
 *  arm to the zero pose, which on this robot is a singularity. */
class RetrieveWaypoint : public ServiceClientBehaviorBase<srvs::GetWaypoint>
{
public:
  RetrieveWaypoint(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources)
    : ServiceClientBehaviorBase<srvs::GetWaypoint>(name, config, std::move(shared_resources),
                                                    "/arm_waypoint_manager/get")
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::string>("waypoint_name", "which taught target"),
        BT::OutputPort<sensor_msgs::msg::JointState>("joint_state", "its joint values, when it has any"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "its Cartesian pose, when it has one"),
        BT::OutputPort<std::string>("pose_link", "the link that pose describes"),
        BT::OutputPort<std::string>("tool", "the end-effector fitted when it was taught"),
        BT::OutputPort<bool>("has_joint_state", ""),
        BT::OutputPort<bool>("has_pose", ""),
    });
  }

protected:
  BtExpected<std::shared_ptr<Request>> createRequest() override
  {
    const auto name = getInputOr<std::string>("waypoint_name", std::string(""));
    if (name.empty())
    {
      return nonstd::make_unexpected("waypoint_name is required");
    }
    auto request = std::make_shared<Request>();
    request->name = name;
    return request;
  }

  BtStatus processResponse(const std::shared_ptr<Response>& response) override
  {
    if (!response->success)
    {
      RCLCPP_ERROR(getLogger(), "%s", response->message.c_str());
      return BtStatus::FAILURE;
    }
    const auto& waypoint = response->waypoint;
    setOutput("joint_state", waypoint.joint_state);
    setOutput("pose", waypoint.pose);
    setOutput("pose_link", waypoint.pose_link);
    setOutput("tool", waypoint.tool);
    setOutput("has_joint_state", waypoint.has_joint_state);
    setOutput("has_pose", waypoint.has_pose);

    if (!waypoint.has_joint_state && !waypoint.has_pose)
    {
      RCLCPP_ERROR(getLogger(), "waypoint '%s' has neither joints nor a pose", waypoint.name.c_str());
      return BtStatus::FAILURE;
    }
    return BtStatus::SUCCESS;
  }
};

/** Teach a target from where the robot is now. */
class TeachWaypoint : public ServiceClientBehaviorBase<srvs::SaveWaypoint>
{
public:
  TeachWaypoint(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources)
    : ServiceClientBehaviorBase<srvs::SaveWaypoint>(name, config, std::move(shared_resources),
                                                     "/arm_waypoint_manager/save")
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::string>("waypoint_name", "what to call it"),
        BT::InputPort<std::string>("planning_group", "", "empty uses the manager's default"),
        BT::InputPort<std::string>("description", "", ""),
        BT::InputPort<std::vector<std::string>>("tags", "labels, for filtering later"),
        BT::InputPort<std::string>("capture", "both", "joints | pose | both"),
        BT::InputPort<bool>("overwrite", false, ""),
        BT::InputPort<bool>("record_carrier_diagnostics", true,
                            "record what the dresspack was doing here, so a target that was always "
                            "marginal can be recognised later"),
        BT::OutputPort<moveit2_extended_msgs::msg::ArmWaypoint>("waypoint", ""),
    });
  }

protected:
  BtExpected<std::shared_ptr<Request>> createRequest() override
  {
    const auto name = getInputOr<std::string>("waypoint_name", std::string(""));
    if (name.empty())
    {
      return nonstd::make_unexpected("waypoint_name is required");
    }

    auto request = std::make_shared<Request>();
    request->name = name;
    request->group = getInputOr<std::string>("planning_group", std::string(""));
    request->description = getInputOr<std::string>("description", std::string(""));
    request->tags = getInputOr<std::vector<std::string>>("tags", std::vector<std::string>{});
    request->overwrite = getInputOr<bool>("overwrite", false);
    request->record_carrier_diagnostics = getInputOr<bool>("record_carrier_diagnostics", true);

    const auto capture = getInputOr<std::string>("capture", std::string("both"));
    if (capture == "joints")
    {
      request->capture = Request::CAPTURE_CURRENT_JOINTS;
    }
    else if (capture == "pose")
    {
      request->capture = Request::CAPTURE_CURRENT_POSE;
    }
    else if (capture == "both")
    {
      request->capture = Request::CAPTURE_CURRENT_BOTH;
    }
    else
    {
      return nonstd::make_unexpected("capture must be joints, pose or both; got '" + capture + "'");
    }
    return request;
  }

  BtStatus processResponse(const std::shared_ptr<Response>& response) override
  {
    if (!response->success)
    {
      RCLCPP_ERROR(getLogger(), "%s", response->message.c_str());
      return BtStatus::FAILURE;
    }
    setOutput("waypoint", response->waypoint);
    return BtStatus::SUCCESS;
  }
};

/** The taught names, for iterating over a row of targets. */
class ListWaypoints : public ServiceClientBehaviorBase<srvs::ListWaypoints>
{
public:
  ListWaypoints(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources)
    : ServiceClientBehaviorBase<srvs::ListWaypoints>(name, config, std::move(shared_resources),
                                                      "/arm_waypoint_manager/list")
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::string>("tag_filter", "", "only targets carrying this tag; empty means all"),
        BT::OutputPort<std::vector<std::string>>("names", "semicolon-separated, for GetElementOfVector"),
        BT::OutputPort<int>("count", ""),
    });
  }

protected:
  BtExpected<std::shared_ptr<Request>> createRequest() override
  {
    auto request = std::make_shared<Request>();
    request->tag_filter = getInputOr<std::string>("tag_filter", std::string(""));
    return request;
  }

  BtStatus processResponse(const std::shared_ptr<Response>& response) override
  {
    std::vector<std::string> names;
    names.reserve(response->waypoints.size());
    for (const auto& waypoint : response->waypoints)
    {
      names.push_back(waypoint.name);
    }
    setOutput("names", names);
    setOutput("count", static_cast<int>(names.size()));
    return BtStatus::SUCCESS;
  }
};

class WaypointBehaviorsLoader : public BehaviorLoaderBase
{
public:
  void registerBehaviors(BtFactory& factory, const BehaviorContextPtr& shared_resources) override
  {
    registerBehavior<RetrieveWaypoint>(factory, "RetrieveWaypoint", shared_resources);
    registerBehavior<TeachWaypoint>(factory, "TeachWaypoint", shared_resources);
    registerBehavior<ListWaypoints>(factory, "ListWaypoints", shared_resources);
  }

  std::unordered_map<std::string, std::string> behaviorDescriptions() const override
  {
    return {
      { "RetrieveWaypoint", "Fetch a taught arm target by name. Fails when the name is unknown." },
      { "TeachWaypoint", "Save the robot's current state as a named target." },
      { "ListWaypoints", "The taught names, optionally filtered by tag." },
    };
  }
};

}  // namespace moveit2_extended::waypoints

PLUGINLIB_EXPORT_CLASS(moveit2_extended::waypoints::WaypointBehaviorsLoader, moveit2_extended::BehaviorLoaderBase)
