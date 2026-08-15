// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// Behaviors for changing the end-effector from inside an Objective.
//
// MoveIt Pro's equivalents are AddURDF / AttachURDF / DetachURDF / RemoveURDFFromScene, which
// carry a whole URDF sub-tree into the planning scene and track its joints from /joint_states.
// These do the same job with primitive collision shapes from a registry, because that is what is
// actually available here -- the Isaac gripper assets are USD references, not meshes, and a tool
// added as an empty shape would collide with nothing at all. The names are kept short and honest
// rather than borrowed for something narrower than what Pro's do.
#include <moveit2_extended_core/behavior_loader_base.hpp>
#include <moveit2_extended_core/service_client_behavior_base.hpp>

#include <moveit2_extended_msgs/srv/get_active_tool.hpp>
#include <moveit2_extended_msgs/srv/list_tools.hpp>
#include <moveit2_extended_msgs/srv/switch_tool.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace moveit2_extended::tools
{
namespace srvs = moveit2_extended_msgs::srv;

/** Fit a different end-effector. */
class SwitchTool : public ServiceClientBehaviorBase<srvs::SwitchTool>
{
public:
  SwitchTool(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources)
    : ServiceClientBehaviorBase<srvs::SwitchTool>(name, config, std::move(shared_resources),
                                                   "/tool_manager/switch")
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::string>("tool_name", "which end-effector to fit"),
        BT::OutputPort<geometry_msgs::msg::Pose>("tcp_offset",
                                                 "the new tool's working point relative to the mount link; "
                                                 "wire this into the Cartesian Behaviors' target offset"),
        BT::OutputPort<std::string>("carrier_config", "the dresspack configuration this tool expects"),
    });
  }

protected:
  BtExpected<std::shared_ptr<Request>> createRequest() override
  {
    const auto name = getInputOr<std::string>("tool_name", std::string(""));
    if (name.empty())
    {
      return nonstd::make_unexpected("tool_name is required; use DetachTool to fit nothing");
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
    setOutput("tcp_offset", response->active_tool.tcp_offset);
    setOutput("carrier_config", response->active_tool.carrier_config);
    return BtStatus::SUCCESS;
  }
};

/** Remove whatever is fitted, leaving a bare mount. */
class DetachTool : public ServiceClientBehaviorBase<srvs::SwitchTool>
{
public:
  DetachTool(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources)
    : ServiceClientBehaviorBase<srvs::SwitchTool>(name, config, std::move(shared_resources),
                                                   "/tool_manager/switch")
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

protected:
  BtExpected<std::shared_ptr<Request>> createRequest() override
  {
    auto request = std::make_shared<Request>();
    request->detach_only = true;
    return request;
  }

  BtStatus processResponse(const std::shared_ptr<Response>& response) override
  {
    if (!response->success)
    {
      RCLCPP_ERROR(getLogger(), "%s", response->message.c_str());
      return BtStatus::FAILURE;
    }
    return BtStatus::SUCCESS;
  }
};

/** What is fitted right now, and its TCP offset. */
class GetActiveTool : public ServiceClientBehaviorBase<srvs::GetActiveTool>
{
public:
  GetActiveTool(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources)
    : ServiceClientBehaviorBase<srvs::GetActiveTool>(name, config, std::move(shared_resources),
                                                      "/tool_manager/active")
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::string>("expected_tool", "",
                                   "when set, FAIL unless this is what is fitted -- so an Objective "
                                   "that needs the cutter cannot silently run with a gripper on"),
        BT::OutputPort<std::string>("tool_name", ""),
        BT::OutputPort<geometry_msgs::msg::Pose>("tcp_offset", ""),
        BT::OutputPort<std::string>("carrier_config", ""),
        BT::OutputPort<bool>("has_tool", ""),
    });
  }

protected:
  BtExpected<std::shared_ptr<Request>> createRequest() override
  {
    return std::make_shared<Request>();
  }

  BtStatus processResponse(const std::shared_ptr<Response>& response) override
  {
    setOutput("has_tool", response->has_tool);
    setOutput("tool_name", response->tool.name);
    setOutput("tcp_offset", response->tool.tcp_offset);
    setOutput("carrier_config", response->tool.carrier_config);

    const auto expected = getInputOr<std::string>("expected_tool", std::string(""));
    if (!expected.empty() && response->tool.name != expected)
    {
      RCLCPP_ERROR(getLogger(), "expected the '%s' to be fitted, but it is '%s'", expected.c_str(),
                   response->has_tool ? response->tool.name.c_str() : "<nothing>");
      return BtStatus::FAILURE;
    }
    return BtStatus::SUCCESS;
  }
};

/** The tools this robot can be fitted with. */
class ListTools : public ServiceClientBehaviorBase<srvs::ListTools>
{
public:
  ListTools(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources)
    : ServiceClientBehaviorBase<srvs::ListTools>(name, config, std::move(shared_resources), "/tool_manager/list")
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::OutputPort<std::vector<std::string>>("names", "semicolon-separated"),
        BT::OutputPort<int>("count", ""),
    });
  }

protected:
  BtExpected<std::shared_ptr<Request>> createRequest() override
  {
    return std::make_shared<Request>();
  }

  BtStatus processResponse(const std::shared_ptr<Response>& response) override
  {
    std::vector<std::string> names;
    for (const auto& tool : response->tools)
    {
      names.push_back(tool.name);
    }
    setOutput("names", names);
    setOutput("count", static_cast<int>(names.size()));
    return BtStatus::SUCCESS;
  }
};

class ToolBehaviorsLoader : public BehaviorLoaderBase
{
public:
  void registerBehaviors(BtFactory& factory, const BehaviorContextPtr& shared_resources) override
  {
    registerBehavior<SwitchTool>(factory, "SwitchTool", shared_resources);
    registerBehavior<DetachTool>(factory, "DetachTool", shared_resources);
    registerBehavior<GetActiveTool>(factory, "GetActiveTool", shared_resources);
    registerBehavior<ListTools>(factory, "ListTools", shared_resources);
  }

  std::unordered_map<std::string, std::string> behaviorDescriptions() const override
  {
    return {
      { "SwitchTool", "Fit a different end-effector, as an attached collision object." },
      { "DetachTool", "Remove whatever end-effector is fitted." },
      { "GetActiveTool", "What is fitted now; optionally fail unless it is what was expected." },
      { "ListTools", "The end-effectors this robot can be fitted with." },
    };
  }
};

}  // namespace moveit2_extended::tools

PLUGINLIB_EXPORT_CLASS(moveit2_extended::tools::ToolBehaviorsLoader, moveit2_extended::BehaviorLoaderBase)
