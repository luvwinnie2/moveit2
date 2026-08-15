// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_gpu/gpu_behaviors.hpp>

#include <moveit2_extended_behaviors/moveit_goal_helpers.hpp>
#include <moveit2_extended_behaviors/port_conversions.hpp>

#include <string>
#include <vector>

namespace moveit2_extended::gpu
{
namespace
{
constexpr const char* kIkService = "/curobo_service/solve_ik_batch";
constexpr const char* kPlanService = "/curobo_service/plan_to_goal";

/** Ports every GPU Behavior shares with its CPU twin, so an Objective can switch between them by
 *  changing only the node name in the XML. */
BT::PortsList commonGoalPorts()
{
  return {
    BT::InputPort<std::string>("planning_group", "", "empty uses the configured default"),
    BT::InputPort<double>("velocity_scaling_factor", 0.0, "0 uses the service default"),
    BT::InputPort<double>("acceleration_scaling_factor", 0.0, "0 uses the service default"),
    BT::InputPort<moveit_msgs::msg::RobotState>("start_state",
                                                "where to plan FROM; empty uses the live robot. "
                                                "Chain a previous stage's end_state in here to plan "
                                                "a whole sequence before moving"),
    BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("trajectory", "the plan"),
    BT::OutputPort<moveit_msgs::msg::RobotState>("end_state", "feed into the next stage's start_state"),
    BT::OutputPort<double>("planning_time", "seconds spent on the GPU"),
  };
}

BtStatus reportPlan(const std::shared_ptr<moveit2_extended_msgs::srv::PlanWithGpu::Response>& response,
                    BT::TreeNode& node, const rclcpp::Logger& logger)
{
  if (!response->success)
  {
    RCLCPP_ERROR(logger, "GPU planning failed: %s", response->message.c_str());
    return BtStatus::FAILURE;
  }
  node.setOutput("trajectory", response->trajectory);
  node.setOutput("end_state", response->end_state);
  node.setOutput("planning_time", response->planning_time);
  RCLCPP_INFO(logger, "%s", response->message.c_str());
  return BtStatus::SUCCESS;
}
}  // namespace

// ---------------------------------------------------------------------------------------------
// SolveIKQueriesGPU
// ---------------------------------------------------------------------------------------------

SolveIKQueriesGPU::SolveIKQueriesGPU(const std::string& name, const NodeConfig& config,
                                     BehaviorContextPtr shared_resources)
  : ServiceClientBehaviorBase<moveit2_extended_msgs::srv::SolveIKBatch>(name, config, std::move(shared_resources),
                                                                        kIkService)
{
}

BT::PortsList SolveIKQueriesGPU::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<std::string>("planning_group", "", "empty uses the configured default"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
          "poses", "candidate poses, '|'-separated; every one is solved in a single batch"),
      BT::InputPort<std::string>("target_link", "",
                                 "which link the poses describe. Empty uses the service's tool "
                                 "frame; set it to the working point (e.g. cutting_point) and the "
                                 "offset is composed away before solving"),
      BT::InputPort<double>("position_tolerance", 0.0, "metres; 0 uses the service default"),
      BT::InputPort<double>("orientation_tolerance", 0.0, "radians; 0 uses the service default"),
      BT::InputPort<bool>("collision_free", true, "reject solutions that collide"),
      BT::InputPort<bool>("require_all", false,
                          "FAIL unless every pose was solved. Off by default: the usual question "
                          "is 'which of these can I reach', and none-reachable is the only real "
                          "failure"),
      BT::OutputPort<sensor_msgs::msg::JointState>("first_solution", "the first pose that solved"),
      BT::OutputPort<int>("solved_count", "how many of the poses solved"),
      BT::OutputPort<int>("first_solved_index", "index of the first solved pose, -1 if none"),
      BT::OutputPort<double>("solve_time", "seconds for the whole batch"),
  });
}

BtExpected<std::shared_ptr<SolveIKQueriesGPU::Request>> SolveIKQueriesGPU::createRequest()
{
  const auto poses = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("poses");
  if (!poses)
  {
    return nonstd::make_unexpected("poses: " + poses.error());
  }
  if (poses->empty())
  {
    return nonstd::make_unexpected("poses is empty; there is nothing to solve");
  }

  auto request = std::make_shared<Request>();
  request->planning_group = getInputOr<std::string>("planning_group", "");
  request->target_link = getInputOr<std::string>("target_link", "");
  request->poses = poses.value();
  request->position_tolerance = getInputOr<double>("position_tolerance", 0.0);
  request->orientation_tolerance = getInputOr<double>("orientation_tolerance", 0.0);
  request->collision_free = getInputOr<bool>("collision_free", true);
  return request;
}

BtStatus SolveIKQueriesGPU::processResponse(const std::shared_ptr<Response>& response)
{
  if (!response->success)
  {
    RCLCPP_ERROR(getLogger(), "batch IK failed: %s", response->message.c_str());
    return BtStatus::FAILURE;
  }

  int solved = 0;
  int first = -1;
  for (size_t index = 0; index < response->solved.size(); ++index)
  {
    if (!response->solved[index])
    {
      continue;
    }
    ++solved;
    if (first < 0)
    {
      first = static_cast<int>(index);
    }
  }

  setOutput("solved_count", solved);
  setOutput("first_solved_index", first);
  setOutput("solve_time", response->solve_time);
  if (first >= 0)
  {
    setOutput("first_solution", response->solutions[static_cast<size_t>(first)]);
  }

  RCLCPP_INFO(getLogger(), "%s", response->message.c_str());

  const bool require_all = getInputOr<bool>("require_all", false);
  if (require_all && solved != static_cast<int>(response->solved.size()))
  {
    RCLCPP_ERROR(getLogger(), "require_all is set and only %d of %zu poses solved", solved,
                 response->solved.size());
    return BtStatus::FAILURE;
  }
  return solved > 0 ? BtStatus::SUCCESS : BtStatus::FAILURE;
}

// ---------------------------------------------------------------------------------------------
// PlanToJointGoalGPU
// ---------------------------------------------------------------------------------------------

PlanToJointGoalGPU::PlanToJointGoalGPU(const std::string& name, const NodeConfig& config,
                                       BehaviorContextPtr shared_resources)
  : ServiceClientBehaviorBase<moveit2_extended_msgs::srv::PlanWithGpu>(name, config, std::move(shared_resources),
                                                                       kPlanService)
{
}

BT::PortsList PlanToJointGoalGPU::providedPorts()
{
  BT::PortsList ports = commonGoalPorts();
  // BT::InputPort already returns a {name, PortInfo} pair, so these go in directly.
  ports.insert(BT::InputPort<sensor_msgs::msg::JointState>("joint_state", "the target joints"));
  ports.insert(BT::InputPort<std::string>("named_state", "", "an SRDF group_state; wins over joint_state"));
  return providedBasicPorts(ports);
}

BtExpected<std::shared_ptr<PlanToJointGoalGPU::Request>> PlanToJointGoalGPU::createRequest()
{
  auto request = std::make_shared<Request>();
  request->goal_type = Request::JOINT_GOAL;
  request->planning_group = getInputOr<std::string>("planning_group", "");
  request->start_state = getInputOr<moveit_msgs::msg::RobotState>("start_state", {});
  request->max_velocity_scaling = getInputOr<double>("velocity_scaling_factor", 0.0);
  request->max_acceleration_scaling = getInputOr<double>("acceleration_scaling_factor", 0.0);

  const auto named = getInputOr<std::string>("named_state", "");
  const auto joints = getInputOr<sensor_msgs::msg::JointState>("joint_state", {});
  if (named.empty() && joints.name.empty())
  {
    return nonstd::make_unexpected("give either joint_state or named_state");
  }

  if (!named.empty())
  {
    // Resolved here rather than in the service: the SRDF is a MoveIt concept and cuRobo has never
    // heard of it. Doing it on this side also means the GPU and CPU Behaviors agree on what
    // named_state="home" means, which is the whole point of them being interchangeable.
    auto* context = getSharedResources().get();
    const auto model = context->robotModel();
    if (!model)
    {
      return nonstd::make_unexpected("named_state needs the robot model, which is not loaded");
    }
    const auto current = context->currentState();
    if (!current)
    {
      return nonstd::make_unexpected("named_state needs the current state, which is not available");
    }
    const auto group = request->planning_group.empty() ? context->config().default_planning_group :
                                                         request->planning_group;
    auto resolved = behaviors::resolveTargetState(model, *current, group, named, joints);
    if (!resolved)
    {
      return nonstd::make_unexpected(resolved.error());
    }
    const auto* joint_group = model->getJointModelGroup(group);
    std::vector<double> values;
    resolved->copyJointGroupPositions(joint_group, values);
    request->joint_goal.name = joint_group->getVariableNames();
    request->joint_goal.position = values;
  }
  else
  {
    request->joint_goal = joints;
  }
  return request;
}

BtStatus PlanToJointGoalGPU::processResponse(const std::shared_ptr<Response>& response)
{
  return reportPlan(response, *this, getLogger());
}

// ---------------------------------------------------------------------------------------------
// PlanToPoseGPU
// ---------------------------------------------------------------------------------------------

PlanToPoseGPU::PlanToPoseGPU(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources)
  : ServiceClientBehaviorBase<moveit2_extended_msgs::srv::PlanWithGpu>(name, config, std::move(shared_resources),
                                                                       kPlanService)
{
}

BT::PortsList PlanToPoseGPU::providedPorts()
{
  BT::PortsList ports = commonGoalPorts();
  ports.insert(BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "where to put target_link"));
  ports.insert(BT::InputPort<std::string>("target_link", "",
                                          "which link `pose` describes. Empty uses the service's "
                                          "tool frame"));
  return providedBasicPorts(ports);
}

BtExpected<std::shared_ptr<PlanToPoseGPU::Request>> PlanToPoseGPU::createRequest()
{
  const auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose");
  if (!pose)
  {
    return nonstd::make_unexpected("pose: " + pose.error());
  }

  auto request = std::make_shared<Request>();
  request->goal_type = Request::POSE_GOAL;
  request->planning_group = getInputOr<std::string>("planning_group", "");
  request->pose_goal = pose.value();
  request->target_link = getInputOr<std::string>("target_link", "");
  request->start_state = getInputOr<moveit_msgs::msg::RobotState>("start_state", {});
  request->max_velocity_scaling = getInputOr<double>("velocity_scaling_factor", 0.0);
  request->max_acceleration_scaling = getInputOr<double>("acceleration_scaling_factor", 0.0);
  return request;
}

BtStatus PlanToPoseGPU::processResponse(const std::shared_ptr<Response>& response)
{
  return reportPlan(response, *this, getLogger());
}

}  // namespace moveit2_extended::gpu

// ---------------------------------------------------------------------------------------------
// plugin
// ---------------------------------------------------------------------------------------------

#include <moveit2_extended_core/behavior_loader_base.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace moveit2_extended::gpu
{

/** Registers the GPU Behaviors.
 *
 *  Registration does NOT need cuRobo, a GPU, or the service to be running: these are service
 *  clients, so the tree builds either way and a missing service is a run-time FAILURE with a clear
 *  message rather than a server that will not start. That is deliberate -- the same Objective XML
 *  has to load on the workstation with the GPU and on the robot without one, with a Fallback
 *  dropping to the OMPL Behaviors. */
class GpuBehaviorsLoader : public BehaviorLoaderBase
{
public:
  void registerBehaviors(BtFactory& factory, const BehaviorContextPtr& shared_resources) override
  {
    registerBehavior<SolveIKQueriesGPU>(factory, "SolveIKQueriesGPU", shared_resources);
    registerBehavior<PlanToJointGoalGPU>(factory, "PlanToJointGoalGPU", shared_resources);
    registerBehavior<PlanToPoseGPU>(factory, "PlanToPoseGPU", shared_resources);
  }

  std::unordered_map<std::string, std::string> behaviorDescriptions() const override
  {
    return {
      { "SolveIKQueriesGPU",
        "Solve many IK queries in one GPU batch. Use when there are candidate poses to sift; one "
        "query is not worth a GPU." },
      { "PlanToJointGoalGPU", "Plan to a joint goal with cuRobo. Same ports as MoveToJointState." },
      { "PlanToPoseGPU", "Plan to a Cartesian goal with cuRobo. Same ports as MoveToPose." },
    };
  }
};

}  // namespace moveit2_extended::gpu

PLUGINLIB_EXPORT_CLASS(moveit2_extended::gpu::GpuBehaviorsLoader, moveit2_extended::BehaviorLoaderBase)
