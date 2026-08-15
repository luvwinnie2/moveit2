// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_behaviors/motion_behaviors.hpp>

#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>

namespace moveit2_extended::behaviors
{
namespace
{
/** Turn a MoveItErrorCodes value into something a person can act on. */
std::string errorName(int32_t code)
{
  using moveit_msgs::msg::MoveItErrorCodes;
  switch (code)
  {
    case MoveItErrorCodes::SUCCESS:
      return "SUCCESS";
    case MoveItErrorCodes::PLANNING_FAILED:
      return "PLANNING_FAILED";
    case MoveItErrorCodes::INVALID_MOTION_PLAN:
      return "INVALID_MOTION_PLAN";
    case MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case MoveItErrorCodes::CONTROL_FAILED:
      return "CONTROL_FAILED";
    case MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA:
      return "UNABLE_TO_AQUIRE_SENSOR_DATA";
    case MoveItErrorCodes::TIMED_OUT:
      return "TIMED_OUT";
    case MoveItErrorCodes::PREEMPTED:
      return "PREEMPTED";
    case MoveItErrorCodes::START_STATE_IN_COLLISION:
      return "START_STATE_IN_COLLISION";
    case MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
      return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case MoveItErrorCodes::GOAL_IN_COLLISION:
      return "GOAL_IN_COLLISION";
    case MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
      return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
      return "GOAL_CONSTRAINTS_VIOLATED";
    case MoveItErrorCodes::INVALID_GROUP_NAME:
      return "INVALID_GROUP_NAME";
    case MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
      return "INVALID_GOAL_CONSTRAINTS";
    case MoveItErrorCodes::INVALID_ROBOT_STATE:
      return "INVALID_ROBOT_STATE";
    case MoveItErrorCodes::INVALID_LINK_NAME:
      return "INVALID_LINK_NAME";
    case MoveItErrorCodes::INVALID_OBJECT_NAME:
      return "INVALID_OBJECT_NAME";
    case MoveItErrorCodes::NO_IK_SOLUTION:
      return "NO_IK_SOLUTION";
    default:
      return "error " + std::to_string(code);
  }
}

/** True when a RobotState message carries nothing, i.e. "plan from the current state". */
bool isEmptyState(const moveit_msgs::msg::RobotState& state)
{
  return state.joint_state.name.empty() && state.joint_state.position.empty() &&
         state.multi_dof_joint_state.joint_names.empty();
}
}  // namespace

BT::PortsList commonPlanningPorts()
{
  return {
    BT::InputPort<std::string>("planning_group", "arm", "the JointModelGroup to plan for"),
    BT::InputPort<moveit_msgs::msg::RobotState>("start_state",
                                                "plan from here instead of the current state; wire a previous "
                                                "stage's end_state to plan a whole sequence before moving"),
    BT::InputPort<double>("velocity_scaling_factor", 0.1, "fraction of the joint velocity limits"),
    BT::InputPort<double>("acceleration_scaling_factor", 0.1, "fraction of the joint acceleration limits"),
    BT::InputPort<std::string>("planner_id", "", "e.g. RRTConnectkConfigDefault; empty uses the pipeline default"),
    BT::InputPort<std::string>("pipeline_id", "ompl", "planning pipeline"),
    BT::InputPort<int>("num_planning_attempts", 5, ""),
    BT::InputPort<double>("allowed_planning_time", 5.0, "seconds"),
    BT::InputPort<bool>("plan_only", true,
                        "plan without executing. Default true so a carrier or approval check can sit between "
                        "planning and moving -- which is the whole point of this stack"),
    BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("trajectory", "the planned trajectory"),
    BT::OutputPort<moveit_msgs::msg::RobotState>("end_state", "the trajectory's last waypoint"),
    BT::OutputPort<int>("error_code", "MoveItErrorCodes value"),
    BT::OutputPort<double>("planning_time", "seconds the planner took"),
  };
}

// ---------------------------------------------------------------------------------------------
// MoveGroupBehaviorBase
// ---------------------------------------------------------------------------------------------

MoveGroupBehaviorBase::MoveGroupBehaviorBase(const std::string& name, const NodeConfig& config,
                                             BehaviorContextPtr shared_resources)
  : ActionClientBehaviorBase<moveit_msgs::action::MoveGroup>(name, config, std::move(shared_resources), "/move_action")
{
}

BtExpected<MoveGroupBehaviorBase::Goal>
MoveGroupBehaviorBase::assembleGoal(const moveit_msgs::msg::Constraints& constraints)
{
  Goal goal;
  auto& request = goal.request;
  request.group_name = getInputOr<std::string>("planning_group", std::string("arm"));
  request.pipeline_id = getInputOr<std::string>("pipeline_id", std::string("ompl"));
  request.planner_id = getInputOr<std::string>("planner_id", std::string(""));
  request.num_planning_attempts = getInputOr<int>("num_planning_attempts", 5);
  request.allowed_planning_time = getInputOr<double>("allowed_planning_time", 5.0);
  request.max_velocity_scaling_factor = getInputOr<double>("velocity_scaling_factor", 0.1);
  request.max_acceleration_scaling_factor = getInputOr<double>("acceleration_scaling_factor", 0.1);
  request.goal_constraints = { constraints };

  const auto start = getInputOr<moveit_msgs::msg::RobotState>("start_state", moveit_msgs::msg::RobotState{});
  if (isEmptyState(start))
  {
    request.start_state.is_diff = true;  // plan from wherever the robot is
  }
  else
  {
    request.start_state = start;
    request.start_state.is_diff = false;
  }

  goal.planning_options.plan_only = getInputOr<bool>("plan_only", true);
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
  return goal;
}

BtStatus MoveGroupBehaviorBase::processResult(const WrappedResult& result)
{
  setOutput("error_code", static_cast<int>(result.result->error_code.val));
  setOutput("planning_time", result.result->planning_time);
  setOutput("trajectory", result.result->planned_trajectory);

  // end_state is what a later stage wires into its start_state to plan ahead without executing.
  const auto model = getSharedResources()->robotModel();
  const auto current = getSharedResources()->currentState();
  if (model && current)
  {
    const auto end_state = finalStateOf(model, *current, result.result->planned_trajectory);
    if (end_state)
    {
      setOutput("end_state", *end_state);
    }
  }

  const int32_t code = result.result->error_code.val;
  if (code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR(getLogger(), "planning failed: %s", errorName(code).c_str());
    return BtStatus::FAILURE;
  }
  return BtStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------
// MoveToJointState
// ---------------------------------------------------------------------------------------------

MoveToJointState::MoveToJointState(const std::string& name, const NodeConfig& config,
                                   BehaviorContextPtr shared_resources)
  : MoveGroupBehaviorBase(name, config, std::move(shared_resources))
{
}

BT::PortsList MoveToJointState::providedPorts()
{
  BT::PortsList ports = commonPlanningPorts();
  ports.insert({
      BT::InputPort<sensor_msgs::msg::JointState>("joint_state", "explicit joint target"),
      BT::InputPort<std::string>("named_state", "", "an SRDF group_state name, e.g. home; wins over joint_state"),
      BT::InputPort<double>("joint_tolerance", 0.001, "radians"),
  });
  return providedBasicPorts(ports);
}

BtExpected<MoveToJointState::Goal> MoveToJointState::createGoal()
{
  const auto model = getSharedResources()->robotModel();
  if (!model)
  {
    return nonstd::make_unexpected("MoveIt is not available; is move_group running?");
  }
  const auto current = getSharedResources()->currentState();
  if (!current)
  {
    return nonstd::make_unexpected("cannot read the current robot state");
  }

  const auto group = getInputOr<std::string>("planning_group", std::string("arm"));
  const auto named_state = getInputOr<std::string>("named_state", std::string(""));
  const auto joint_state = getInputOr<sensor_msgs::msg::JointState>("joint_state", sensor_msgs::msg::JointState{});

  const auto target = resolveTargetState(model, *current, group, named_state, joint_state);
  if (!target)
  {
    return nonstd::make_unexpected(target.error());
  }

  const auto constraints = makeJointGoal(*target, group, getInputOr<double>("joint_tolerance", 0.001));
  if (!constraints)
  {
    return nonstd::make_unexpected(constraints.error());
  }
  return assembleGoal(*constraints);
}

// ---------------------------------------------------------------------------------------------
// MoveToPose
// ---------------------------------------------------------------------------------------------

MoveToPose::MoveToPose(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources)
  : MoveGroupBehaviorBase(name, config, std::move(shared_resources))
{
}

BT::PortsList MoveToPose::providedPorts()
{
  BT::PortsList ports = commonPlanningPorts();
  ports.insert({
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose", "where the target_link should end up"),
      BT::InputPort<std::string>("target_link", "", "the link the pose describes; empty uses the group tip"),
      BT::InputPort<std::string>("ik_link", "",
                                 "the link the solver can constrain. On this robot crx_kinematics always "
                                 "solves to 'flange' regardless of what it is asked, so leaving this wrong "
                                 "moves the tool by the tool offset with no error reported"),
      BT::InputPort<double>("position_tolerance", 0.001, "metres"),
      BT::InputPort<double>("orientation_tolerance", 0.01, "radians"),
  });
  return providedBasicPorts(ports);
}

BtExpected<MoveToPose::Goal> MoveToPose::createGoal()
{
  const auto model = getSharedResources()->robotModel();
  if (!model)
  {
    return nonstd::make_unexpected("MoveIt is not available; is move_group running?");
  }

  const auto pose = getInput<geometry_msgs::msg::PoseStamped>("target_pose");
  if (!pose)
  {
    return nonstd::make_unexpected("target_pose: " + pose.error());
  }

  const auto group = getInputOr<std::string>("planning_group", std::string("arm"));
  const moveit::core::JointModelGroup* jmg = model->getJointModelGroup(group);
  if (!jmg)
  {
    return nonstd::make_unexpected("no planning group named '" + group + "'");
  }

  std::string target_link = getInputOr<std::string>("target_link", std::string(""));
  if (target_link.empty())
  {
    target_link = jmg->getLinkModelNames().empty() ? "" : jmg->getLinkModelNames().back();
  }
  // Defaults to the group tip, which is right for a robot whose solver honours ik_link_name. On
  // this one it does not, and the Objective is expected to say so explicitly.
  const std::string ik_link = getInputOr<std::string>("ik_link", target_link);

  const auto retargeted = retargetPose(model, pose.value(), target_link, ik_link);
  if (!retargeted)
  {
    return nonstd::make_unexpected(retargeted.error());
  }

  const auto constraints = makePoseGoal(*retargeted, ik_link, getInputOr<double>("position_tolerance", 0.001),
                                        getInputOr<double>("orientation_tolerance", 0.01));
  return assembleGoal(constraints);
}

// ---------------------------------------------------------------------------------------------
// ExecuteTrajectory
// ---------------------------------------------------------------------------------------------

ExecuteTrajectory::ExecuteTrajectory(const std::string& name, const NodeConfig& config,
                                     BehaviorContextPtr shared_resources)
  : ActionClientBehaviorBase<moveit_msgs::action::ExecuteTrajectory>(name, config, std::move(shared_resources),
                                                                     "/execute_trajectory")
{
}

BT::PortsList ExecuteTrajectory::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory", "what to run"),
      BT::OutputPort<int>("error_code", "MoveItErrorCodes value"),
  });
}

std::optional<BtStatus> ExecuteTrajectory::shortCircuit()
{
  const auto trajectory = getInput<moveit_msgs::msg::RobotTrajectory>("trajectory");
  if (!trajectory)
  {
    return std::nullopt;  // createGoal() reports the port error properly.
  }

  // A single point is what the planner returns when the arm is already standing on the target --
  // there is no motion to run and no time to parameterise. Refusing it (which the t=0 check below
  // used to do) makes an Objective fail the second time it is run, having succeeded the first,
  // which is about as confusing as a failure can be.
  //
  // An EMPTY trajectory is a different thing entirely -- it means nothing was planned -- so it is
  // deliberately left to fall through to createGoal(), which fails it.
  const size_t joint_points = trajectory->joint_trajectory.points.size();
  const size_t multi_dof_points = trajectory->multi_dof_joint_trajectory.points.size();
  if (std::max(joint_points, multi_dof_points) == 1)
  {
    RCLCPP_INFO(getLogger(), "already at the target: nothing to execute");
    return BtStatus::SUCCESS;
  }
  return std::nullopt;
}

BtExpected<ExecuteTrajectory::Goal> ExecuteTrajectory::createGoal()
{
  const auto trajectory = getInput<moveit_msgs::msg::RobotTrajectory>("trajectory");
  if (!trajectory)
  {
    return nonstd::make_unexpected("trajectory: " + trajectory.error());
  }
  if (trajectory->joint_trajectory.points.empty())
  {
    return nonstd::make_unexpected("refusing to execute a trajectory with no waypoints");
  }

  // A trajectory straight out of /compute_cartesian_path has no timing. Running it would either be
  // refused by the controller or executed as fast as the hardware allows.
  const auto& last = trajectory->joint_trajectory.points.back();
  if (last.time_from_start.sec == 0 && last.time_from_start.nanosec == 0)
  {
    return nonstd::make_unexpected("this trajectory is not time-parameterised (its last point is at t=0). "
                                   "Put a RetimeTrajectory between the planner and here.");
  }

  Goal goal;
  goal.trajectory = trajectory.value();
  return goal;
}

BtStatus ExecuteTrajectory::processResult(const WrappedResult& result)
{
  setOutput("error_code", static_cast<int>(result.result->error_code.val));
  const int32_t code = result.result->error_code.val;
  if (code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR(getLogger(), "execution failed: %s", errorName(code).c_str());
    return BtStatus::FAILURE;
  }
  return BtStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------
// PlanCartesianPath
// ---------------------------------------------------------------------------------------------

PlanCartesianPath::PlanCartesianPath(const std::string& name, const NodeConfig& config,
                                     BehaviorContextPtr shared_resources)
  : ServiceClientBehaviorBase<moveit_msgs::srv::GetCartesianPath>(name, config, std::move(shared_resources),
                                                                  "/compute_cartesian_path")
{
}

BT::PortsList PlanCartesianPath::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<std::string>("planning_group", "arm", ""),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints", "poses for target_link, in order"),
      BT::InputPort<std::string>("target_link", "", "the link the waypoints describe; empty uses the group tip"),
      BT::InputPort<std::string>("ik_link", "", "the link the solver constrains; see MoveToPose"),
      BT::InputPort<moveit_msgs::msg::RobotState>("start_state", "plan from here instead of the current state"),
      BT::InputPort<double>("max_step", 0.005, "Cartesian interpolation resolution, metres"),
      BT::InputPort<double>("jump_threshold", 0.0, "0 disables the joint-space jump check"),
      BT::InputPort<bool>("avoid_collisions", true, ""),
      BT::InputPort<double>("min_fraction", 0.99, "fail when less of the path than this was solved"),
      BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("trajectory", "UNTIMED; follow with RetimeTrajectory"),
      BT::OutputPort<double>("fraction", "how much of the path was solved, 0..1"),
  });
}

BtExpected<std::shared_ptr<PlanCartesianPath::Request>> PlanCartesianPath::createRequest()
{
  const auto model = getSharedResources()->robotModel();
  if (!model)
  {
    return nonstd::make_unexpected("MoveIt is not available; is move_group running?");
  }

  const auto waypoints = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints");
  if (!waypoints)
  {
    return nonstd::make_unexpected("waypoints: " + waypoints.error());
  }
  if (waypoints->empty())
  {
    return nonstd::make_unexpected("waypoints is empty");
  }

  const auto group = getInputOr<std::string>("planning_group", std::string("arm"));
  const moveit::core::JointModelGroup* jmg = model->getJointModelGroup(group);
  if (!jmg)
  {
    return nonstd::make_unexpected("no planning group named '" + group + "'");
  }

  std::string target_link = getInputOr<std::string>("target_link", std::string(""));
  if (target_link.empty())
  {
    target_link = jmg->getLinkModelNames().empty() ? "" : jmg->getLinkModelNames().back();
  }
  const std::string ik_link = getInputOr<std::string>("ik_link", target_link);

  auto request = std::make_shared<Request>();
  request->group_name = group;
  request->link_name = ik_link;
  request->max_step = getInputOr<double>("max_step", 0.005);
  request->jump_threshold = getInputOr<double>("jump_threshold", 0.0);
  request->avoid_collisions = getInputOr<bool>("avoid_collisions", true);
  request->header.frame_id = waypoints->front().header.frame_id;

  for (const auto& waypoint : *waypoints)
  {
    const auto retargeted = retargetPose(model, waypoint, target_link, ik_link);
    if (!retargeted)
    {
      return nonstd::make_unexpected(retargeted.error());
    }
    if (retargeted->header.frame_id != request->header.frame_id)
    {
      // /compute_cartesian_path takes one header for all waypoints, so mixed frames would be
      // silently reinterpreted in the first one.
      return nonstd::make_unexpected("all waypoints must be in the same frame; got '" +
                                     retargeted->header.frame_id + "' after '" + request->header.frame_id + "'");
    }
    request->waypoints.push_back(retargeted->pose);
  }

  const auto start = getInputOr<moveit_msgs::msg::RobotState>("start_state", moveit_msgs::msg::RobotState{});
  if (!isEmptyState(start))
  {
    request->start_state = start;
  }
  else
  {
    request->start_state.is_diff = true;
  }
  return request;
}

BtStatus PlanCartesianPath::processResponse(const std::shared_ptr<Response>& response)
{
  setOutput("fraction", response->fraction);
  setOutput("trajectory", response->solution);

  const double minimum = getInputOr<double>("min_fraction", 0.99);
  if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR(getLogger(), "Cartesian planning failed: %s", errorName(response->error_code.val).c_str());
    return BtStatus::FAILURE;
  }
  if (response->fraction < minimum)
  {
    RCLCPP_ERROR(getLogger(), "only %.1f%% of the Cartesian path was solved, needed %.1f%%",
                 100.0 * response->fraction, 100.0 * minimum);
    return BtStatus::FAILURE;
  }
  return BtStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------
// RetimeTrajectory
// ---------------------------------------------------------------------------------------------

RetimeTrajectory::RetimeTrajectory(const std::string& name, const NodeConfig& config,
                                   BehaviorContextPtr shared_resources)
  : SyncBehaviorBase(name, config, std::move(shared_resources))
{
}

BT::PortsList RetimeTrajectory::providedPorts()
{
  return {
    BT::InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory", "the untimed trajectory"),
    BT::InputPort<std::string>("planning_group", "arm", ""),
    BT::InputPort<double>("velocity_scaling_factor", 0.1, ""),
    BT::InputPort<double>("acceleration_scaling_factor", 0.1, ""),
    BT::InputPort<std::string>("algorithm", "totg", "totg | iterative"),
    BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("retimed_trajectory", "the time-parameterised trajectory"),
    BT::OutputPort<double>("duration", "seconds"),
  };
}

BtStatus RetimeTrajectory::tick()
{
  const auto model = getSharedResources()->robotModel();
  const auto current = getSharedResources()->currentState();
  if (!model || !current)
  {
    RCLCPP_ERROR(getLogger(), "MoveIt is not available; is move_group running?");
    return BtStatus::FAILURE;
  }

  const auto message = getInput<moveit_msgs::msg::RobotTrajectory>("trajectory");
  if (!message)
  {
    RCLCPP_ERROR(getLogger(), "trajectory: %s", message.error().c_str());
    return BtStatus::FAILURE;
  }

  const auto group = getInputOr<std::string>("planning_group", std::string("arm"));
  robot_trajectory::RobotTrajectory trajectory(model, group);
  try
  {
    moveit_msgs::msg::RobotState reference;
    moveit::core::robotStateToRobotStateMsg(*current, reference);
    trajectory.setRobotTrajectoryMsg(*current, reference, message.value());
  }
  catch (const std::exception& exc)
  {
    RCLCPP_ERROR(getLogger(), "cannot read the trajectory: %s", exc.what());
    return BtStatus::FAILURE;
  }
  if (trajectory.empty())
  {
    RCLCPP_ERROR(getLogger(), "the trajectory has no waypoints");
    return BtStatus::FAILURE;
  }

  const double velocity = getInputOr<double>("velocity_scaling_factor", 0.1);
  const double acceleration = getInputOr<double>("acceleration_scaling_factor", 0.1);
  const auto algorithm = getInputOr<std::string>("algorithm", std::string("totg"));

  bool ok = false;
  if (algorithm == "iterative")
  {
    trajectory_processing::IterativeParabolicTimeParameterization timing;
    ok = timing.computeTimeStamps(trajectory, velocity, acceleration);
  }
  else
  {
    trajectory_processing::TimeOptimalTrajectoryGeneration timing;
    ok = timing.computeTimeStamps(trajectory, velocity, acceleration);
  }
  if (!ok)
  {
    RCLCPP_ERROR(getLogger(), "time parameterisation failed using '%s'", algorithm.c_str());
    return BtStatus::FAILURE;
  }

  moveit_msgs::msg::RobotTrajectory out;
  trajectory.getRobotTrajectoryMsg(out);
  setOutput("retimed_trajectory", out);
  setOutput("duration", trajectory.getDuration());
  return BtStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------
// ComputeInverseKinematics
// ---------------------------------------------------------------------------------------------

ComputeInverseKinematics::ComputeInverseKinematics(const std::string& name, const NodeConfig& config,
                                                   BehaviorContextPtr shared_resources)
  : ServiceClientBehaviorBase<moveit_msgs::srv::GetPositionIK>(name, config, std::move(shared_resources),
                                                               "/compute_ik")
{
}

BT::PortsList ComputeInverseKinematics::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose", "where target_link should end up"),
      BT::InputPort<std::string>("planning_group", "arm", ""),
      BT::InputPort<std::string>("target_link", "", "empty uses the group tip"),
      BT::InputPort<std::string>("ik_link", "", "the link the solver constrains; see MoveToPose"),
      BT::InputPort<double>("timeout", 0.05, "seconds the solver may spend"),
      BT::InputPort<bool>("avoid_collisions", true, ""),
      BT::OutputPort<moveit_msgs::msg::RobotState>("solution", "the joint state that reaches the pose"),
      BT::OutputPort<int>("error_code", "MoveItErrorCodes value"),
  });
}

BtExpected<std::shared_ptr<ComputeInverseKinematics::Request>> ComputeInverseKinematics::createRequest()
{
  const auto model = getSharedResources()->robotModel();
  const auto current = getSharedResources()->currentState();
  if (!model || !current)
  {
    return nonstd::make_unexpected("MoveIt is not available; is move_group running?");
  }

  const auto pose = getInput<geometry_msgs::msg::PoseStamped>("target_pose");
  if (!pose)
  {
    return nonstd::make_unexpected("target_pose: " + pose.error());
  }

  const auto group = getInputOr<std::string>("planning_group", std::string("arm"));
  const moveit::core::JointModelGroup* jmg = model->getJointModelGroup(group);
  if (!jmg)
  {
    return nonstd::make_unexpected("no planning group named '" + group + "'");
  }

  std::string target_link = getInputOr<std::string>("target_link", std::string(""));
  if (target_link.empty())
  {
    target_link = jmg->getLinkModelNames().empty() ? "" : jmg->getLinkModelNames().back();
  }
  const std::string ik_link = getInputOr<std::string>("ik_link", target_link);

  const auto retargeted = retargetPose(model, pose.value(), target_link, ik_link);
  if (!retargeted)
  {
    return nonstd::make_unexpected(retargeted.error());
  }

  auto request = std::make_shared<Request>();
  request->ik_request.group_name = group;
  request->ik_request.ik_link_name = ik_link;
  request->ik_request.pose_stamped = *retargeted;
  request->ik_request.avoid_collisions = getInputOr<bool>("avoid_collisions", true);
  request->ik_request.timeout = rclcpp::Duration::from_seconds(getInputOr<double>("timeout", 0.05));
  moveit::core::robotStateToRobotStateMsg(*current, request->ik_request.robot_state);
  return request;
}

BtStatus ComputeInverseKinematics::processResponse(const std::shared_ptr<Response>& response)
{
  setOutput("error_code", static_cast<int>(response->error_code.val));
  if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_WARN(getLogger(), "no IK solution: %s", errorName(response->error_code.val).c_str());
    return BtStatus::FAILURE;
  }
  setOutput("solution", response->solution);
  return BtStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------
// StopMotion
// ---------------------------------------------------------------------------------------------

StopMotion::StopMotion(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources)
  : SendMessageToTopicBehaviorBase<std_msgs::msg::String>(name, config, std::move(shared_resources),
                                                          "/trajectory_execution_event")
{
}

BT::PortsList StopMotion::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<std::string>("command", "stop", "the execution event to publish"),
  });
}

BtExpected<std_msgs::msg::String> StopMotion::createMessage()
{
  std_msgs::msg::String message;
  message.data = getInputOr<std::string>("command", std::string("stop"));
  return message;
}

}  // namespace moveit2_extended::behaviors
