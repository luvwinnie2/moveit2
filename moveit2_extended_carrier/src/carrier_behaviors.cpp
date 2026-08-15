// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_carrier/carrier_behaviors.hpp>

#include <moveit_cable_carrier/carrier_registry.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <algorithm>

namespace moveit2_extended::carrier
{
namespace msgs = moveit2_extended_msgs::msg;
using moveit_cable_carrier::CarrierRegistry;
using moveit_cable_carrier::CarrierTrajectoryAnalyzer;

// ---------------------------------------------------------------------------------------------
// AnalyzerCache
// ---------------------------------------------------------------------------------------------

AnalyzerCache& AnalyzerCache::instance()
{
  static AnalyzerCache cache;
  return cache;
}

std::vector<std::shared_ptr<CarrierTrajectoryAnalyzer>>
AnalyzerCache::get(const moveit::core::RobotModelConstPtr& model, const std::vector<std::string>& names)
{
  std::vector<std::shared_ptr<CarrierTrajectoryAnalyzer>> out;
  if (!model)
  {
    return out;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (model_ != model.get())
  {
    // A different robot model means every cached analyser is bound to the wrong one.
    analyzers_.clear();
    model_ = model.get();
  }

  for (const auto& params : CarrierRegistry::instance().carriers())
  {
    if (!names.empty() && std::find(names.begin(), names.end(), params.name) == names.end())
    {
      continue;
    }
    auto it = analyzers_.find(params.name);
    if (it == analyzers_.end())
    {
      auto analyzer = std::make_shared<CarrierTrajectoryAnalyzer>(params, model);
      if (!analyzer->valid())
      {
        // Bad base_link/tip_link: skip it rather than returning something inert that would report
        // "everything is fine" for a carrier that was never evaluated.
        continue;
      }
      it = analyzers_.emplace(params.name, std::move(analyzer)).first;
    }
    out.push_back(it->second);
  }
  return out;
}

void AnalyzerCache::invalidate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  analyzers_.clear();
}

// ---------------------------------------------------------------------------------------------
// conversions
// ---------------------------------------------------------------------------------------------

msgs::CarrierDiagnostics toMsg(const moveit_cable_carrier::CarrierWaypointResult& result,
                               const std::string& carrier_name)
{
  msgs::CarrierDiagnostics out;
  out.carrier_name = carrier_name;
  out.feasible = result.feasible;
  out.min_bend_radius = result.min_bend_radius;
  out.bend_utilisation = result.bend_utilisation;
  out.twist_utilisation = result.twist_utilisation;
  out.cable_bend_ratio = result.cable_bend_ratio;
  out.required_cable_bend_ratio = result.required_cable_bend_ratio;
  out.worst_cable = result.worst_cable;
  out.cable_strain = result.cable_strain;
  out.cables_within_limit = result.cablesWithinLimit();
  out.tension = result.tension;
  out.max_penetration = result.max_penetration;
  out.penetrating_obstacle = result.penetrating_obstacle;
  return out;
}

msgs::CarrierTrajectoryReport toMsg(const moveit_cable_carrier::CarrierTrajectoryReport& report)
{
  msgs::CarrierTrajectoryReport out;
  out.carrier_name = report.carrier_name;
  out.bend_radius = report.bend_radius;
  out.bend_utilisation_warn = report.bend_utilisation_warn;

  out.waypoints.reserve(report.waypoints.size());
  for (const auto& waypoint : report.waypoints)
  {
    msgs::CarrierWaypointResult row;
    row.index = static_cast<uint32_t>(waypoint.index);
    row.time_from_start = waypoint.time_from_start;
    row.collision = waypoint.collision;
    row.diagnostics = toMsg(waypoint, report.carrier_name);
    out.waypoints.push_back(std::move(row));
  }

  out.segments.reserve(report.segments.size());
  for (const auto& segment : report.segments)
  {
    msgs::CarrierSegmentFatigue entry;
    entry.segment = static_cast<uint32_t>(segment.segment);
    entry.min_curvature = segment.min_curvature;
    entry.max_curvature = segment.max_curvature;
    entry.reversals = static_cast<int32_t>(segment.reversals);
    out.segments.push_back(entry);
  }

  out.infeasible_count = static_cast<int32_t>(report.infeasible_count);
  out.colliding_count = static_cast<int32_t>(report.colliding_count);
  out.over_worked_count = static_cast<int32_t>(report.over_worked_count);
  out.cable_violation_count = static_cast<int32_t>(report.cable_violation_count);
  out.worst_bend_utilisation = report.worst_bend_utilisation;
  out.worst_twist_utilisation = report.worst_twist_utilisation;
  out.worst_cable_strain = report.worst_cable_strain;
  out.worst_cable_bend_ratio = report.worst_cable_bend_ratio;
  out.required_cable_bend_ratio = report.required_cable_bend_ratio;
  out.worst_cable = report.worst_cable;
  out.tightest_radius = report.tightest_radius;
  out.has_tightest_radius = report.has_tightest_radius;
  out.hottest_segment = static_cast<uint32_t>(report.hottest_segment);
  out.hottest_segment_min_curvature = report.hottest_segment_min_curvature;
  out.hottest_segment_max_curvature = report.hottest_segment_max_curvature;
  out.hottest_segment_reversals = static_cast<int32_t>(report.hottest_segment_reversals);
  out.has_hottest_segment = report.has_hottest_segment;
  out.first_bad_index = report.first_bad_index;
  out.safe = report.safe();
  out.verdict = report.verdict();
  return out;
}

// ---------------------------------------------------------------------------------------------
// ValidateCarrierAlongTrajectory
// ---------------------------------------------------------------------------------------------

ValidateCarrierAlongTrajectory::ValidateCarrierAlongTrajectory(const std::string& name, const NodeConfig& config,
                                                               BehaviorContextPtr shared_resources)
  : AsyncBehaviorBase(name, config, std::move(shared_resources))
{
}

BT::PortsList ValidateCarrierAlongTrajectory::providedPorts()
{
  return {
    BT::InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory", "the planned trajectory to judge"),
    BT::InputPort<moveit_msgs::msg::RobotState>("start_state",
                                                "the state the trajectory is relative to; empty uses the "
                                                "current one"),
    BT::InputPort<moveit_msgs::msg::PlanningScene>("planning_scene",
                                                   "judge against this scene instead of the live one"),
    BT::InputPort<std::vector<std::string>>("carrier_names", "which carriers; empty means all configured"),
    BT::InputPort<double>("sample_period", 0.05,
                          "seconds between sampled waypoints; 0 checks every waypoint"),
    BT::InputPort<double>("max_bend_utilisation", -1.0,
                          "over-worked threshold; negative uses the carrier's own configured warning level"),
    BT::InputPort<bool>("fail_on_over_worked", false,
                        "by default being over-worked is a warning, not a failure"),
    BT::InputPort<bool>("fail_on_cable_violation", true,
                        "a cable below its own minimum bend radius fails by default: on this robot the "
                        "cable, not the chain, is usually the binding limit"),
    BT::InputPort<bool>("check_collision", true, ""),
    BT::InputPort<bool>("stop_at_first_failure", true, "early-out; set false to get a full report"),
    BT::OutputPort<moveit2_extended_msgs::msg::CarrierTrajectoryReport>("report", "the worst carrier's report"),
    BT::OutputPort<double>("worst_bend_utilisation", ""),
    BT::OutputPort<double>("worst_twist_utilisation", ""),
    BT::OutputPort<int>("infeasible_count", ""),
    BT::OutputPort<int>("first_bad_index", "first failing waypoint; -1 when clean"),
    BT::OutputPort<std::string>("verdict", "the one-line human sentence"),
  };
}

bool ValidateCarrierAlongTrajectory::prepare()
{
  states_.clear();
  times_.clear();
  reports_.clear();
  scene_.reset();

  const auto model = getSharedResources()->robotModel();
  const auto psm = getSharedResources()->planningSceneMonitor();
  if (!model || !psm)
  {
    RCLCPP_ERROR(getLogger(), "MoveIt is not available; is move_group running?");
    return false;
  }

  const auto message = getInput<moveit_msgs::msg::RobotTrajectory>("trajectory");
  if (!message)
  {
    RCLCPP_ERROR(getLogger(), "%s", std::string("trajectory: " + message.error()).c_str());
    return false;
  }

  analyzers_ = AnalyzerCache::instance().get(
      model, getInputOr<std::vector<std::string>>("carrier_names", std::vector<std::string>{}));
  if (analyzers_.empty())
  {
    // Not "nothing to check": a tree that asked for a carrier check and got none silently would
    // conclude the motion is safe.
    RCLCPP_ERROR(getLogger(), "no cable carriers are configured; is MOVEIT_CABLE_CARRIER_CONFIG set?");
    return false;
  }

  // Clone the scene ONCE and let go of the lock. Holding LockedPlanningSceneRO for the whole
  // analysis would block every planning query on the robot for its duration.
  {
    planning_scene_monitor::LockedPlanningSceneRO locked(psm);
    const auto supplied = getInput<moveit_msgs::msg::PlanningScene>("planning_scene");
    scene_ = planning_scene::PlanningScene::clone(locked);
    if (supplied)
    {
      scene_->usePlanningSceneMsg(supplied.value());
    }
  }

  // Expand the trajectory into states here, on the tick thread, because it needs the robot model
  // and the current state -- both of which the worker must not touch.
  moveit::core::RobotState reference = scene_->getCurrentState();
  const auto start = getInputOr<moveit_msgs::msg::RobotState>("start_state", moveit_msgs::msg::RobotState{});
  if (!start.joint_state.name.empty())
  {
    moveit::core::robotStateMsgToRobotState(start, reference);
  }

  robot_trajectory::RobotTrajectory trajectory(model, "");
  try
  {
    moveit_msgs::msg::RobotState reference_msg;
    moveit::core::robotStateToRobotStateMsg(reference, reference_msg);
    trajectory.setRobotTrajectoryMsg(reference, reference_msg, message.value());
  }
  catch (const std::exception& exc)
  {
    RCLCPP_ERROR(getLogger(), "%s", std::string(std::string("cannot read the trajectory: ") + exc.what()).c_str());
    return false;
  }
  if (trajectory.empty())
  {
    RCLCPP_ERROR(getLogger(), "%s", std::string("the trajectory has no waypoints").c_str());
    return false;
  }

  const double sample_period = getInputOr<double>("sample_period", 0.05);
  double next_sample = -1.0;
  for (size_t i = 0; i < trajectory.getWayPointCount(); ++i)
  {
    const double t = trajectory.getWayPointDurationFromStart(i);
    const bool is_last = (i + 1 == trajectory.getWayPointCount());
    // The last waypoint is always kept: it is where the robot ends up, and skipping it because it
    // fell between samples would be the one omission nobody would forgive.
    if (sample_period > 0.0 && !is_last && t < next_sample)
    {
      continue;
    }
    next_sample = t + sample_period;
    states_.push_back(trajectory.getWayPoint(i));
    times_.push_back(t);
  }
  return true;
}

BtStatus ValidateCarrierAlongTrajectory::doWork()
{
  CarrierTrajectoryAnalyzer::Options options;
  options.bend_utilisation_warn = getInputOr<double>("max_bend_utilisation", -1.0);
  options.check_collision = getInputOr<bool>("check_collision", true);
  options.stop_at_first_failure = getInputOr<bool>("stop_at_first_failure", true);
  options.keep_waypoints = true;
  options.cancel = &cancelFlag();

  const bool fail_on_over_worked = getInputOr<bool>("fail_on_over_worked", false);
  const bool fail_on_cable = getInputOr<bool>("fail_on_cable_violation", true);

  bool ok = true;
  for (const auto& analyzer : analyzers_)
  {
    if (cancelRequested())
    {
      return BtStatus::FAILURE;
    }
    auto report = analyzer->analyze(states_, times_, scene_.get(), options);

    if (report.infeasible_count > 0 || report.colliding_count > 0)
    {
      ok = false;
    }
    if (fail_on_cable && report.cable_violation_count > 0)
    {
      ok = false;
    }
    if (fail_on_over_worked && report.over_worked_count > 0)
    {
      ok = false;
    }
    reports_.push_back(std::move(report));
  }
  return ok ? BtStatus::SUCCESS : BtStatus::FAILURE;
}

void ValidateCarrierAlongTrajectory::publishResults(BtStatus /*result*/)
{
  if (reports_.empty())
  {
    return;
  }

  // "Worst" is the one with the highest bend utilisation, which is the number a person watching
  // this actually cares about.
  const auto worst = std::max_element(reports_.begin(), reports_.end(),
                                      [](const auto& a, const auto& b) {
                                        return a.worst_bend_utilisation < b.worst_bend_utilisation;
                                      });

  setOutput("report", toMsg(*worst));
  setOutput("worst_bend_utilisation", worst->worst_bend_utilisation);
  setOutput("worst_twist_utilisation", worst->worst_twist_utilisation);
  setOutput("infeasible_count", static_cast<int>(worst->infeasible_count));
  setOutput("first_bad_index", worst->first_bad_index);
  setOutput("verdict", worst->verdict());

  for (const auto& report : reports_)
  {
    RCLCPP_INFO(getLogger(), "carrier '%s': %s", report.carrier_name.c_str(), report.verdict().c_str());
  }
}

// ---------------------------------------------------------------------------------------------
// IsCarrierFeasible
// ---------------------------------------------------------------------------------------------

IsCarrierFeasible::IsCarrierFeasible(const std::string& name, const NodeConfig& config,
                                     BehaviorContextPtr shared_resources)
  : ConditionBehaviorBase(name, config, std::move(shared_resources))
{
}

BT::PortsList IsCarrierFeasible::providedPorts()
{
  return {
    BT::InputPort<std::vector<std::string>>("carrier_names", "which carriers; empty means all configured"),
    BT::InputPort<double>("max_bend_utilisation", -1.0, "negative uses the carrier's own warning level"),
    BT::InputPort<double>("max_twist_utilisation", 1.0, ""),
    BT::InputPort<bool>("require_cables_within_limit", true, ""),
    BT::OutputPort<double>("bend_utilisation", ""),
    BT::OutputPort<double>("twist_utilisation", ""),
    BT::OutputPort<std::string>("reason", "why it failed, when it did"),
  };
}

BtStatus IsCarrierFeasible::tick()
{
  const auto model = getSharedResources()->robotModel();
  const auto state = getSharedResources()->currentState();
  if (!model || !state)
  {
    setOutput("reason", std::string("MoveIt is not available"));
    return BtStatus::FAILURE;
  }

  const auto analyzers = AnalyzerCache::instance().get(
      model, getInputOr<std::vector<std::string>>("carrier_names", std::vector<std::string>{}));
  if (analyzers.empty())
  {
    setOutput("reason", std::string("no cable carriers are configured"));
    return BtStatus::FAILURE;
  }

  const double max_twist = getInputOr<double>("max_twist_utilisation", 1.0);
  const bool require_cables = getInputOr<bool>("require_cables_within_limit", true);
  const double requested_bend = getInputOr<double>("max_bend_utilisation", -1.0);

  double worst_bend = 0.0;
  double worst_twist = 0.0;
  std::string reason;

  for (const auto& analyzer : analyzers)
  {
    // No collision check: a condition is ticked far more often than an action, and a scene query
    // per tick is not the cheap operation this needs to be.
    const auto result = analyzer->analyzeState(*state, nullptr);
    const double limit = requested_bend >= 0.0 ? requested_bend : analyzer->params().bend_utilisation_warn;

    worst_bend = std::max(worst_bend, result.bend_utilisation);
    worst_twist = std::max(worst_twist, result.twist_utilisation);

    if (!result.feasible)
    {
      reason = "carrier '" + analyzer->params().name + "' cannot take the required shape here";
      break;
    }
    if (result.bend_utilisation > limit)
    {
      reason = "carrier '" + analyzer->params().name + "' is at " +
               std::to_string(result.bend_utilisation) + " of its bend limit (max " + std::to_string(limit) + ")";
      break;
    }
    if (result.twist_utilisation > max_twist)
    {
      reason = "carrier '" + analyzer->params().name + "' twists past its torsion stop";
      break;
    }
    if (require_cables && !result.cablesWithinLimit())
    {
      reason = "cable '" + result.worst_cable + "' is below its minimum bend radius";
      break;
    }
  }

  setOutput("bend_utilisation", worst_bend);
  setOutput("twist_utilisation", worst_twist);
  setOutput("reason", reason);
  return reason.empty() ? BtStatus::SUCCESS : BtStatus::FAILURE;
}

// ---------------------------------------------------------------------------------------------
// GetCarrierDiagnostics
// ---------------------------------------------------------------------------------------------

GetCarrierDiagnostics::GetCarrierDiagnostics(const std::string& name, const NodeConfig& config,
                                             BehaviorContextPtr shared_resources)
  : SyncBehaviorBase(name, config, std::move(shared_resources))
{
}

BT::PortsList GetCarrierDiagnostics::providedPorts()
{
  return {
    BT::InputPort<std::vector<std::string>>("carrier_names", "which carriers; empty means all configured"),
    BT::OutputPort<moveit2_extended_msgs::msg::CarrierDiagnostics>("diagnostics", "the worst carrier"),
    BT::OutputPort<double>("bend_utilisation", ""),
    BT::OutputPort<double>("twist_utilisation", ""),
    BT::OutputPort<double>("min_bend_radius", "m"),
    BT::OutputPort<double>("cable_bend_ratio", ""),
    BT::OutputPort<double>("tension", "N"),
    BT::OutputPort<bool>("feasible", ""),
  };
}

BtStatus GetCarrierDiagnostics::tick()
{
  const auto model = getSharedResources()->robotModel();
  const auto state = getSharedResources()->currentState();
  if (!model || !state)
  {
    RCLCPP_ERROR(getLogger(), "MoveIt is not available; is move_group running?");
    return BtStatus::FAILURE;
  }

  const auto analyzers = AnalyzerCache::instance().get(
      model, getInputOr<std::vector<std::string>>("carrier_names", std::vector<std::string>{}));
  if (analyzers.empty())
  {
    RCLCPP_ERROR(getLogger(), "no cable carriers are configured");
    return BtStatus::FAILURE;
  }

  moveit_cable_carrier::CarrierWaypointResult worst;
  std::string worst_name;
  for (const auto& analyzer : analyzers)
  {
    const auto result = analyzer->analyzeState(*state, nullptr);
    if (worst_name.empty() || result.bend_utilisation > worst.bend_utilisation)
    {
      worst = result;
      worst_name = analyzer->params().name;
    }
  }

  setOutput("diagnostics", toMsg(worst, worst_name));
  setOutput("bend_utilisation", worst.bend_utilisation);
  setOutput("twist_utilisation", worst.twist_utilisation);
  setOutput("min_bend_radius", worst.min_bend_radius);
  setOutput("cable_bend_ratio", worst.cable_bend_ratio);
  setOutput("tension", worst.tension);
  setOutput("feasible", worst.feasible);
  return BtStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------
// SwitchCarrierType
// ---------------------------------------------------------------------------------------------

SwitchCarrierType::SwitchCarrierType(const std::string& name, const NodeConfig& config,
                                     BehaviorContextPtr shared_resources)
  : SyncBehaviorBase(name, config, std::move(shared_resources))
{
}

BT::PortsList SwitchCarrierType::providedPorts()
{
  return {
    BT::InputPort<std::string>("config_file", "a carrier YAML; package:// accepted"),
    BT::OutputPort<int>("carrier_count", "how many carriers are configured afterwards"),
  };
}

BtStatus SwitchCarrierType::tick()
{
  const auto path = getInputOr<std::string>("config_file", std::string(""));
  if (path.empty())
  {
    RCLCPP_ERROR(getLogger(), "config_file is required");
    return BtStatus::FAILURE;
  }

  std::string error;
  if (!CarrierRegistry::instance().loadFromYaml(path, &error))
  {
    RCLCPP_ERROR(getLogger(), "cannot load '%s': %s", path.c_str(), error.c_str());
    return BtStatus::FAILURE;
  }
  AnalyzerCache::instance().invalidate();

  const auto count = CarrierRegistry::instance().carriers().size();
  setOutput("carrier_count", static_cast<int>(count));

  // Said every time, because it is the kind of limitation that is easy to forget and expensive to
  // rediscover: CollisionEnvCarrier snapshots the registry when it is constructed, and its
  // diff-scene copy constructor keeps the parent's set on purpose.
  RCLCPP_WARN(getLogger(),
              "loaded %zu carrier(s) from '%s'. This changes what THIS process checks against. A "
              "running move_group keeps planning against the configuration it started with -- to "
              "change that, restart it with a different MOVEIT_CABLE_CARRIER_CONFIG.",
              count, path.c_str());
  return BtStatus::SUCCESS;
}

}  // namespace moveit2_extended::carrier
