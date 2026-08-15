// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

// Behaviors that ask whether the cable carrier (ザイルベア / dresspack) survives a motion.
//
// WHERE THIS SITS, because it is easy to assume it does more than it does:
//
// move_group is already launched with collision_detector: moveit_cable_carrier/CABLE_CARRIER, so
// planning ALREADY avoids configurations the carrier cannot take -- that happens inside the
// planner, with no Behavior involved. These Behaviors are the layer above: they put a NUMBER on
// how close a plan came to the limit, so an Objective can refuse a path that is merely survivable,
// or take a gentler branch, or tell the operator why it will not run.
//
// This is the same position MoveIt Pro's ValidateTrajectory occupies, and Pro has nothing like the
// underlying model -- a deformable carrier whose SHAPE, not just pose, is re-solved for every
// robot configuration.

#include <moveit2_extended_core/async_behavior_base.hpp>
#include <moveit2_extended_core/shared_resources_node.hpp>

#include <moveit2_extended_msgs/msg/carrier_diagnostics.hpp>
#include <moveit2_extended_msgs/msg/carrier_trajectory_report.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <moveit_cable_carrier/trajectory_analysis.hpp>

#include <memory>
#include <string>

namespace moveit2_extended::carrier
{

using moveit2_extended::AsyncBehaviorBase;
using moveit2_extended::BehaviorContextPtr;
using moveit2_extended::BtStatus;
using moveit2_extended::ConditionBehaviorBase;
using moveit2_extended::NodeConfig;
using moveit2_extended::SyncBehaviorBase;

/** Analysers are expensive to build and NOT thread safe -- the rod solver keeps a warm start
 *  between calls -- so they are cached per carrier and handed out under a lock. The generation
 *  counter lets SwitchCarrierType invalidate the cache without anyone holding a stale analyser. */
class AnalyzerCache
{
public:
  static AnalyzerCache& instance();

  /** Analysers for the named carriers, or for all of them when `names` is empty. Empty result
   *  means the registry has nothing configured, which is a configuration error worth reporting
   *  rather than treating as "nothing to check". */
  std::vector<std::shared_ptr<moveit_cable_carrier::CarrierTrajectoryAnalyzer>>
  get(const moveit::core::RobotModelConstPtr& model, const std::vector<std::string>& names);

  /** Drop everything, so the next get() rebuilds from the current registry. */
  void invalidate();

private:
  std::mutex mutex_;
  std::map<std::string, std::shared_ptr<moveit_cable_carrier::CarrierTrajectoryAnalyzer>> analyzers_;
  const moveit::core::RobotModel* model_ = nullptr;
};

/** The headline Behavior: run a planned trajectory past the carrier and say how hard it works it.
 *
 *  Follows MoveIt Pro's ValidateTrajectory in taking the planning scene as an INPUT rather than
 *  querying the live one, so a tree can validate against a hypothetical scene. Unwired, it clones
 *  the current scene once at the start and works from that -- holding the scene lock for the whole
 *  analysis would stall every planning query on the robot. */
class ValidateCarrierAlongTrajectory : public AsyncBehaviorBase
{
public:
  ValidateCarrierAlongTrajectory(const std::string& name, const NodeConfig& config,
                                 BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

protected:
  /** Reads the ports, clones the scene and expands the trajectory into states -- all on the tick
   *  thread, because doWork() must touch none of those. */
  bool prepare() override;
  BtStatus doWork() override;
  void publishResults(BtStatus result) override;

private:
  // Filled by onStart (tick thread), read by doWork (worker thread), written back by
  // publishResults (tick thread). Nothing else crosses.
  std::vector<moveit::core::RobotState> states_;
  std::vector<double> times_;
  planning_scene::PlanningScenePtr scene_;
  std::vector<std::shared_ptr<moveit_cable_carrier::CarrierTrajectoryAnalyzer>> analyzers_;
  std::vector<moveit_cable_carrier::CarrierTrajectoryReport> reports_;
};

/** Cheap yes/no for the current pose, or for a supplied one. A condition must stay cheap, and one
 *  rod solve is tens of microseconds, so this is synchronous. */
class IsCarrierFeasible : public ConditionBehaviorBase
{
public:
  IsCarrierFeasible(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

private:
  BtStatus tick() override;
};

/** The numbers for one pose, onto the blackboard, for an Objective that wants to log or branch on
 *  them without a full trajectory analysis. */
class GetCarrierDiagnostics : public SyncBehaviorBase
{
public:
  GetCarrierDiagnostics(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

private:
  BtStatus tick() override;
};

/** Load a different carrier configuration at run time -- a different dresspack for a different
 *  tool, say.
 *
 *  HONEST LIMITATION, logged on every call: this changes what THIS process checks against. It does
 *  NOT change what a running move_group plans against, because CollisionEnvCarrier snapshots the
 *  registry when it is constructed and its diff-scene copy constructor deliberately keeps the
 *  parent's set. To change planning, restart move_group with a different
 *  MOVEIT_CABLE_CARRIER_CONFIG. */
class SwitchCarrierType : public SyncBehaviorBase
{
public:
  SwitchCarrierType(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

private:
  BtStatus tick() override;
};

/** Convert an analyser report into the message form. */
moveit2_extended_msgs::msg::CarrierTrajectoryReport
toMsg(const moveit_cable_carrier::CarrierTrajectoryReport& report);
moveit2_extended_msgs::msg::CarrierDiagnostics toMsg(const moveit_cable_carrier::CarrierWaypointResult& result,
                                                     const std::string& carrier_name);

}  // namespace moveit2_extended::carrier
