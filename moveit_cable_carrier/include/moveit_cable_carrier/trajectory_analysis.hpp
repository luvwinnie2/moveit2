// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit_cable_carrier/carrier_attacher.hpp>
#include <moveit_cable_carrier/carrier_params.hpp>
#include <moveit_cable_carrier/rod_solver.hpp>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <atomic>
#include <string>
#include <vector>

namespace moveit_cable_carrier
{

/** What the carrier does at one waypoint of a trajectory. */
struct CarrierWaypointResult
{
  size_t index = 0;
  double time_from_start = 0.0;  ///< [s]; 0 when the caller supplies no timing

  bool feasible = false;   ///< the two brackets can actually be joined by this carrier here
  bool collision = false;  ///< the planning scene reports a collision at this state

  double min_bend_radius = 0.0;   ///< [m]
  double bend_utilisation = 0.0;  ///< max_curvature * bend_radius; 1.0 = at the hardware stop
  double twist_utilisation = 0.0;
  double cable_bend_ratio = 0.0;           ///< achieved radius / worst cable OD
  double required_cable_bend_ratio = 0.0;  ///< what that cable's own spec demands
  double cable_strain = 0.0;
  double tension = 0.0;          ///< [N]
  double max_penetration = 0.0;  ///< [m], centreline vs link solid

  std::string worst_cable;
  std::string penetrating_obstacle;

  /** Mirrors CarrierShape::cablesWithinLimit() so a caller need not re-derive the rule. */
  bool cablesWithinLimit() const
  {
    return required_cable_bend_ratio <= 0.0 || cable_bend_ratio >= required_cable_bend_ratio;
  }
};

/** How hard one link of the chain is worked over the whole run.
 *
 *  A drag chain fails where it is bent *repeatedly*, not where it is bent once, so the curvature
 *  swing and the reversal count matter more than any single-pose number. */
struct CarrierSegmentFatigue
{
  size_t segment = 0;
  double min_curvature = 0.0;  ///< [1/m]
  double max_curvature = 0.0;  ///< [1/m]
  long reversals = 0;          ///< bending-direction reversals: a cheap stand-in for a rainflow count
};

/** Whole-trajectory verdict for one carrier. */
struct CarrierTrajectoryReport
{
  std::string carrier_name;
  double bend_radius = 0.0;             ///< [m], the hardware limit this was judged against
  double bend_utilisation_warn = 0.0;   ///< the threshold used for "over-worked"

  std::vector<CarrierWaypointResult> waypoints;
  std::vector<CarrierSegmentFatigue> segments;

  long infeasible_count = 0;
  long colliding_count = 0;
  long over_worked_count = 0;
  long cable_violation_count = 0;

  double worst_bend_utilisation = 0.0;
  double worst_twist_utilisation = 0.0;
  double worst_cable_strain = 0.0;
  double worst_cable_bend_ratio = 0.0;      ///< 0 when no cable was ever evaluated
  double required_cable_bend_ratio = 0.0;   ///< 0 when no inner cables are declared
  double tightest_radius = 0.0;  ///< [m]
  /** False when no waypoint was feasible. Needed as a separate flag because a perfectly straight
   *  run legitimately reports a tightest radius of 0, so 0 cannot double as "never set". */
  bool has_tightest_radius = false;
  std::string worst_cable;

  size_t hottest_segment = 0;
  double hottest_segment_min_curvature = 0.0;
  double hottest_segment_max_curvature = 0.0;
  long hottest_segment_reversals = 0;
  bool has_hottest_segment = false;

  /** First waypoint that failed outright (infeasible, colliding or over its cable limit).
   *  -1 when the run is clean. */
  int first_bad_index = -1;

  /** True when nothing infeasible, colliding or cable-violating occurred. Being "over-worked" is
   *  deliberately NOT a failure here -- it is a warning, and the caller decides. */
  bool safe() const { return infeasible_count == 0 && colliding_count == 0 && cable_violation_count == 0; }

  /** The one-line human verdict, worded exactly as the CLI reports it. */
  std::string verdict() const;
};

/** Runs a carrier over a whole trajectory and says how hard it is being worked and where it fails.
 *
 *  This is the analysis the carrier_trajectory_check CLI performs, lifted out of its main() so a
 *  BehaviorTree node (or anything else) can run the same numbers without shelling out. The CLI is
 *  now a thin wrapper over this class, so the two can never disagree.
 *
 *  NOT thread safe: the rod solver underneath keeps a warm start between calls. Give each thread
 *  its own analyzer. */
class CarrierTrajectoryAnalyzer
{
public:
  struct Options
  {
    /** Threshold above which a waypoint counts as "over-worked". <0 takes
     *  CarrierParams::bend_utilisation_warn, which is the CLI's behaviour. */
    double bend_utilisation_warn = -1.0;
    /** Run a planning-scene collision check per waypoint. Requires a non-null scene. */
    bool check_collision = true;
    /** Stop at the first waypoint that fails outright. Cheap early-out for a condition check;
     *  leave false when the caller wants a full report to display. */
    bool stop_at_first_failure = false;
    /** Keep the per-waypoint rows. Set false to get only the summary, which is all a gate needs. */
    bool keep_waypoints = true;
    /** Polled every `cancel_check_stride` waypoints so a long analysis can be halted. */
    const std::atomic_bool* cancel = nullptr;
    size_t cancel_check_stride = 16;
  };

  CarrierTrajectoryAnalyzer(const CarrierParams& params, const moveit::core::RobotModelConstPtr& model);

  /** False when the mount links are missing from the robot model; the analyzer is then inert. */
  bool valid() const { return attacher_.valid(); }
  const CarrierParams& params() const { return attacher_.params(); }

  /** Analyse a whole trajectory.
   *
   *  `times_from_start` may be empty, in which case every result carries time 0. `scene` may be
   *  null, which disables the collision column regardless of Options::check_collision -- the bend,
   *  twist and cable numbers come from the analyzer's own attacher and are always computed. */
  CarrierTrajectoryReport analyze(const std::vector<moveit::core::RobotState>& states,
                                  const std::vector<double>& times_from_start,
                                  const planning_scene::PlanningScene* scene, const Options& options) const;

  /** One state. Used by the condition/diagnostics behaviors, which do not have a trajectory. */
  CarrierWaypointResult analyzeState(const moveit::core::RobotState& state,
                                     const planning_scene::PlanningScene* scene) const;

private:
  /** Fill the scalar fields from a solved shape. Collision is added by the caller, which is the
   *  only part that needs a planning scene. */
  static CarrierWaypointResult toResult(const CarrierShape& shape);

  /** computeShape() is const; CarrierAttacher already keeps its own warm start mutable. */
  CarrierAttacher attacher_;
};

}  // namespace moveit_cable_carrier
