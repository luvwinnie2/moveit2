// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit_cable_carrier/trajectory_analysis.hpp>

#include <algorithm>
#include <cstdio>
#include <limits>

namespace moveit_cable_carrier
{
namespace
{
/** Sentinels used while accumulating, so "never set" is distinguishable from "set to zero".
 *  Normalised away before the report is returned. */
constexpr double kNoRadius = 1e30;
constexpr double kNoRatio = 1e30;
}  // namespace

std::string CarrierTrajectoryReport::verdict() const
{
  char buf[256];
  if (!safe())
  {
    return "this trajectory is NOT safe for the carrier as mounted.";
  }
  if (over_worked_count)
  {
    std::snprintf(buf, sizeof(buf), "reachable, but the carrier runs tighter than %.0f%% of R_min on %ld waypoints.",
                  100.0 * bend_utilisation_warn, over_worked_count);
    return buf;
  }
  return "carrier stays within its bend limit and clear of collisions.";
}

CarrierTrajectoryAnalyzer::CarrierTrajectoryAnalyzer(const CarrierParams& params,
                                                     const moveit::core::RobotModelConstPtr& model)
  : attacher_(params, model)
{
}

CarrierWaypointResult CarrierTrajectoryAnalyzer::toResult(const CarrierShape& shape)
{
  CarrierWaypointResult out;
  out.feasible = shape.feasible;
  out.min_bend_radius = shape.min_bend_radius();
  out.bend_utilisation = shape.bend_utilisation;
  out.twist_utilisation = shape.twist_utilisation;
  out.cable_bend_ratio = shape.min_cable_bend_ratio;
  out.required_cable_bend_ratio = shape.required_cable_bend_ratio;
  out.cable_strain = shape.max_cable_strain;
  out.tension = shape.tension;
  out.max_penetration = shape.max_penetration;
  out.worst_cable = shape.worst_cable;
  out.penetrating_obstacle = shape.penetrating_obstacle;
  return out;
}

CarrierWaypointResult CarrierTrajectoryAnalyzer::analyzeState(const moveit::core::RobotState& state,
                                                              const planning_scene::PlanningScene* scene) const
{
  if (!attacher_.valid())
  {
    return CarrierWaypointResult{};
  }
  CarrierWaypointResult out = toResult(attacher_.computeShape(state));
  if (scene)
  {
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    scene->checkCollision(req, res, state);
    out.collision = res.collision;
  }
  return out;
}

CarrierTrajectoryReport CarrierTrajectoryAnalyzer::analyze(const std::vector<moveit::core::RobotState>& states,
                                                           const std::vector<double>& times_from_start,
                                                           const planning_scene::PlanningScene* scene,
                                                           const Options& options) const
{
  const CarrierParams& params = attacher_.params();

  CarrierTrajectoryReport report;
  report.carrier_name = params.name;
  report.bend_radius = params.bend_radius;
  report.bend_utilisation_warn =
      options.bend_utilisation_warn >= 0.0 ? options.bend_utilisation_warn : params.bend_utilisation_warn;

  if (!attacher_.valid() || states.empty())
  {
    return report;
  }

  const planning_scene::PlanningScene* collision_scene = options.check_collision ? scene : nullptr;

  // Per-segment flex history. `joints` counts the articulations between links, which is one fewer
  // than the number of links -- that is what CarrierShape::curvature is indexed by.
  const size_t joints = static_cast<size_t>(std::max(0, params.num_segments - 1));
  std::vector<double> seg_min(joints, kNoRadius), seg_max(joints, 0.0), seg_prev(joints, 0.0);
  std::vector<long> seg_reversals(joints, 0);
  std::vector<int> seg_dir(joints, 0);

  double worst_cable_ratio = kNoRatio;
  double tightest_radius = kNoRadius;

  if (options.keep_waypoints)
  {
    report.waypoints.reserve(states.size());
  }

  for (size_t step = 0; step < states.size(); ++step)
  {
    if (options.cancel && options.cancel_check_stride > 0 && (step % options.cancel_check_stride) == 0 &&
        options.cancel->load(std::memory_order_acquire))
    {
      break;
    }

    // computeShape() is called here rather than through analyzeState() because the per-segment
    // fatigue accumulation below needs the shape's per-joint curvature, and solving twice for the
    // same state would double the cost of the whole analysis.
    const CarrierShape shape = attacher_.computeShape(states[step]);
    CarrierWaypointResult wp = toResult(shape);
    wp.index = step;
    wp.time_from_start = step < times_from_start.size() ? times_from_start[step] : 0.0;
    if (collision_scene)
    {
      collision_detection::CollisionRequest req;
      collision_detection::CollisionResult res;
      collision_scene->checkCollision(req, res, states[step]);
      wp.collision = res.collision;
    }

    if (!wp.feasible)
    {
      ++report.infeasible_count;
    }
    if (wp.collision)
    {
      ++report.colliding_count;
    }
    if (wp.feasible && wp.bend_utilisation > report.bend_utilisation_warn)
    {
      ++report.over_worked_count;
    }

    if (wp.feasible)
    {
      report.worst_bend_utilisation = std::max(report.worst_bend_utilisation, wp.bend_utilisation);
      report.worst_twist_utilisation = std::max(report.worst_twist_utilisation, wp.twist_utilisation);
      report.worst_cable_strain = std::max(report.worst_cable_strain, wp.cable_strain);

      if (wp.required_cable_bend_ratio > 0.0)
      {
        if (wp.cable_bend_ratio < worst_cable_ratio)
        {
          worst_cable_ratio = wp.cable_bend_ratio;
          report.worst_cable = wp.worst_cable;
          report.required_cable_bend_ratio = wp.required_cable_bend_ratio;
        }
        if (!wp.cablesWithinLimit())
        {
          ++report.cable_violation_count;
        }
      }
      tightest_radius = std::min(tightest_radius, wp.min_bend_radius);

      for (size_t k = 0; k < joints && k < shape.curvature.size(); ++k)
      {
        const double kappa = shape.curvature[k];
        seg_min[k] = std::min(seg_min[k], kappa);
        seg_max[k] = std::max(seg_max[k], kappa);
        // A flex cycle is a reversal of the bending direction; counting reversals is the cheap
        // stand-in for a rainflow count and is what tells you which segment wears out first.
        if (step > 0)
        {
          const double d = kappa - seg_prev[k];
          const int dir = (d > 1e-4) ? 1 : (d < -1e-4 ? -1 : 0);
          if (dir != 0)
          {
            if (seg_dir[k] != 0 && dir != seg_dir[k])
            {
              ++seg_reversals[k];
            }
            seg_dir[k] = dir;
          }
        }
        seg_prev[k] = kappa;
      }
    }

    const bool bad = !wp.feasible || wp.collision || (wp.required_cable_bend_ratio > 0.0 && !wp.cablesWithinLimit());
    if (bad && report.first_bad_index < 0)
    {
      report.first_bad_index = static_cast<int>(step);
    }

    if (options.keep_waypoints)
    {
      report.waypoints.push_back(std::move(wp));
    }

    if (bad && options.stop_at_first_failure)
    {
      break;
    }
  }

  report.has_tightest_radius = tightest_radius < kNoRadius;
  report.tightest_radius = report.has_tightest_radius ? tightest_radius : 0.0;
  report.worst_cable_bend_ratio = worst_cable_ratio < kNoRatio ? worst_cable_ratio : 0.0;

  // Which segment is worked hardest -- that is where the carrier will fail first.
  report.segments.reserve(joints);
  double hot_range = -1.0;
  for (size_t k = 0; k < joints; ++k)
  {
    if (seg_min[k] >= kNoRadius)
    {
      continue;
    }
    CarrierSegmentFatigue f;
    f.segment = k;
    f.min_curvature = seg_min[k];
    f.max_curvature = seg_max[k];
    f.reversals = seg_reversals[k];
    report.segments.push_back(f);

    const double range = seg_max[k] - seg_min[k];
    if (range > hot_range)
    {
      hot_range = range;
      report.hottest_segment = k;
      report.hottest_segment_min_curvature = seg_min[k];
      report.hottest_segment_max_curvature = seg_max[k];
      report.hottest_segment_reversals = seg_reversals[k];
      report.has_hottest_segment = true;
    }
  }
  return report;
}

}  // namespace moveit_cable_carrier
