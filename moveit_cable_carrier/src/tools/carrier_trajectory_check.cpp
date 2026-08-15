// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// Motion-cycle analysis for a cable carrier, in the spirit of what commercial cable-simulation
// tools (e.g. Siemens Kineo Flexible Cables) report: run a whole trajectory, and say how hard the
// carrier is being worked and where it fails.
//
// Per step it reports whether the carrier can physically take the required shape, the tightest
// bend radius reached against the hardware limit, the peak bending and torsional stress, and
// whether the carrier collides. Over the whole run it accumulates the flex cycle count per
// segment, which is what actually determines service life: a drag chain fails where it is bent
// repeatedly, not where it is bent once.
//
//   ros2 run moveit_cable_carrier carrier_trajectory_check \
//        <urdf> <srdf> <carrier.yaml> <trajectory.csv> [report.json]
//
// trajectory.csv: first line is a comma-separated list of joint names, each following line is one
// waypoint of joint positions in radians. Lines starting with '#' are ignored.

#include <moveit_cable_carrier/carrier_attacher.hpp>
#include <moveit_cable_carrier/carrier_registry.hpp>
#include <moveit_cable_carrier/collision_env_carrier.hpp>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace
{
std::string readFile(const std::string& path)
{
  std::ifstream f(path);
  if (!f)
  {
    return {};
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

std::vector<std::string> splitCsv(const std::string& line)
{
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ','))
  {
    const size_t b = cell.find_first_not_of(" \t\r\n");
    const size_t e = cell.find_last_not_of(" \t\r\n");
    out.push_back(b == std::string::npos ? "" : cell.substr(b, e - b + 1));
  }
  return out;
}
}  // namespace

int main(int argc, char** argv)
{
  if (argc < 5)
  {
    std::fprintf(stderr, "usage: %s <urdf> <srdf> <carrier.yaml> <trajectory.csv> [report.json]\n", argv[0]);
    return 1;
  }
  const std::string report_json = (argc > 5) ? argv[5] : "";

  const std::string urdf_text = readFile(argv[1]);
  if (urdf_text.empty())
  {
    std::fprintf(stderr, "cannot read URDF %s\n", argv[1]);
    return 1;
  }
  auto urdf_model = urdf::parseURDF(urdf_text);
  auto srdf_model = std::make_shared<srdf::Model>();
  if (!urdf_model || !srdf_model->initFile(*urdf_model, argv[2]))
  {
    std::fprintf(stderr, "cannot parse URDF/SRDF\n");
    return 1;
  }
  auto robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);

  std::string err;
  if (!moveit_cable_carrier::CarrierRegistry::instance().loadFromYaml(argv[3], &err))
  {
    std::fprintf(stderr, "cannot load carrier config: %s\n", err.c_str());
    return 1;
  }
  const auto carriers = moveit_cable_carrier::CarrierRegistry::instance().carriers();
  if (carriers.empty())
  {
    std::fprintf(stderr, "no carriers configured\n");
    return 1;
  }
  const auto& params = carriers.front();
  moveit_cable_carrier::CarrierAttacher attacher(params, robot_model);
  if (!attacher.valid())
  {
    std::fprintf(stderr, "attacher invalid: check base_link/tip_link\n");
    return 1;
  }

  // ---- trajectory ----
  std::ifstream traj(argv[4]);
  if (!traj)
  {
    std::fprintf(stderr, "cannot read trajectory %s\n", argv[4]);
    return 1;
  }
  std::vector<std::string> joint_names;
  std::vector<std::vector<double>> waypoints;
  std::string line;
  while (std::getline(traj, line))
  {
    if (line.empty() || line[0] == '#')
    {
      continue;
    }
    const auto cells = splitCsv(line);
    if (joint_names.empty())
    {
      joint_names = cells;
      continue;
    }
    std::vector<double> q;
    q.reserve(cells.size());
    for (const auto& c : cells)
    {
      q.push_back(c.empty() ? 0.0 : std::stod(c));
    }
    if (q.size() != joint_names.size())
    {
      std::fprintf(stderr, "waypoint %zu has %zu values but %zu joint names\n", waypoints.size() + 1, q.size(),
                   joint_names.size());
      return 1;
    }
    waypoints.push_back(std::move(q));
  }
  if (waypoints.empty())
  {
    std::fprintf(stderr, "trajectory has no waypoints\n");
    return 1;
  }
  for (const auto& name : joint_names)
  {
    if (!robot_model->hasJointModel(name))
    {
      std::fprintf(stderr, "unknown joint '%s'\n", name.c_str());
      return 1;
    }
  }

  auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  scene->allocateCollisionDetector(moveit_cable_carrier::CollisionDetectorAllocatorCarrier::create());
  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();

  std::printf("carrier '%s'  L=%.3f m  R_min=%.3f m  E=%.2g Pa  G=%.2g Pa\n", params.name.c_str(), params.length,
              params.bend_radius, params.youngs_modulus, params.shear_modulus);
  std::printf("mount %s -> %s   waypoints %zu   joints %zu\n\n", params.base_link.c_str(), params.tip_link.c_str(),
              waypoints.size(), joint_names.size());
  std::printf("%5s %5s %10s %8s %8s %12s %6s\n", "step", "ok", "R[mm]", "bend/lim", "twist/lim", "cableR/OD",
              "coll");
  std::printf("%s\n", std::string(66, '-').c_str());

  // ---- per-segment flex history, for the fatigue view ----
  const size_t joints = static_cast<size_t>(std::max(0, params.num_segments - 1));
  std::vector<double> seg_min(joints, 1e30), seg_max(joints, 0.0), seg_prev(joints, 0.0);
  std::vector<long> seg_reversals(joints, 0);
  std::vector<int> seg_dir(joints, 0);

  long infeasible = 0, colliding = 0, over_worked = 0, cable_violations = 0;
  double worst_util = 0.0, worst_twist_util = 0.0, worst_strain = 0.0;
  double worst_cable_ratio = 1e30, required_cable_ratio = 0.0;
  std::string worst_cable_name;
  double tightest_radius = 1e30;
  std::ostringstream rows;

  for (size_t step = 0; step < waypoints.size(); ++step)
  {
    for (size_t j = 0; j < joint_names.size(); ++j)
    {
      state.setJointPositions(joint_names[j], &waypoints[step][j]);
    }
    state.update();

    const auto shape = attacher.computeShape(state);

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    scene->checkCollision(req, res, state);

    const double radius_mm = 1000.0 * shape.min_bend_radius();

    if (!shape.feasible)
    {
      ++infeasible;
    }
    if (res.collision)
    {
      ++colliding;
    }
    if (shape.feasible && shape.bend_utilisation > params.bend_utilisation_warn)
    {
      ++over_worked;
    }
    if (shape.feasible)
    {
      worst_util = std::max(worst_util, shape.bend_utilisation);
      worst_twist_util = std::max(worst_twist_util, shape.twist_utilisation);
      worst_strain = std::max(worst_strain, shape.max_cable_strain);
      if (shape.required_cable_bend_ratio > 0.0)
      {
        if (shape.min_cable_bend_ratio < worst_cable_ratio)
        {
          worst_cable_ratio = shape.min_cable_bend_ratio;
          worst_cable_name = shape.worst_cable;
          required_cable_ratio = shape.required_cable_bend_ratio;
        }
        if (!shape.cablesWithinLimit())
        {
          ++cable_violations;
        }
      }
      tightest_radius = std::min(tightest_radius, shape.min_bend_radius());

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

    char cable_cell[24];
    if (shape.required_cable_bend_ratio > 0.0 && shape.feasible)
    {
      std::snprintf(cable_cell, sizeof(cable_cell), "%.1f/%.0f%s", shape.min_cable_bend_ratio,
                    shape.required_cable_bend_ratio, shape.cablesWithinLimit() ? "" : " !");
    }
    else
    {
      std::snprintf(cable_cell, sizeof(cable_cell), "-");
    }
    std::printf("%5zu %5s %10.1f %8.2f %8.2f %12s %6s\n", step, shape.feasible ? "yes" : "NO",
                shape.feasible ? radius_mm : 0.0, shape.bend_utilisation, shape.twist_utilisation,
                cable_cell, res.collision ? "YES" : "-");

    if (!report_json.empty())
    {
      rows << (step ? ",\n  " : "\n  ") << "{\"step\": " << step << ", \"feasible\": "
           << (shape.feasible ? "true" : "false") << ", \"collision\": " << (res.collision ? "true" : "false")
           << ", \"min_bend_radius\": " << shape.min_bend_radius()
           << ", \"bend_utilisation\": " << shape.bend_utilisation
           << ", \"twist_utilisation\": " << shape.twist_utilisation
           << ", \"cable_bend_ratio\": " << shape.min_cable_bend_ratio
           << ", \"cable_strain\": " << shape.max_cable_strain << "}";
    }
  }

  std::printf("\n=== summary over %zu waypoints ===\n", waypoints.size());
  std::printf("infeasible shape   : %ld\n", infeasible);
  std::printf("in collision       : %ld\n", colliding);
  std::printf("above %.0f%% of R_min: %ld\n", 100.0 * params.bend_utilisation_warn, over_worked);
  if (tightest_radius < 1e29)
  {
    std::printf("tightest radius    : %.1f mm  (hardware limit %.1f mm, utilisation %.2f)\n",
                1000.0 * tightest_radius, 1000.0 * params.bend_radius, worst_util);
  }
  std::printf("peak twist per link: %.2f of the torsion stop\n", worst_twist_util);
  if (required_cable_ratio > 0.0)
  {
    std::printf("worst inner cable  : '%s' at %.1fx OD (needs %.0fx), strain %.2f%%  -- %ld waypoints over limit\n",
                worst_cable_name.c_str(), worst_cable_ratio, required_cable_ratio, 100.0 * worst_strain,
                cable_violations);
  }
  else
  {
    std::printf("inner cables       : none declared, so no cable bend-radius check was made\n");
  }

  // Which segment is worked hardest -- that is where the carrier will fail first.
  size_t hot = 0;
  double hot_range = -1.0;
  for (size_t k = 0; k < joints; ++k)
  {
    if (seg_min[k] > 1e29)
    {
      continue;
    }
    const double range = seg_max[k] - seg_min[k];
    if (range > hot_range)
    {
      hot_range = range;
      hot = k;
    }
  }
  if (hot_range >= 0.0)
  {
    std::printf("\nmost-worked segment: #%zu of %zu  (curvature swing %.1f -> %.1f 1/m, %ld reversals)\n", hot,
                joints, seg_min[hot], seg_max[hot], seg_reversals[hot]);
    std::printf("  -> this is where the carrier fatigues first; add slack or move a bracket to relieve it.\n");
  }

  if (infeasible || colliding || cable_violations)
  {
    std::printf("\nVERDICT: this trajectory is NOT safe for the carrier as mounted.\n");
  }
  else if (over_worked)
  {
    std::printf("\nVERDICT: reachable, but the carrier runs tighter than %.0f%% of R_min on %ld waypoints.\n",
                100.0 * params.bend_utilisation_warn, over_worked);
  }
  else
  {
    std::printf("\nVERDICT: carrier stays within its bend limit and clear of collisions.\n");
  }

  if (!report_json.empty())
  {
    std::ofstream out(report_json);
    if (out)
    {
      out << "{\"carrier\": \"" << params.name << "\", \"bend_radius\": " << params.bend_radius
          << ", \"waypoints\": [" << rows.str() << "\n ]}\n";
      std::printf("\nwrote %s\n", report_json.c_str());
    }
  }
  return (infeasible || colliding || cable_violations) ? 2 : 0;
}
