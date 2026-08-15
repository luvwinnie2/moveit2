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
// The analysis itself lives in CarrierTrajectoryAnalyzer so a BehaviorTree node can run exactly
// the same numbers without shelling out to this binary. This file is argument parsing and
// printing only -- keep it that way, so the CLI and the Behavior can never disagree.
//
//   ros2 run moveit_cable_carrier carrier_trajectory_check \
//        <urdf> <srdf> <carrier.yaml> <trajectory.csv> [report.json]
//
// trajectory.csv: first line is a comma-separated list of joint names, each following line is one
// waypoint of joint positions in radians. Lines starting with '#' are ignored.

#include <moveit_cable_carrier/carrier_registry.hpp>
#include <moveit_cable_carrier/collision_env_carrier.hpp>
#include <moveit_cable_carrier/trajectory_analysis.hpp>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <algorithm>
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
  moveit_cable_carrier::CarrierTrajectoryAnalyzer analyzer(params, robot_model);
  if (!analyzer.valid())
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
  std::vector<moveit::core::RobotState> states;
  states.reserve(waypoints.size());
  for (const auto& q : waypoints)
  {
    for (size_t j = 0; j < joint_names.size(); ++j)
    {
      state.setJointPositions(joint_names[j], &q[j]);
    }
    state.update();
    states.push_back(state);
  }

  std::printf("carrier '%s'  L=%.3f m  R_min=%.3f m  E=%.2g Pa  G=%.2g Pa\n", params.name.c_str(), params.length,
              params.bend_radius, params.youngs_modulus, params.shear_modulus);
  std::printf("mount %s -> %s   waypoints %zu   joints %zu\n\n", params.base_link.c_str(), params.tip_link.c_str(),
              waypoints.size(), joint_names.size());
  std::printf("%5s %5s %10s %8s %8s %12s %6s\n", "step", "ok", "R[mm]", "bend/lim", "twist/lim", "cableR/OD",
              "coll");
  std::printf("%s\n", std::string(66, '-').c_str());

  moveit_cable_carrier::CarrierTrajectoryAnalyzer::Options options;  // defaults match the CLI's behaviour
  const auto report = analyzer.analyze(states, {}, scene.get(), options);

  std::ostringstream rows;
  for (const auto& wp : report.waypoints)
  {
    char cable_cell[24];
    if (wp.required_cable_bend_ratio > 0.0 && wp.feasible)
    {
      std::snprintf(cable_cell, sizeof(cable_cell), "%.1f/%.0f%s", wp.cable_bend_ratio, wp.required_cable_bend_ratio,
                    wp.cablesWithinLimit() ? "" : " !");
    }
    else
    {
      std::snprintf(cable_cell, sizeof(cable_cell), "-");
    }
    std::printf("%5zu %5s %10.1f %8.2f %8.2f %12s %6s\n", wp.index, wp.feasible ? "yes" : "NO",
                wp.feasible ? 1000.0 * wp.min_bend_radius : 0.0, wp.bend_utilisation, wp.twist_utilisation,
                cable_cell, wp.collision ? "YES" : "-");

    if (!report_json.empty())
    {
      rows << (wp.index ? ",\n  " : "\n  ") << "{\"step\": " << wp.index << ", \"feasible\": "
           << (wp.feasible ? "true" : "false") << ", \"collision\": " << (wp.collision ? "true" : "false")
           << ", \"min_bend_radius\": " << wp.min_bend_radius << ", \"bend_utilisation\": " << wp.bend_utilisation
           << ", \"twist_utilisation\": " << wp.twist_utilisation << ", \"cable_bend_ratio\": " << wp.cable_bend_ratio
           << ", \"cable_strain\": " << wp.cable_strain << "}";
    }
  }

  std::printf("\n=== summary over %zu waypoints ===\n", waypoints.size());
  std::printf("infeasible shape   : %ld\n", report.infeasible_count);
  std::printf("in collision       : %ld\n", report.colliding_count);
  std::printf("above %.0f%% of R_min: %ld\n", 100.0 * report.bend_utilisation_warn, report.over_worked_count);
  if (report.has_tightest_radius)
  {
    std::printf("tightest radius    : %.1f mm  (hardware limit %.1f mm, utilisation %.2f)\n",
                1000.0 * report.tightest_radius, 1000.0 * params.bend_radius, report.worst_bend_utilisation);
  }
  std::printf("peak twist per link: %.2f of the torsion stop\n", report.worst_twist_utilisation);
  if (report.required_cable_bend_ratio > 0.0)
  {
    std::printf("worst inner cable  : '%s' at %.1fx OD (needs %.0fx), strain %.2f%%  -- %ld waypoints over limit\n",
                report.worst_cable.c_str(), report.worst_cable_bend_ratio, report.required_cable_bend_ratio,
                100.0 * report.worst_cable_strain, report.cable_violation_count);
  }
  else
  {
    std::printf("inner cables       : none declared, so no cable bend-radius check was made\n");
  }

  // Which segment is worked hardest -- that is where the carrier will fail first.
  if (report.has_hottest_segment)
  {
    const size_t joints = static_cast<size_t>(std::max(0, params.num_segments - 1));
    std::printf("\nmost-worked segment: #%zu of %zu  (curvature swing %.1f -> %.1f 1/m, %ld reversals)\n",
                report.hottest_segment, joints, report.hottest_segment_min_curvature,
                report.hottest_segment_max_curvature, report.hottest_segment_reversals);
    std::printf("  -> this is where the carrier fatigues first; add slack or move a bracket to relieve it.\n");
  }

  std::printf("\nVERDICT: %s\n", report.verdict().c_str());

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
  return report.safe() ? 0 : 2;
}
