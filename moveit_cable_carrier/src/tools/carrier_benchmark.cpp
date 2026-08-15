// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// Sweeps the joints the carrier spans and reports, for the built-in rod solver:
//   - how often the carrier shape is physically feasible,
//   - the achieved bend radius versus the hardware limit,
//   - solve and full-collision-check timings.
//
// The timing number is the one that decides the architecture: if a per-state solve is cheap
// enough, no precomputed lookup table is needed and the planner can call the solver directly.
//
//   ros2 run moveit_cable_carrier carrier_benchmark <urdf> <srdf> <carrier.yaml> [samples_per_axis]

#include <moveit_cable_carrier/carrier_attacher.hpp>
#include <moveit_cable_carrier/carrier_registry.hpp>
#include <moveit_cable_carrier/collision_env_carrier.hpp>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <numeric>
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

double percentile(std::vector<double> v, double p)
{
  if (v.empty())
  {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  const size_t idx = std::min(v.size() - 1, static_cast<size_t>(p * (v.size() - 1)));
  return v[idx];
}
}  // namespace

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::fprintf(stderr, "usage: %s <urdf> <srdf> <carrier.yaml> [samples_per_axis] [shapes.json] [cases.json]\n",
                 argv[0]);
    return 1;
  }
  const std::string urdf_path = argv[1];
  const std::string srdf_path = argv[2];
  const std::string yaml_path = argv[3];
  const int samples = (argc > 4) ? std::atoi(argv[4]) : 9;
  const std::string shapes_json = (argc > 5) ? argv[5] : "";
  const std::string cases_json = (argc > 6) ? argv[6] : "";

  const std::string urdf_text = readFile(urdf_path);
  if (urdf_text.empty())
  {
    std::fprintf(stderr, "cannot read URDF %s\n", urdf_path.c_str());
    return 1;
  }
  auto urdf_model = urdf::parseURDF(urdf_text);
  if (!urdf_model)
  {
    std::fprintf(stderr, "cannot parse URDF\n");
    return 1;
  }
  auto srdf_model = std::make_shared<srdf::Model>();
  if (!srdf_model->initFile(*urdf_model, srdf_path))
  {
    std::fprintf(stderr, "cannot parse SRDF %s\n", srdf_path.c_str());
    return 1;
  }
  auto robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);

  std::string err;
  if (!moveit_cable_carrier::CarrierRegistry::instance().loadFromYaml(yaml_path, &err))
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
  std::printf("carrier '%s': L=%.3f m  R_min=%.3f m  N=%d  margin=%.3f m\n", params.name.c_str(), params.length,
              params.bend_radius, params.num_segments, params.safety_margin);
  std::printf("mount: %s -> %s\n\n", params.base_link.c_str(), params.tip_link.c_str());

  // Joints strictly between the two mount links are the ones that change the carrier's shape.
  std::vector<const moveit::core::JointModel*> spanned;
  {
    const moveit::core::LinkModel* link = robot_model->getLinkModel(params.tip_link);
    while (link && link->getName() != params.base_link)
    {
      const moveit::core::JointModel* pj = link->getParentJointModel();
      if (pj && pj->getVariableCount() == 1)
      {
        spanned.push_back(pj);
      }
      link = pj ? pj->getParentLinkModel() : nullptr;
    }
    std::reverse(spanned.begin(), spanned.end());
  }
  std::printf("spanned joints (%zu):", spanned.size());
  for (const auto* j : spanned)
  {
    std::printf(" %s", j->getName().c_str());
  }
  std::printf("\n\n");
  if (spanned.empty())
  {
    std::fprintf(stderr, "no spanned joints found -- check base_link/tip_link\n");
    return 1;
  }

  moveit_cable_carrier::CarrierAttacher attacher(params, robot_model);
  if (!attacher.valid())
  {
    std::fprintf(stderr, "attacher invalid (unknown links)\n");
    return 1;
  }

  auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  scene->allocateCollisionDetector(moveit_cable_carrier::CollisionDetectorAllocatorCarrier::create());

  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();

  // Cartesian sweep over the spanned joints.
  std::vector<std::vector<double>> grids;
  for (const auto* j : spanned)
  {
    const auto& b = j->getVariableBounds()[0];
    const double lo = b.position_bounded_ ? b.min_position_ : -M_PI;
    const double hi = b.position_bounded_ ? b.max_position_ : M_PI;
    std::vector<double> g;
    for (int i = 0; i < samples; ++i)
    {
      g.push_back(lo + (hi - lo) * i / std::max(1, samples - 1));
    }
    grids.push_back(std::move(g));
  }

  long total = 1;
  for (const auto& g : grids)
  {
    total *= static_cast<long>(g.size());
  }

  long feasible = 0, colliding = 0;
  std::vector<double> solve_us, check_us, curvature, chords, penetration, strain, tension;
  solve_us.reserve(total);
  check_us.reserve(total);

  // Optional export so scripts/compare_reference.py can measure this solver against a real
  // deformable engine (see scripts/generate_reference_shapes.py).
  std::ostringstream shapes_out, cases_out;
  const bool dump = !shapes_json.empty();
  if (dump)
  {
    shapes_out << "{\"cases\": [";
    cases_out << "{\"carrier\": {\"length\": " << params.length << ", \"bend_radius\": " << params.bend_radius
              << ", \"num_segments\": " << params.num_segments << ", \"outer_height\": " << params.outer_height
              << ", \"outer_width\": " << params.outer_width << "}, \"cases\": [";
  }
  bool first_dump = true;

  for (long idx = 0; idx < total; ++idx)
  {
    long rem = idx;
    for (size_t d = 0; d < grids.size(); ++d)
    {
      const size_t n = grids[d].size();
      state.setJointPositions(spanned[d], &grids[d][rem % n]);
      rem /= static_cast<long>(n);
    }
    state.update();

    {
      const Eigen::Isometry3d tb = state.getGlobalLinkTransform(params.base_link) * params.base_mount;
      const Eigen::Isometry3d tt = state.getGlobalLinkTransform(params.tip_link) * params.tip_mount;
      chords.push_back((tt.translation() - tb.translation()).norm());
    }

    auto t0 = std::chrono::steady_clock::now();
    const auto shape = attacher.computeShape(state);
    auto t1 = std::chrono::steady_clock::now();
    solve_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());

    if (shape.feasible)
    {
      ++feasible;
      curvature.push_back(shape.max_curvature);
      penetration.push_back(shape.max_penetration);
      strain.push_back(shape.axial_strain);
      tension.push_back(shape.tension);
    }

    if (dump)
    {
      if (!first_dump)
      {
        shapes_out << ", ";
        cases_out << ", ";
      }
      first_dump = false;

      shapes_out << "{\"feasible\": " << (shape.feasible ? "true" : "false") << ", \"nodes\": [";
      for (size_t k = 0; k < shape.nodes.size(); ++k)
      {
        shapes_out << (k ? ", [" : "[") << shape.nodes[k].x() << ", " << shape.nodes[k].y() << ", "
                   << shape.nodes[k].z() << "]";
      }
      shapes_out << "]}";

      const Eigen::Isometry3d tb = state.getGlobalLinkTransform(params.base_link) * params.base_mount;
      const Eigen::Isometry3d tt = state.getGlobalLinkTransform(params.tip_link) * params.tip_mount;
      const Eigen::Quaterniond qb(tb.linear()), qt(tt.linear());
      cases_out << "{\"base\": {\"xyz\": [" << tb.translation().x() << ", " << tb.translation().y() << ", "
                << tb.translation().z() << "], \"quat_xyzw\": [" << qb.x() << ", " << qb.y() << ", " << qb.z()
                << ", " << qb.w() << "]}, \"tip\": {\"xyz\": [" << tt.translation().x() << ", "
                << tt.translation().y() << ", " << tt.translation().z() << "], \"quat_xyzw\": [" << qt.x() << ", "
                << qt.y() << ", " << qt.z() << ", " << qt.w() << "]}}";
    }

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    auto t2 = std::chrono::steady_clock::now();
    scene->checkCollision(req, res, state);
    auto t3 = std::chrono::steady_clock::now();
    check_us.push_back(std::chrono::duration<double, std::micro>(t3 - t2).count());
    if (res.collision)
    {
      ++colliding;
    }
  }

  const double mean_solve = solve_us.empty() ? 0.0 : std::accumulate(solve_us.begin(), solve_us.end(), 0.0) / solve_us.size();
  const double mean_check = check_us.empty() ? 0.0 : std::accumulate(check_us.begin(), check_us.end(), 0.0) / check_us.size();

  std::printf("bracket chord          : min %.3f m  p50 %.3f  max %.3f  (carrier length %.3f m)\n",
              percentile(chords, 0.0), percentile(chords, 0.5), percentile(chords, 1.0), params.length);
  if (percentile(chords, 1.0) >= params.length)
  {
    std::printf("  -> carrier is TOO SHORT: it must exceed the largest chord, with slack on top.\n");
  }
  std::printf("samples                : %ld (%d per axis)\n", total, samples);
  std::printf("carrier feasible       : %ld (%.1f%%)\n", feasible, 100.0 * feasible / total);
  std::printf("in collision           : %ld (%.1f%%)\n", colliding, 100.0 * colliding / total);
  std::printf("\nshape solve   mean %8.1f us   p50 %8.1f   p95 %8.1f   max %8.1f\n", mean_solve,
              percentile(solve_us, 0.50), percentile(solve_us, 0.95), percentile(solve_us, 1.0));
  std::printf("full check    mean %8.1f us   p50 %8.1f   p95 %8.1f   max %8.1f\n", mean_check,
              percentile(check_us, 0.50), percentile(check_us, 0.95), percentile(check_us, 1.0));
  if (!strain.empty() && percentile(strain, 1.0) > 0.0)
  {
    std::printf("axial strain          : max %.3f %%  -> tension %.1f N  (0 = never pulled taut)\n",
                100.0 * percentile(strain, 1.0), percentile(tension, 1.0));
  }
  if (!penetration.empty())
  {
    std::printf("link penetration      : p50 %.1f mm  p95 %.1f mm  max %.1f mm  (0 = the run lies on "
                "the arm rather than through it)\n",
                1000.0 * percentile(penetration, 0.5), 1000.0 * percentile(penetration, 0.95),
                1000.0 * percentile(penetration, 1.0));
  }
  if (!curvature.empty())
  {
    std::printf("\nachieved bend radius: min %.4f m (hardware limit %.4f m)\n", 1.0 / percentile(curvature, 1.0),
                params.bend_radius);
  }

  if (dump)
  {
    shapes_out << "]}";
    cases_out << "]}";
    std::ofstream sf(shapes_json);
    if (sf)
    {
      sf << shapes_out.str();
      std::printf("\nwrote solved shapes to %s\n", shapes_json.c_str());
    }
    else
    {
      std::fprintf(stderr, "cannot write %s\n", shapes_json.c_str());
    }
    if (!cases_json.empty())
    {
      std::ofstream cf(cases_json);
      if (cf)
      {
        cf << cases_out.str();
        std::printf("wrote reference cases to %s\n", cases_json.c_str());
      }
      else
      {
        std::fprintf(stderr, "cannot write %s\n", cases_json.c_str());
      }
    }
  }
  return 0;
}
