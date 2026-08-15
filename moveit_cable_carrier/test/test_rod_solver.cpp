// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit_cable_carrier/carrier_registry.hpp>
#include <moveit_cable_carrier/rod_solver.hpp>

#include <gtest/gtest.h>

#include <sstream>
#include <string>

using moveit_cable_carrier::BendMode;
using moveit_cable_carrier::CarrierParams;
using moveit_cable_carrier::CarrierShape;
using moveit_cable_carrier::RodSolver;

namespace
{
CarrierParams makeParams()
{
  CarrierParams p;
  p.length = 0.50;
  p.bend_radius = 0.04;
  p.outer_height = 0.016;
  p.outer_width = 0.026;
  p.num_segments = 24;
  p.max_iterations = 200;
  p.tolerance = 1e-5;
  p.safety_margin = 0.006;
  p.gravity_sag = 0.0;  // deterministic for tests
  p.bend_mode = BendMode::Spatial;
  p.unilateral = false;
  return p;
}

Eigen::Isometry3d bracket(const Eigen::Vector3d& t, const Eigen::Vector3d& x_dir)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = t;
  T.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), x_dir.normalized()).toRotationMatrix();
  return T;
}

double totalLength(const CarrierShape& s)
{
  double L = 0.0;
  for (size_t i = 0; i + 1 < s.nodes.size(); ++i)
  {
    L += (s.nodes[i + 1] - s.nodes[i]).norm();
  }
  return L;
}

/** Residuals in the failure message: without them a bare "feasible == false" says nothing about
 *  which constraint the solve could not satisfy. */
std::string diag(const CarrierShape& s, const CarrierParams& p)
{
  std::ostringstream os;
  os << "edge_err=" << s.endpoint_error << " (limit " << 10 * p.tolerance << ")"
     << "  max_curvature=" << s.max_curvature << " (limit " << 1.0 / p.bend_radius * 1.05 << ")"
     << "  length=" << totalLength(s) << " (target " << p.length << ")"
     << "  iters=" << s.iterations;
  return os.str();
}
}  // namespace

// A straight run whose chord exactly equals the arc length must come out straight.
TEST(RodSolver, StraightRunIsStraight)
{
  CarrierParams p = makeParams();
  RodSolver solver(p);
  const auto a = bracket(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
  const auto b = bracket(Eigen::Vector3d(p.length, 0, 0), Eigen::Vector3d::UnitX());

  const CarrierShape s = solver.solve(a, b);
  ASSERT_TRUE(s.feasible) << diag(s, p);
  EXPECT_NEAR(totalLength(s), p.length, 1e-3);
  EXPECT_LT(s.max_curvature, 1.0) << "a straight configuration must not bend";
  for (const auto& n : s.nodes)
  {
    EXPECT_NEAR(n.y(), 0.0, 1e-6);
    EXPECT_NEAR(n.z(), 0.0, 1e-6);
  }
}

// Arc length is conserved: this is the property the whole collision model rests on.
TEST(RodSolver, LengthIsConserved)
{
  CarrierParams p = makeParams();
  RodSolver solver(p);
  const auto a = bracket(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());

  // Stay clear of the carrier length: a chord approaching L is genuinely unreachable and is
  // supposed to come back infeasible (OverstretchIsInfeasible covers that case).
  for (double d = 0.20; d <= 0.40; d += 0.05)
  {
    const auto b = bracket(Eigen::Vector3d(d, 0.05, 0.02), Eigen::Vector3d::UnitX());
    const CarrierShape s = solver.solve(a, b);
    ASSERT_TRUE(s.feasible) << "chord " << d << " is well inside the carrier length; " << diag(s, p);

    // Inextensibility is exact in this formulation, not merely approximate: the unknowns are the
    // joint rotations, so no edge can change length no matter how badly the solve converges.
    const double seg = p.length / p.num_segments;
    for (size_t i = 0; i + 1 < s.nodes.size(); ++i)
    {
      EXPECT_NEAR((s.nodes[i + 1] - s.nodes[i]).norm(), seg, 1e-12) << "chord " << d << " edge " << i;
    }
    EXPECT_NEAR(totalLength(s), p.length, 1e-9) << "chord " << d;
  }
}

// The endpoints and the exit tangents are hard constraints imposed by the brackets.
TEST(RodSolver, EndpointsAndTangentsAreClamped)
{
  CarrierParams p = makeParams();
  RodSolver solver(p);
  const Eigen::Vector3d ta(0, 0, 0), tb(0.30, 0.10, 0.0);
  const Eigen::Vector3d da = Eigen::Vector3d::UnitX(), db = Eigen::Vector3d(0, 1, 0);
  const CarrierShape s = solver.solve(bracket(ta, da), bracket(tb, db));
  ASSERT_TRUE(s.feasible) << diag(s, p);

  // The base bracket is exact by construction: the first edge *is* the bracket frame.
  EXPECT_NEAR((s.nodes.front() - ta).norm(), 0.0, 1e-12);
  const Eigen::Vector3d first = (s.nodes[1] - s.nodes[0]).normalized();
  EXPECT_GT(first.dot(da), 1.0 - 1e-12);

  // The moving bracket is what the iteration drives to, so it is met to the solver's tolerance
  // rather than exactly. That is the deliberate trade: length and bend radius are exact, and the
  // residual here is absorbed by CarrierParams::safety_margin.
  EXPECT_LT((s.nodes.back() - tb).norm(), 1e-3) << diag(s, p);
  const Eigen::Vector3d last = (s.nodes[s.nodes.size() - 1] - s.nodes[s.nodes.size() - 2]).normalized();
  EXPECT_GT(last.dot(db), 0.99) << diag(s, p);
}

// Brackets further apart than the carrier is long cannot be joined; that must be reported,
// not silently approximated by a stretched carrier.
TEST(RodSolver, OverstretchIsInfeasible)
{
  CarrierParams p = makeParams();
  RodSolver solver(p);
  const auto a = bracket(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
  const auto b = bracket(Eigen::Vector3d(p.length * 1.2, 0, 0), Eigen::Vector3d::UnitX());
  EXPECT_FALSE(solver.solve(a, b).feasible);
}

// Simplification must stay conservative: the inflated radius has to cover every original node.
TEST(RodSolver, SimplifyStaysConservative)
{
  CarrierParams p = makeParams();
  RodSolver solver(p);
  const auto a = bracket(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
  const auto b = bracket(Eigen::Vector3d(0.25, 0.12, 0.03), Eigen::Vector3d(1, 1, 0));
  const CarrierShape full = solver.solve(a, b);
  ASSERT_TRUE(full.feasible) << diag(full, p);

  const CarrierShape simple = full.simplify(0.004);
  ASSERT_LE(simple.nodes.size(), full.nodes.size());
  EXPECT_GE(simple.radius, full.radius);

  for (const auto& node : full.nodes)
  {
    double best = std::numeric_limits<double>::max();
    for (size_t i = 0; i + 1 < simple.nodes.size(); ++i)
    {
      const Eigen::Vector3d s0 = simple.nodes[i], s1 = simple.nodes[i + 1];
      const Eigen::Vector3d seg = s1 - s0;
      const double len2 = seg.squaredNorm();
      const double t = len2 > 1e-12 ? std::clamp((node - s0).dot(seg) / len2, 0.0, 1.0) : 0.0;
      best = std::min(best, (node - (s0 + t * seg)).norm());
    }
    EXPECT_LE(best, simple.radius) << "simplified chain must still enclose the true centreline";
  }
}

// The back-stop: a real drag chain refuses to bend the wrong way.
TEST(RodSolver, BackStopLimitsReverseBending)
{
  CarrierParams p = makeParams();
  p.bend_mode = BendMode::Planar;
  p.unilateral = true;
  p.bend_axis = Eigen::Vector3d::UnitZ();
  RodSolver solver(p);

  // Canonical drag-chain layout: the two brackets face each other and the chain folds through
  // 180 degrees. Every turn is then on the same side, which is exactly what a back-stopped chain
  // can do. (A layout that needs turns on both sides is not a valid mounting for this hardware.)
  const auto a = bracket(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
  const auto b = bracket(Eigen::Vector3d(0.0, 0.10, 0.0), -Eigen::Vector3d::UnitX());
  const CarrierShape s = solver.solve(a, b);
  ASSERT_TRUE(s.feasible) << diag(s, p);

  // Every turn that survives must be on the permitted side of the back-stop.
  int wrong_side = 0;
  for (size_t i = 1; i + 1 < s.nodes.size(); ++i)
  {
    const Eigen::Vector3d e1 = s.nodes[i] - s.nodes[i - 1];
    const Eigen::Vector3d e2 = s.nodes[i + 1] - s.nodes[i];
    const Eigen::Vector3d c = e1.cross(e2);
    if (c.norm() > 1e-6 && c.normalized().dot(Eigen::Vector3d::UnitZ()) < -0.2)
    {
      const double theta = std::atan2(c.norm(), e1.dot(e2));
      if (theta > 0.05)
      {
        ++wrong_side;
      }
    }
  }
  EXPECT_EQ(wrong_side, 0) << "chain bent against its back-stop at " << wrong_side << " nodes";
}

// The hardware's minimum bend radius is a hard limit, and this formulation enforces it by
// construction rather than by penalty -- check that it really holds even for a cramped layout.
TEST(RodSolver, BendRadiusIsNeverViolated)
{
  CarrierParams p = makeParams();
  RodSolver solver(p);
  const double seg = p.length / p.num_segments;
  const double theta_max = seg / p.bend_radius;

  const auto a = bracket(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
  for (double y = -0.15; y <= 0.15; y += 0.05)
  {
    const auto b = bracket(Eigen::Vector3d(0.15, y, 0.05), Eigen::Vector3d(0, 0, 1));
    const CarrierShape s = solver.solve(a, b);
    for (size_t i = 1; i + 1 < s.nodes.size(); ++i)
    {
      const Eigen::Vector3d e1 = s.nodes[i] - s.nodes[i - 1];
      const Eigen::Vector3d e2 = s.nodes[i + 1] - s.nodes[i];
      const double theta = std::atan2(e1.cross(e2).norm(), e1.dot(e2));
      EXPECT_LE(theta, theta_max * (1.0 + 1e-6)) << "y=" << y << " node " << i;
    }
  }
}

TEST(CarrierRegistry, ParsesYaml)
{
  const std::string yaml = R"(
carriers:
  - name: unit_test
    length: 0.4
    bend_radius: 0.05
    base_link: a
    tip_link: b
    bend_mode: spatial
    base_mount: {xyz: [0.1, 0.0, 0.0], rpy: [0.0, 0.0, 0.0]}
    touch_links: [a, b]
)";
  std::vector<CarrierParams> out;
  std::string err;
  ASSERT_TRUE(moveit_cable_carrier::parseCarrierYaml(yaml, out, &err)) << err;
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].name, "unit_test");
  EXPECT_DOUBLE_EQ(out[0].length, 0.4);
  EXPECT_EQ(out[0].bend_mode, BendMode::Spatial);
  EXPECT_NEAR(out[0].base_mount.translation().x(), 0.1, 1e-12);
  EXPECT_EQ(out[0].touch_links.size(), 2u);
}

TEST(CarrierRegistry, RejectsMissingLinks)
{
  const std::string yaml = "carriers:\n  - name: bad\n    length: 0.4\n";
  std::vector<CarrierParams> out;
  std::string err;
  EXPECT_FALSE(moveit_cable_carrier::parseCarrierYaml(yaml, out, &err));
  EXPECT_FALSE(err.empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
