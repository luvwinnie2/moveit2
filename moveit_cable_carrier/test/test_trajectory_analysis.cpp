// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// Pins CarrierTrajectoryAnalyzer, which is the logic the carrier_trajectory_check CLI used to
// carry inline in its main(). The CLI is now a thin wrapper over this class, so if these tests
// pass the CLI's numbers are the same numbers a BehaviorTree node will see.
//
// Deliberately hermetic: a two-joint arm is built from an inline URDF rather than loading
// crx5ia_moveit, so the test needs no other package and runs in milliseconds.

#include <moveit_cable_carrier/trajectory_analysis.hpp>

#include <moveit/robot_model/robot_model.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <string>
#include <vector>

using moveit_cable_carrier::BendMode;
using moveit_cable_carrier::CarrierParams;
using moveit_cable_carrier::CarrierTrajectoryAnalyzer;
using moveit_cable_carrier::CarrierTrajectoryReport;

namespace
{
/** Planar 2R arm. link2's origin is a fixed 0.30 m from the base, so the carrier's chord only
 *  varies once its tip bracket is offset out along link2 -- which is what makes joint_b sweep the
 *  carrier from over-stretched, through comfortable, to tightly folded. */
constexpr const char* kUrdf = R"(<?xml version="1.0"?>
<robot name="test_arm">
  <link name="base_link">
    <collision><origin xyz="0 0 0"/><geometry><box size="0.06 0.06 0.06"/></geometry></collision>
  </link>
  <link name="link1">
    <collision><origin xyz="0.15 0 0" rpy="0 1.5707963 0"/>
      <geometry><cylinder radius="0.03" length="0.30"/></geometry></collision>
  </link>
  <link name="link2">
    <collision><origin xyz="0.12 0 0" rpy="0 1.5707963 0"/>
      <geometry><cylinder radius="0.025" length="0.25"/></geometry></collision>
  </link>
  <joint name="joint_a" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.15" upper="3.15" effort="10" velocity="1"/>
  </joint>
  <joint name="joint_b" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0.30 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.15" upper="3.15" effort="10" velocity="1"/>
  </joint>
</robot>)";

constexpr const char* kSrdf = R"(<?xml version="1.0"?>
<robot name="test_arm">
  <group name="arm">
    <chain base_link="base_link" tip_link="link2"/>
  </group>
</robot>)";

moveit::core::RobotModelPtr makeModel()
{
  auto urdf_model = urdf::parseURDF(kUrdf);
  EXPECT_TRUE(static_cast<bool>(urdf_model)) << "inline URDF failed to parse";
  auto srdf_model = std::make_shared<srdf::Model>();
  EXPECT_TRUE(srdf_model->initString(*urdf_model, kSrdf)) << "inline SRDF failed to parse";
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

CarrierParams makeParams()
{
  CarrierParams p;
  p.name = "test_carrier";
  p.base_link = "base_link";
  p.tip_link = "link2";
  p.length = 0.42;
  p.bend_radius = 0.04;
  p.outer_height = 0.016;
  p.outer_width = 0.026;
  p.num_segments = 24;
  p.max_iterations = 200;
  p.tolerance = 1e-5;
  p.safety_margin = 0.004;
  p.gravity_sag = 0.0;  // deterministic
  p.bend_mode = BendMode::Spatial;
  p.unilateral = false;
  p.bend_utilisation_warn = 0.8;
  // Tip bracket sits out along link2 so the base->tip chord actually changes with joint_b.
  p.tip_mount = Eigen::Isometry3d::Identity();
  p.tip_mount.translation() = Eigen::Vector3d(0.25, 0.0, 0.0);
  return p;
}

/** Sweep joint_b over `count` samples between `from` and `to`, joint_a held at 0. */
std::vector<moveit::core::RobotState> sweep(const moveit::core::RobotModelPtr& model, double from, double to,
                                            size_t count)
{
  std::vector<moveit::core::RobotState> states;
  states.reserve(count);
  moveit::core::RobotState s(model);
  s.setToDefaultValues();
  for (size_t i = 0; i < count; ++i)
  {
    const double t = count > 1 ? static_cast<double>(i) / static_cast<double>(count - 1) : 0.0;
    double a = 0.0;
    double b = from + t * (to - from);
    s.setJointPositions("joint_a", &a);
    s.setJointPositions("joint_b", &b);
    s.update();
    states.push_back(s);
  }
  return states;
}
}  // namespace

TEST(TrajectoryAnalysis, ValidWhenMountLinksExist)
{
  auto model = makeModel();
  CarrierTrajectoryAnalyzer analyzer(makeParams(), model);
  EXPECT_TRUE(analyzer.valid());
  EXPECT_EQ(analyzer.params().name, "test_carrier");
}

TEST(TrajectoryAnalysis, InertWhenMountLinkMissing)
{
  auto model = makeModel();
  CarrierParams p = makeParams();
  p.tip_link = "no_such_link";
  CarrierTrajectoryAnalyzer analyzer(p, model);
  EXPECT_FALSE(analyzer.valid());

  const auto report = analyzer.analyze(sweep(model, 0.5, 2.5, 5), {}, nullptr, {});
  EXPECT_TRUE(report.waypoints.empty());
  EXPECT_EQ(report.infeasible_count, 0);
}

/** The sweep is chosen to cross from "chord longer than the carrier" to "carrier tightly folded",
 *  so a single run exercises the infeasible, comfortable and over-worked branches. */
TEST(TrajectoryAnalysis, SweepProducesAResultPerWaypoint)
{
  auto model = makeModel();
  CarrierTrajectoryAnalyzer analyzer(makeParams(), model);

  const auto states = sweep(model, 0.0, M_PI, 24);
  const auto report = analyzer.analyze(states, {}, nullptr, {});

  ASSERT_EQ(report.waypoints.size(), states.size());
  for (size_t i = 0; i < report.waypoints.size(); ++i)
  {
    EXPECT_EQ(report.waypoints[i].index, i);
  }
  EXPECT_EQ(report.carrier_name, "test_carrier");
  EXPECT_DOUBLE_EQ(report.bend_radius, 0.04);
  EXPECT_DOUBLE_EQ(report.bend_utilisation_warn, 0.8);

  // At joint_b = 0 the tip is 0.55 m from the base but the carrier is only 0.42 m long, so the
  // first waypoint cannot be reached. If this ever stops holding the fixture has drifted and the
  // rest of the assertions below stop meaning anything.
  EXPECT_FALSE(report.waypoints.front().feasible)
      << "fixture expects an over-stretched first waypoint; chord/length no longer disagree";
  EXPECT_GT(report.infeasible_count, 0);

  // ... and somewhere in the sweep it must become reachable, otherwise the solver is broken.
  bool any_feasible = false;
  for (const auto& wp : report.waypoints)
  {
    any_feasible = any_feasible || wp.feasible;
  }
  EXPECT_TRUE(any_feasible);
  EXPECT_TRUE(report.has_tightest_radius);
  EXPECT_GT(report.worst_bend_utilisation, 0.0);
}

TEST(TrajectoryAnalysis, SummaryMatchesPerWaypointRows)
{
  auto model = makeModel();
  CarrierTrajectoryAnalyzer analyzer(makeParams(), model);
  const auto report = analyzer.analyze(sweep(model, 0.0, M_PI, 24), {}, nullptr, {});

  long infeasible = 0, over_worked = 0;
  double worst_util = 0.0, worst_twist = 0.0;
  for (const auto& wp : report.waypoints)
  {
    if (!wp.feasible)
    {
      ++infeasible;
      continue;
    }
    if (wp.bend_utilisation > report.bend_utilisation_warn)
    {
      ++over_worked;
    }
    worst_util = std::max(worst_util, wp.bend_utilisation);
    worst_twist = std::max(worst_twist, wp.twist_utilisation);
  }
  EXPECT_EQ(report.infeasible_count, infeasible);
  EXPECT_EQ(report.over_worked_count, over_worked);
  EXPECT_DOUBLE_EQ(report.worst_bend_utilisation, worst_util);
  EXPECT_DOUBLE_EQ(report.worst_twist_utilisation, worst_twist);
}

TEST(TrajectoryAnalysis, TimesFromStartArePassedThrough)
{
  auto model = makeModel();
  CarrierTrajectoryAnalyzer analyzer(makeParams(), model);
  const auto states = sweep(model, 1.0, 2.5, 5);
  const std::vector<double> times{ 0.0, 0.25, 0.5, 0.75, 1.0 };

  const auto report = analyzer.analyze(states, times, nullptr, {});
  ASSERT_EQ(report.waypoints.size(), times.size());
  for (size_t i = 0; i < times.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(report.waypoints[i].time_from_start, times[i]);
  }

  // A short/absent timing vector must not be an error -- the CLI supplies none at all.
  const auto untimed = analyzer.analyze(states, {}, nullptr, {});
  for (const auto& wp : untimed.waypoints)
  {
    EXPECT_DOUBLE_EQ(wp.time_from_start, 0.0);
  }
}

TEST(TrajectoryAnalysis, StopAtFirstFailureTruncatesAtTheReportedIndex)
{
  auto model = makeModel();
  CarrierTrajectoryAnalyzer analyzer(makeParams(), model);
  const auto states = sweep(model, M_PI, 0.0, 24);  // reversed: starts folded, ends over-stretched

  CarrierTrajectoryAnalyzer::Options full;
  const auto full_report = analyzer.analyze(states, {}, nullptr, full);
  ASSERT_GE(full_report.first_bad_index, 0) << "fixture expects the reversed sweep to fail somewhere";

  CarrierTrajectoryAnalyzer::Options early;
  early.stop_at_first_failure = true;
  const auto early_report = analyzer.analyze(states, {}, nullptr, early);

  EXPECT_EQ(early_report.first_bad_index, full_report.first_bad_index);
  EXPECT_EQ(static_cast<int>(early_report.waypoints.size()), full_report.first_bad_index + 1);
  EXPECT_LT(early_report.waypoints.size(), full_report.waypoints.size());
}

TEST(TrajectoryAnalysis, KeepWaypointsFalseStillCounts)
{
  auto model = makeModel();
  CarrierTrajectoryAnalyzer analyzer(makeParams(), model);
  const auto states = sweep(model, 0.0, M_PI, 24);

  const auto with_rows = analyzer.analyze(states, {}, nullptr, {});
  CarrierTrajectoryAnalyzer::Options lean;
  lean.keep_waypoints = false;
  const auto without_rows = analyzer.analyze(states, {}, nullptr, lean);

  EXPECT_TRUE(without_rows.waypoints.empty());
  EXPECT_EQ(without_rows.infeasible_count, with_rows.infeasible_count);
  EXPECT_EQ(without_rows.over_worked_count, with_rows.over_worked_count);
  EXPECT_EQ(without_rows.first_bad_index, with_rows.first_bad_index);
  EXPECT_DOUBLE_EQ(without_rows.worst_bend_utilisation, with_rows.worst_bend_utilisation);
}

/** The reversal count is the fatigue signal that decides which link is reported as wearing out
 *  first, so it has to respond to the motion actually reversing.
 *
 *  Note what this test deliberately does NOT assert: that a one-way joint sweep produces zero
 *  reversals. It does not. Sweeping one joint monotonically does not make the curvature at each
 *  individual rod segment monotonic -- as the chord changes the solver redistributes bend along
 *  the chain, and that redistribution crosses the 1e-4 direction dead-band many times. Measured on
 *  this fixture, a 20-point one-way sweep already reports ~34 reversals summed over 23 segments.
 *  So the metric carries a real noise floor and should be read as a relative ranking between
 *  segments of one run, not as an absolute flex-cycle count. That behaviour predates this
 *  refactor and is preserved exactly; it is recorded here so nobody "fixes" the number later
 *  without realising the CLI reports the same one. */
TEST(TrajectoryAnalysis, ReversalsRespondToTheMotionActuallyReversing)
{
  auto model = makeModel();
  CarrierTrajectoryAnalyzer analyzer(makeParams(), model);

  const auto monotonic = analyzer.analyze(sweep(model, 1.2, 2.6, 20), {}, nullptr, {});
  long monotonic_reversals = 0;
  for (const auto& s : monotonic.segments)
  {
    monotonic_reversals += s.reversals;
  }

  // There and back over exactly the same joint range, so the only added physical content is the
  // turn-around itself.
  auto out = sweep(model, 1.2, 2.6, 20);
  const auto back = sweep(model, 2.6, 1.2, 20);
  out.insert(out.end(), back.begin(), back.end());
  const auto round_trip = analyzer.analyze(out, {}, nullptr, {});
  long round_trip_reversals = 0;
  for (const auto& s : round_trip.segments)
  {
    round_trip_reversals += s.reversals;
  }

  EXPECT_GE(monotonic_reversals, 0);
  EXPECT_GT(round_trip_reversals, monotonic_reversals)
      << "reversing the motion must raise the reversal count above the one-way baseline";
  EXPECT_TRUE(round_trip.has_hottest_segment);
  EXPECT_FALSE(round_trip.segments.empty());
}

TEST(TrajectoryAnalysis, CancelStopsEarly)
{
  auto model = makeModel();
  CarrierTrajectoryAnalyzer analyzer(makeParams(), model);
  const auto states = sweep(model, 1.0, 2.6, 64);

  std::atomic_bool cancel{ true };  // already set: the very first stride check must bail
  CarrierTrajectoryAnalyzer::Options options;
  options.cancel = &cancel;
  options.cancel_check_stride = 16;

  const auto report = analyzer.analyze(states, {}, nullptr, options);
  EXPECT_TRUE(report.waypoints.empty());
}

TEST(TrajectoryAnalysis, VerdictWording)
{
  CarrierTrajectoryReport clean;
  clean.bend_utilisation_warn = 0.8;
  EXPECT_TRUE(clean.safe());
  EXPECT_EQ(clean.verdict(), "carrier stays within its bend limit and clear of collisions.");

  CarrierTrajectoryReport warned = clean;
  warned.over_worked_count = 3;
  EXPECT_TRUE(warned.safe()) << "over-worked is a warning, not a failure";
  EXPECT_EQ(warned.verdict(), "reachable, but the carrier runs tighter than 80% of R_min on 3 waypoints.");

  CarrierTrajectoryReport failed = clean;
  failed.infeasible_count = 1;
  EXPECT_FALSE(failed.safe());
  EXPECT_EQ(failed.verdict(), "this trajectory is NOT safe for the carrier as mounted.");

  CarrierTrajectoryReport cable = clean;
  cable.cable_violation_count = 2;
  EXPECT_FALSE(cable.safe()) << "a cable over its own bend limit must fail the run";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
