// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// These check the parts that can be checked without a robot: that the Behaviors register, that the
// analyser cache does not hand the same non-thread-safe analyser to two callers by accident, and
// that a report converts to its message form without losing anything that a decision depends on.
//
// What is NOT covered here, and is covered by the workspace's end-to-end tests instead: whether
// the numbers are right. That is the job of moveit_cable_carrier's own tests and of the CLI parity
// check, and duplicating it here would just be a second place to update.
#include <moveit2_extended_carrier/carrier_behaviors.hpp>

#include <moveit2_extended_core/behavior_loader_base.hpp>

#include <gtest/gtest.h>
#include <moveit_cable_carrier/carrier_registry.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

using moveit2_extended::BehaviorContext;
using moveit2_extended::BehaviorContextConfig;
using moveit2_extended::BehaviorContextPtr;
using moveit2_extended::BehaviorLoaderBase;
using moveit2_extended::BehaviorParameterMap;
using moveit2_extended::BtFactory;
using moveit2_extended::carrier::AnalyzerCache;
using moveit_cable_carrier::CarrierParams;
using moveit_cable_carrier::CarrierRegistry;

namespace
{
constexpr const char* kUrdf = R"(<?xml version="1.0"?>
<robot name="test_arm">
  <link name="base_link">
    <collision><geometry><box size="0.05 0.05 0.05"/></geometry></collision>
  </link>
  <link name="link1">
    <collision><origin xyz="0.15 0 0" rpy="0 1.5707963 0"/>
      <geometry><cylinder radius="0.03" length="0.30"/></geometry></collision>
  </link>
  <joint name="joint_a" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
</robot>)";

constexpr const char* kSrdf = R"(<?xml version="1.0"?>
<robot name="test_arm"><group name="arm"><chain base_link="base_link" tip_link="link1"/></group></robot>)";

moveit::core::RobotModelPtr makeModel()
{
  auto urdf_model = urdf::parseURDF(kUrdf);
  EXPECT_TRUE(static_cast<bool>(urdf_model));
  auto srdf_model = std::make_shared<srdf::Model>();
  EXPECT_TRUE(srdf_model->initString(*urdf_model, kSrdf));
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

CarrierParams makeCarrier(const std::string& name)
{
  CarrierParams p;
  p.name = name;
  p.base_link = "base_link";
  p.tip_link = "link1";
  p.length = 0.42;
  p.bend_radius = 0.04;
  p.outer_height = 0.016;
  p.outer_width = 0.026;
  p.num_segments = 16;
  p.gravity_sag = 0.0;  // deterministic
  p.bend_utilisation_warn = 0.8;
  p.tip_mount = Eigen::Isometry3d::Identity();
  p.tip_mount.translation() = Eigen::Vector3d(0.25, 0.0, 0.0);
  return p;
}
}  // namespace

TEST(CarrierBehaviors, LoaderRegistersThroughPluginlib)
{
  pluginlib::ClassLoader<BehaviorLoaderBase> class_loader("moveit2_extended_core",
                                                          "moveit2_extended::BehaviorLoaderBase");
  auto node = std::make_shared<rclcpp::Node>("test_carrier_behaviors");
  BehaviorContextConfig config;
  config.start_planning_scene_monitor = false;
  auto context = std::make_shared<BehaviorContext>(node, config, BehaviorParameterMap{});

  BtFactory factory;
  auto loader = class_loader.createSharedInstance("moveit2_extended::carrier::CarrierBehaviorsLoader");
  loader->registerBehaviors(factory, context);

  for (const char* name : { "ValidateCarrierAlongTrajectory", "IsCarrierFeasible", "GetCarrierDiagnostics",
                            "SwitchCarrierType" })
  {
    EXPECT_TRUE(factory.builders().count(name)) << name << " was not registered";
    ASSERT_TRUE(factory.manifests().count(name));
    EXPECT_FALSE(factory.manifests().at(name).ports.empty()) << name << " declares no ports";
  }
}

/** Analysers keep a warm start and are NOT thread safe, so the cache must return the same instance
 *  for the same carrier rather than quietly building a second one each call. */
TEST(CarrierBehaviors, AnalyzerCacheReusesInstances)
{
  auto model = makeModel();
  CarrierRegistry::instance().setCarriers({ makeCarrier("one") });
  AnalyzerCache::instance().invalidate();

  const auto first = AnalyzerCache::instance().get(model, {});
  const auto second = AnalyzerCache::instance().get(model, {});
  ASSERT_EQ(first.size(), 1u);
  ASSERT_EQ(second.size(), 1u);
  EXPECT_EQ(first.front().get(), second.front().get()) << "the cache built a second analyser for the same carrier";
}

TEST(CarrierBehaviors, AnalyzerCacheFiltersByName)
{
  auto model = makeModel();
  CarrierRegistry::instance().setCarriers({ makeCarrier("wrist"), makeCarrier("upper_arm") });
  AnalyzerCache::instance().invalidate();

  EXPECT_EQ(AnalyzerCache::instance().get(model, {}).size(), 2u);

  const auto one = AnalyzerCache::instance().get(model, { "wrist" });
  ASSERT_EQ(one.size(), 1u);
  EXPECT_EQ(one.front()->params().name, "wrist");

  EXPECT_TRUE(AnalyzerCache::instance().get(model, { "no_such_carrier" }).empty());
}

/** A carrier whose mount links are not in the robot model must be dropped, not returned inert.
 *  An inert analyser would report "nothing wrong" for a carrier that was never evaluated -- the
 *  most dangerous possible answer. */
TEST(CarrierBehaviors, InvalidCarriersAreDroppedNotSilentlyPassed)
{
  auto model = makeModel();
  CarrierParams broken = makeCarrier("broken");
  broken.tip_link = "no_such_link";
  CarrierRegistry::instance().setCarriers({ broken });
  AnalyzerCache::instance().invalidate();

  EXPECT_TRUE(AnalyzerCache::instance().get(model, {}).empty())
      << "a carrier with unknown mount links must not be handed out as if it were usable";
}

TEST(CarrierBehaviors, InvalidateForcesARebuild)
{
  auto model = makeModel();
  CarrierRegistry::instance().setCarriers({ makeCarrier("one") });
  AnalyzerCache::instance().invalidate();

  const auto before = AnalyzerCache::instance().get(model, {});
  ASSERT_EQ(before.size(), 1u);

  AnalyzerCache::instance().invalidate();
  const auto after = AnalyzerCache::instance().get(model, {});
  ASSERT_EQ(after.size(), 1u);
  EXPECT_NE(before.front().get(), after.front().get()) << "invalidate() did not force a rebuild";
}

/** Every field a decision could depend on must survive the conversion to the message form.
 *  A dropped `safe` or `verdict` would make an Objective branch on a default. */
TEST(CarrierBehaviors, ReportConvertsWithoutLosingAnything)
{
  moveit_cable_carrier::CarrierTrajectoryReport report;
  report.carrier_name = "wrist";
  report.bend_radius = 0.04;
  report.bend_utilisation_warn = 0.8;
  report.infeasible_count = 2;
  report.colliding_count = 1;
  report.over_worked_count = 5;
  report.cable_violation_count = 3;
  report.worst_bend_utilisation = 0.93;
  report.worst_twist_utilisation = 0.41;
  report.worst_cable_bend_ratio = 5.4;
  report.required_cable_bend_ratio = 10.0;
  report.worst_cable = "cutter_power_signal";
  report.worst_cable_strain = 0.0934;
  report.tightest_radius = 0.0535;
  report.has_tightest_radius = true;
  report.hottest_segment = 22;
  report.hottest_segment_reversals = 8;
  report.has_hottest_segment = true;
  report.first_bad_index = 7;

  moveit_cable_carrier::CarrierWaypointResult waypoint;
  waypoint.index = 7;
  waypoint.time_from_start = 1.25;
  waypoint.collision = true;
  waypoint.feasible = false;
  waypoint.bend_utilisation = 0.93;
  waypoint.cable_bend_ratio = 5.4;
  waypoint.required_cable_bend_ratio = 10.0;
  waypoint.worst_cable = "cutter_power_signal";
  report.waypoints.push_back(waypoint);

  moveit_cable_carrier::CarrierSegmentFatigue segment;
  segment.segment = 22;
  segment.min_curvature = 1.5;
  segment.max_curvature = 18.7;
  segment.reversals = 8;
  report.segments.push_back(segment);

  const auto message = moveit2_extended::carrier::toMsg(report);

  EXPECT_EQ(message.carrier_name, "wrist");
  EXPECT_EQ(message.infeasible_count, 2);
  EXPECT_EQ(message.cable_violation_count, 3);
  EXPECT_DOUBLE_EQ(message.worst_bend_utilisation, 0.93);
  EXPECT_EQ(message.worst_cable, "cutter_power_signal");
  EXPECT_TRUE(message.has_tightest_radius);
  EXPECT_DOUBLE_EQ(message.tightest_radius, 0.0535);
  EXPECT_EQ(message.first_bad_index, 7);
  EXPECT_FALSE(message.safe) << "a report with failures must not convert to safe=true";
  EXPECT_FALSE(message.verdict.empty());

  ASSERT_EQ(message.waypoints.size(), 1u);
  EXPECT_EQ(message.waypoints.front().index, 7u);
  EXPECT_TRUE(message.waypoints.front().collision);
  EXPECT_FALSE(message.waypoints.front().diagnostics.feasible);
  EXPECT_FALSE(message.waypoints.front().diagnostics.cables_within_limit)
      << "5.4x OD against a required 10x is a violation and must be reported as one";

  ASSERT_EQ(message.segments.size(), 1u);
  EXPECT_EQ(message.segments.front().segment, 22u);
  EXPECT_EQ(message.segments.front().reversals, 8);
}

TEST(CarrierBehaviors, ACleanReportConvertsToSafe)
{
  moveit_cable_carrier::CarrierTrajectoryReport report;
  report.carrier_name = "wrist";
  report.bend_utilisation_warn = 0.8;
  const auto message = moveit2_extended::carrier::toMsg(report);
  EXPECT_TRUE(message.safe);
  EXPECT_NE(message.verdict.find("within its bend limit"), std::string::npos) << message.verdict;
}

/** Over-worked is a warning, not a failure -- the Objective decides. A report that turned it into
 *  a failure would make every slightly-tight motion unrunnable. */
TEST(CarrierBehaviors, OverWorkedAloneIsStillSafe)
{
  moveit_cable_carrier::CarrierTrajectoryReport report;
  report.bend_utilisation_warn = 0.8;
  report.over_worked_count = 4;
  const auto message = moveit2_extended::carrier::toMsg(report);
  EXPECT_TRUE(message.safe);
  EXPECT_NE(message.verdict.find("reachable, but"), std::string::npos) << message.verdict;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
