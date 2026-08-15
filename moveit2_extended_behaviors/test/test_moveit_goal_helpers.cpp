// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// retargetPose is the single most load-bearing piece of arithmetic in this project.
//
// On the CRX-5iA the SRDF names `cutting_point` as the planning group's tip, but
// crx_kinematics always solves to `flange` and ignores both the SRDF tip and ik_link_name. A
// Behavior that constrains the tip directly puts the FLANGE there instead, so the tool lands
// 150 mm away -- and nothing errors, because the solver thinks it succeeded. These tests pin the
// correction by round-tripping through forward kinematics: place the tool where we asked, ask for
// the flange pose, put the flange there, and check the tool came out where we wanted.
#include <moveit2_extended_behaviors/moveit_goal_helpers.hpp>

#include <gtest/gtest.h>
#include <moveit/robot_model/robot_model.h>
#include <srdfdom/model.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <urdf_parser/urdf_parser.h>

using moveit2_extended::behaviors::finalStateOf;
using moveit2_extended::behaviors::makeJointGoal;
using moveit2_extended::behaviors::resolveTargetState;
using moveit2_extended::behaviors::retargetPose;

namespace
{
/** A 2R arm ending in a rigid tool: link2 -> flange -> tool_tip, with the tool offset chosen to
 *  mirror the real cutter's (0.150, 0, 0.028). `wrist` is deliberately reachable through a movable
 *  joint from the flange so the rigidity check has a negative case to catch. */
constexpr const char* kUrdf = R"(<?xml version="1.0"?>
<robot name="test_arm">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="flange"/>
  <link name="tool_tip"/>
  <joint name="joint_a" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <joint name="joint_b" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0.30 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <joint name="flange_joint" type="fixed">
    <parent link="link2"/><child link="flange"/>
    <origin xyz="0.20 0 0"/>
  </joint>
  <joint name="tool_joint" type="fixed">
    <parent link="flange"/><child link="tool_tip"/>
    <origin xyz="0.150 0 0.028" rpy="0 0 0"/>
  </joint>
</robot>)";

constexpr const char* kSrdf = R"(<?xml version="1.0"?>
<robot name="test_arm">
  <group name="arm">
    <chain base_link="base_link" tip_link="tool_tip"/>
  </group>
  <group_state name="home" group="arm">
    <joint name="joint_a" value="0.3"/>
    <joint name="joint_b" value="-0.7"/>
  </group_state>
</robot>)";

moveit::core::RobotModelPtr makeModel()
{
  auto urdf_model = urdf::parseURDF(kUrdf);
  EXPECT_TRUE(static_cast<bool>(urdf_model));
  auto srdf_model = std::make_shared<srdf::Model>();
  EXPECT_TRUE(srdf_model->initString(*urdf_model, kSrdf));
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}
}  // namespace

/** The round trip that matters: a pose asked for at the tool must actually be reached BY the tool
 *  once the flange is placed where retargetPose says. */
TEST(MoveItGoalHelpers, RetargetRoundTripsThroughForwardKinematics)
{
  auto model = makeModel();
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  double a = 0.4, b = -0.6;
  state.setJointPositions("joint_a", &a);
  state.setJointPositions("joint_b", &b);
  state.update();

  // Where the tool currently is: use that as the requested target, so the answer is checkable.
  geometry_msgs::msg::PoseStamped wanted;
  wanted.header.frame_id = model->getModelFrame();
  wanted.pose = tf2::toMsg(state.getGlobalLinkTransform("tool_tip"));

  const auto flange_goal = retargetPose(model, wanted, "tool_tip", "flange");
  ASSERT_TRUE(static_cast<bool>(flange_goal)) << flange_goal.error();

  // Placing the flange at the returned pose must put the tool back at `wanted`.
  Eigen::Isometry3d flange_target;
  tf2::fromMsg(flange_goal->pose, flange_target);
  moveit::core::RobotState probe(model);
  probe.setToDefaultValues();
  probe.update();
  const Eigen::Isometry3d flange_to_tool =
      probe.getGlobalLinkTransform("flange").inverse() * probe.getGlobalLinkTransform("tool_tip");
  const Eigen::Isometry3d achieved = flange_target * flange_to_tool;

  Eigen::Isometry3d expected;
  tf2::fromMsg(wanted.pose, expected);
  EXPECT_TRUE(achieved.isApprox(expected, 1e-9))
      << "the tool did not land where it was asked to; retargeting is wrong";
}

/** Without the correction the error is exactly the tool offset -- 152.6 mm here. This test exists
 *  to record the size of the mistake, so nobody decides the correction is a nicety. */
TEST(MoveItGoalHelpers, SkippingTheRetargetMissesByTheToolOffset)
{
  auto model = makeModel();
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.update();

  geometry_msgs::msg::PoseStamped wanted;
  wanted.header.frame_id = model->getModelFrame();
  wanted.pose = tf2::toMsg(state.getGlobalLinkTransform("tool_tip"));

  const auto flange_goal = retargetPose(model, wanted, "tool_tip", "flange");
  ASSERT_TRUE(static_cast<bool>(flange_goal));

  Eigen::Isometry3d corrected, naive;
  tf2::fromMsg(flange_goal->pose, corrected);
  tf2::fromMsg(wanted.pose, naive);
  const double error = (naive.translation() - corrected.translation()).norm();
  EXPECT_NEAR(error, std::hypot(0.150, 0.028), 1e-9)
      << "the uncorrected goal should be off by exactly the tool offset";
}

TEST(MoveItGoalHelpers, RetargetIsIdentityWhenTheLinksMatch)
{
  auto model = makeModel();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.position.x = 0.4;
  pose.pose.orientation.w = 1.0;

  const auto out = retargetPose(model, pose, "flange", "flange");
  ASSERT_TRUE(static_cast<bool>(out));
  EXPECT_DOUBLE_EQ(out->pose.position.x, 0.4);
}

/** A fixed-offset retarget between links separated by a movable joint is meaningless. Returning a
 *  plausible-looking wrong pose there would be worse than refusing. */
TEST(MoveItGoalHelpers, RefusesWhenTheLinksAreNotRigidlyConnected)
{
  auto model = makeModel();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.orientation.w = 1.0;

  const auto out = retargetPose(model, pose, "tool_tip", "link1");
  ASSERT_FALSE(static_cast<bool>(out));
  EXPECT_NE(out.error().find("rigidly connected"), std::string::npos) << out.error();
}

TEST(MoveItGoalHelpers, RetargetRejectsUnknownLinks)
{
  auto model = makeModel();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.orientation.w = 1.0;

  EXPECT_FALSE(static_cast<bool>(retargetPose(model, pose, "no_such_link", "flange")));
  EXPECT_FALSE(static_cast<bool>(retargetPose(model, pose, "tool_tip", "no_such_link")));
  EXPECT_FALSE(static_cast<bool>(retargetPose(nullptr, pose, "tool_tip", "flange")));
}

TEST(MoveItGoalHelpers, ResolvesAnSrdfNamedState)
{
  auto model = makeModel();
  moveit::core::RobotState reference(model);
  reference.setToDefaultValues();

  const auto state = resolveTargetState(model, reference, "arm", "home", sensor_msgs::msg::JointState{});
  ASSERT_TRUE(static_cast<bool>(state)) << state.error();
  EXPECT_NEAR(state->getVariablePosition("joint_a"), 0.3, 1e-9);
  EXPECT_NEAR(state->getVariablePosition("joint_b"), -0.7, 1e-9);
}

TEST(MoveItGoalHelpers, UnknownNamedStateIsReported)
{
  auto model = makeModel();
  moveit::core::RobotState reference(model);
  reference.setToDefaultValues();

  const auto state = resolveTargetState(model, reference, "arm", "not_a_pose", sensor_msgs::msg::JointState{});
  ASSERT_FALSE(static_cast<bool>(state));
  EXPECT_NE(state.error().find("not_a_pose"), std::string::npos);
}

/** Named joints are order independent, and joints the caller did not mention keep their current
 *  values rather than snapping to zero -- which on the real arm is a singularity. */
TEST(MoveItGoalHelpers, NamedJointsAreOrderIndependentAndPartial)
{
  auto model = makeModel();
  moveit::core::RobotState reference(model);
  reference.setToDefaultValues();
  double b = -0.9;
  reference.setJointPositions("joint_b", &b);
  reference.update();

  sensor_msgs::msg::JointState joints;
  joints.name = { "joint_a" };
  joints.position = { 0.55 };

  const auto state = resolveTargetState(model, reference, "arm", "", joints);
  ASSERT_TRUE(static_cast<bool>(state)) << state.error();
  EXPECT_NEAR(state->getVariablePosition("joint_a"), 0.55, 1e-9);
  EXPECT_NEAR(state->getVariablePosition("joint_b"), -0.9, 1e-9) << "unmentioned joints must be left alone";
}

TEST(MoveItGoalHelpers, UnnamedJointsMustMatchTheGroupExactly)
{
  auto model = makeModel();
  moveit::core::RobotState reference(model);
  reference.setToDefaultValues();

  sensor_msgs::msg::JointState right;
  right.position = { 0.1, 0.2 };
  EXPECT_TRUE(static_cast<bool>(resolveTargetState(model, reference, "arm", "", right)));

  sensor_msgs::msg::JointState wrong;
  wrong.position = { 0.1 };
  const auto out = resolveTargetState(model, reference, "arm", "", wrong);
  EXPECT_FALSE(static_cast<bool>(out)) << "a short unnamed list must be refused, not padded";
}

TEST(MoveItGoalHelpers, MismatchedNameAndPositionCountsAreRefused)
{
  auto model = makeModel();
  moveit::core::RobotState reference(model);
  reference.setToDefaultValues();

  sensor_msgs::msg::JointState joints;
  joints.name = { "joint_a", "joint_b" };
  joints.position = { 0.1 };
  EXPECT_FALSE(static_cast<bool>(resolveTargetState(model, reference, "arm", "", joints)));
}

TEST(MoveItGoalHelpers, JointGoalConstraintsCoverTheGroup)
{
  auto model = makeModel();
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.update();

  const auto constraints = makeJointGoal(state, "arm", 0.01);
  ASSERT_TRUE(static_cast<bool>(constraints)) << constraints.error();
  EXPECT_EQ(constraints->joint_constraints.size(), 2u);

  EXPECT_FALSE(static_cast<bool>(makeJointGoal(state, "no_such_group", 0.01)));
}

TEST(MoveItGoalHelpers, FinalStateOfAnEmptyTrajectoryIsRefused)
{
  auto model = makeModel();
  moveit::core::RobotState reference(model);
  reference.setToDefaultValues();

  moveit_msgs::msg::RobotTrajectory empty;
  EXPECT_FALSE(static_cast<bool>(finalStateOf(model, reference, empty)));
}

/** end_state -> start_state chaining is what replaces MoveIt Task Constructor for plan-ahead, so
 *  the last waypoint has to come back exactly. */
TEST(MoveItGoalHelpers, FinalStateOfReturnsTheLastWaypoint)
{
  auto model = makeModel();
  moveit::core::RobotState reference(model);
  reference.setToDefaultValues();
  reference.update();

  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = { "joint_a", "joint_b" };
  trajectory_msgs::msg::JointTrajectoryPoint first;
  first.positions = { 0.0, 0.0 };
  first.time_from_start.sec = 0;
  trajectory_msgs::msg::JointTrajectoryPoint last;
  last.positions = { 0.25, -0.45 };
  last.time_from_start.sec = 1;
  trajectory.joint_trajectory.points = { first, last };

  const auto final_state = finalStateOf(model, reference, trajectory);
  ASSERT_TRUE(static_cast<bool>(final_state)) << final_state.error();

  double a = 0.0, b = 0.0;
  for (size_t i = 0; i < final_state->joint_state.name.size(); ++i)
  {
    if (final_state->joint_state.name[i] == "joint_a")
    {
      a = final_state->joint_state.position[i];
    }
    if (final_state->joint_state.name[i] == "joint_b")
    {
      b = final_state->joint_state.position[i];
    }
  }
  EXPECT_NEAR(a, 0.25, 1e-9);
  EXPECT_NEAR(b, -0.45, 1e-9);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
