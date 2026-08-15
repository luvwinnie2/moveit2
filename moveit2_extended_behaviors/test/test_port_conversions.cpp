// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// Without these conversions an XML attribute for a ROS-message port throws at tree-build time.
// With a sloppy one, it silently parses to the wrong pose -- which is worse. Hence the emphasis
// below on what must be REFUSED.
#include <moveit2_extended_behaviors/port_conversions.hpp>

#include <gtest/gtest.h>

#include <cmath>

TEST(PortConversions, StringVector)
{
  const auto out = BT::convertFromString<std::vector<std::string>>("alpha; beta ;gamma");
  ASSERT_EQ(out.size(), 3u);
  EXPECT_EQ(out[0], "alpha");
  EXPECT_EQ(out[1], "beta") << "surrounding whitespace must be trimmed";
  EXPECT_EQ(out[2], "gamma");
}

TEST(PortConversions, NamedJointState)
{
  const auto out = BT::convertFromString<sensor_msgs::msg::JointState>("J1=0.0;J2=-1.0472;J3=0.5");
  ASSERT_EQ(out.name.size(), 3u);
  ASSERT_EQ(out.position.size(), 3u);
  EXPECT_EQ(out.name[1], "J2");
  EXPECT_NEAR(out.position[1], -1.0472, 1e-9);
}

TEST(PortConversions, UnnamedJointState)
{
  const auto out = BT::convertFromString<sensor_msgs::msg::JointState>("0.1;0.2;0.3");
  EXPECT_TRUE(out.name.empty());
  ASSERT_EQ(out.position.size(), 3u);
  EXPECT_NEAR(out.position[2], 0.3, 1e-9);
}

/** Half-named is ambiguous: which value belongs to which joint? Guessing here would put the arm
 *  somewhere nobody asked for. */
TEST(PortConversions, MixedNamedAndUnnamedIsRefused)
{
  EXPECT_THROW(BT::convertFromString<sensor_msgs::msg::JointState>("J1=0.0;0.5"), BT::RuntimeError);
}

TEST(PortConversions, NonNumericJointValueIsRefused)
{
  EXPECT_THROW(BT::convertFromString<sensor_msgs::msg::JointState>("J1=abc"), BT::RuntimeError);
}

TEST(PortConversions, PoseStampedWithRpy)
{
  const auto out = BT::convertFromString<geometry_msgs::msg::PoseStamped>("robot_base;0.5;0.1;0.6;0;1.5707963;0");
  EXPECT_EQ(out.header.frame_id, "robot_base");
  EXPECT_NEAR(out.pose.position.x, 0.5, 1e-9);
  EXPECT_NEAR(out.pose.position.z, 0.6, 1e-9);
  // A 90 deg pitch is (0, sqrt(2)/2, 0, sqrt(2)/2).
  EXPECT_NEAR(out.pose.orientation.y, std::sqrt(2.0) / 2.0, 1e-6);
  EXPECT_NEAR(out.pose.orientation.w, std::sqrt(2.0) / 2.0, 1e-6);
}

TEST(PortConversions, PoseStampedWithQuaternion)
{
  const auto out = BT::convertFromString<geometry_msgs::msg::PoseStamped>("world;1;2;3;0;0.7071;0;0.7071");
  EXPECT_EQ(out.header.frame_id, "world");
  EXPECT_NEAR(out.pose.position.y, 2.0, 1e-9);
  EXPECT_NEAR(out.pose.orientation.y, std::sqrt(2.0) / 2.0, 1e-4);
}

/** A pose with no frame is not a pose. Defaulting it to the planning frame is how a target ends up
 *  somewhere unexpected with nothing in the logs. */
TEST(PortConversions, PoseStampedWithoutAFrameIsRefused)
{
  EXPECT_THROW(BT::convertFromString<geometry_msgs::msg::PoseStamped>(";0.5;0.1;0.6;0;0;0"), BT::RuntimeError);
}

TEST(PortConversions, PoseStampedWithWrongFieldCountIsRefused)
{
  EXPECT_THROW(BT::convertFromString<geometry_msgs::msg::PoseStamped>("world;0.5;0.1"), BT::RuntimeError);
  EXPECT_THROW(BT::convertFromString<geometry_msgs::msg::PoseStamped>("world;1;2;3;0;0"), BT::RuntimeError);
}

TEST(PortConversions, ZeroLengthQuaternionIsRefused)
{
  EXPECT_THROW(BT::convertFromString<geometry_msgs::msg::PoseStamped>("world;1;2;3;0;0;0;0"), BT::RuntimeError);
}

TEST(PortConversions, PoseWithoutFrame)
{
  const auto out = BT::convertFromString<geometry_msgs::msg::Pose>("0.1;0.2;0.3;0;0;0");
  EXPECT_NEAR(out.position.x, 0.1, 1e-9);
  EXPECT_NEAR(out.orientation.w, 1.0, 1e-9);
}

TEST(PortConversions, Vector3)
{
  const auto out = BT::convertFromString<geometry_msgs::msg::Vector3>("1;-2;3.5");
  EXPECT_NEAR(out.x, 1.0, 1e-9);
  EXPECT_NEAR(out.y, -2.0, 1e-9);
  EXPECT_NEAR(out.z, 3.5, 1e-9);
  EXPECT_THROW(BT::convertFromString<geometry_msgs::msg::Vector3>("1;2"), BT::RuntimeError);
}

TEST(PortConversions, PoseStampedList)
{
  const auto out = BT::convertFromString<std::vector<geometry_msgs::msg::PoseStamped>>(
      "base;0;0;0;0;0;0 | base;0.1;0;0;0;0;0 | base;0.2;0;0;0;0;0");
  ASSERT_EQ(out.size(), 3u);
  EXPECT_NEAR(out[2].pose.position.x, 0.2, 1e-9);
  EXPECT_EQ(out[1].header.frame_id, "base");
}

/** Leaving these undefined would produce an unreadable std::bad_any_cast at tick time. A throw
 *  that says what to do instead is the point. */
TEST(PortConversions, BlackboardOnlyTypesExplainThemselves)
{
  try
  {
    BT::convertFromString<moveit_msgs::msg::RobotTrajectory>("anything");
    FAIL() << "a RobotTrajectory literal should not be accepted";
  }
  catch (const BT::RuntimeError& error)
  {
    const std::string message = error.what();
    EXPECT_NE(message.find("blackboard"), std::string::npos)
        << "the error must say what to do instead, got: " << message;
  }

  EXPECT_THROW(BT::convertFromString<moveit_msgs::msg::RobotState>("x"), BT::RuntimeError);
  EXPECT_THROW(BT::convertFromString<moveit_msgs::msg::PlanningScene>("x"), BT::RuntimeError);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
