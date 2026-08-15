// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

// String -> port-type conversions.
//
// BehaviorTree.CPP ships convertFromString for int, double, bool, std::string and
// std::vector<int|double>. It has none for std::vector<std::string> and none for any ROS message.
// Without the specialisations below, an XML attribute for such a port throws at tree-build time
// with a message that names the type but not the port.
//
// Two categories of port type:
//
//   literal-able   a human can reasonably write it in the XML: a pose, a joint state, a list of
//                  names. These get a real parser.
//   blackboard-only  a trajectory, a planning scene, a robot state. Nobody types those by hand;
//                  they are produced by one Behavior and consumed by another. These get a
//                  conversion that throws a clear explanation, because leaving them undefined
//                  produces an unreadable std::bad_any_cast at tick time instead.

#include <behaviortree_cpp_v3/basic_types.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <string>
#include <vector>

namespace BT
{

/** "a;b;c" -- semicolon separated, matching BehaviorTree.CPP's own vector<int>/vector<double>. */
template <>
std::vector<std::string> convertFromString<std::vector<std::string>>(StringView str);

/** Either
 *    "J1=0.0;J2=-1.0472;J3=0.5"     names and values together, order independent
 *  or
 *    "0.0;-1.0472;0.5"              values only; the caller must supply the names elsewhere
 *
 *  The named form is strongly preferred in Objective XML: a bare list silently means something
 *  different the day someone reorders a planning group. */
template <>
sensor_msgs::msg::JointState convertFromString<sensor_msgs::msg::JointState>(StringView str);

/** "frame_id;x;y;z;roll;pitch;yaw"        6 numbers  -> RPY, in radians
 *  "frame_id;x;y;z;qx;qy;qz;qw"           7 numbers  -> quaternion
 *
 *  The frame is mandatory. A pose without a frame is not a pose, and defaulting it to the planning
 *  frame is how a target ends up somewhere nobody asked for. */
template <>
geometry_msgs::msg::PoseStamped convertFromString<geometry_msgs::msg::PoseStamped>(StringView str);

/** Same as PoseStamped but with no frame field: "x;y;z;roll;pitch;yaw" or the 7-number form. */
template <>
geometry_msgs::msg::Pose convertFromString<geometry_msgs::msg::Pose>(StringView str);

/** "x;y;z" */
template <>
geometry_msgs::msg::Vector3 convertFromString<geometry_msgs::msg::Vector3>(StringView str);

/** Several PoseStamped, separated by "|". Each element uses the PoseStamped syntax above. */
template <>
std::vector<geometry_msgs::msg::PoseStamped>
convertFromString<std::vector<geometry_msgs::msg::PoseStamped>>(StringView str);

// ---- blackboard-only: these throw, on purpose --------------------------------------------------
template <>
moveit_msgs::msg::RobotTrajectory convertFromString<moveit_msgs::msg::RobotTrajectory>(StringView str);
template <>
moveit_msgs::msg::RobotState convertFromString<moveit_msgs::msg::RobotState>(StringView str);
template <>
moveit_msgs::msg::PlanningScene convertFromString<moveit_msgs::msg::PlanningScene>(StringView str);

}  // namespace BT

namespace moveit2_extended::behaviors
{

/** Split on `separator`, trimming whitespace around each field. Exposed because several Behaviors
 *  parse their own compound strings and should do it the same way. */
std::vector<std::string> splitAndTrim(const std::string& text, char separator);

}  // namespace moveit2_extended::behaviors
