// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_behaviors/port_conversions.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <sstream>
#include <stdexcept>

namespace moveit2_extended::behaviors
{

std::vector<std::string> splitAndTrim(const std::string& text, char separator)
{
  std::vector<std::string> out;
  std::stringstream stream(text);
  std::string field;
  while (std::getline(stream, field, separator))
  {
    const size_t begin = field.find_first_not_of(" \t\r\n");
    const size_t end = field.find_last_not_of(" \t\r\n");
    out.push_back(begin == std::string::npos ? "" : field.substr(begin, end - begin + 1));
  }
  return out;
}

}  // namespace moveit2_extended::behaviors

namespace
{
using moveit2_extended::behaviors::splitAndTrim;

double toDouble(const std::string& text, const std::string& context)
{
  try
  {
    return std::stod(text);
  }
  catch (const std::exception&)
  {
    throw BT::RuntimeError(context + ": '" + text + "' is not a number");
  }
}

/** Fill orientation from either 3 (RPY) or 4 (quaternion) trailing numbers. */
void setOrientation(geometry_msgs::msg::Quaternion& out, const std::vector<std::string>& fields, size_t first,
                    const std::string& context)
{
  const size_t remaining = fields.size() - first;
  if (remaining == 3)
  {
    tf2::Quaternion q;
    q.setRPY(toDouble(fields[first], context), toDouble(fields[first + 1], context),
             toDouble(fields[first + 2], context));
    q.normalize();
    out.x = q.x();
    out.y = q.y();
    out.z = q.z();
    out.w = q.w();
  }
  else if (remaining == 4)
  {
    tf2::Quaternion q(toDouble(fields[first], context), toDouble(fields[first + 1], context),
                      toDouble(fields[first + 2], context), toDouble(fields[first + 3], context));
    if (q.length2() < 1e-9)
    {
      throw BT::RuntimeError(context + ": the quaternion is zero-length");
    }
    q.normalize();
    out.x = q.x();
    out.y = q.y();
    out.z = q.z();
    out.w = q.w();
  }
  else
  {
    throw BT::RuntimeError(context + ": expected 3 numbers for roll;pitch;yaw or 4 for a quaternion, got " +
                           std::to_string(remaining));
  }
}

[[noreturn]] void blackboardOnly(const char* type_name)
{
  throw BT::RuntimeError(std::string("a ") + type_name +
                         " port cannot be written as a literal in the XML; bind it to a blackboard "
                         "entry instead, e.g. trajectory=\"{planned_trajectory}\"");
}
}  // namespace

namespace BT
{

template <>
std::vector<std::string> convertFromString<std::vector<std::string>>(StringView str)
{
  return splitAndTrim(std::string(str), ';');
}

template <>
sensor_msgs::msg::JointState convertFromString<sensor_msgs::msg::JointState>(StringView str)
{
  const std::string text(str);
  const auto fields = splitAndTrim(text, ';');
  sensor_msgs::msg::JointState state;
  if (fields.empty() || (fields.size() == 1 && fields.front().empty()))
  {
    return state;
  }

  const bool named = text.find('=') != std::string::npos;
  for (const auto& field : fields)
  {
    if (field.empty())
    {
      continue;
    }
    if (named)
    {
      const size_t equals = field.find('=');
      if (equals == std::string::npos)
      {
        throw BT::RuntimeError("JointState: mixed named and unnamed entries near '" + field +
                               "'; use either 'J1=0.0;J2=1.0' or '0.0;1.0' throughout");
      }
      state.name.push_back(field.substr(0, equals));
      state.position.push_back(toDouble(field.substr(equals + 1), "JointState"));
    }
    else
    {
      state.position.push_back(toDouble(field, "JointState"));
    }
  }
  return state;
}

template <>
geometry_msgs::msg::PoseStamped convertFromString<geometry_msgs::msg::PoseStamped>(StringView str)
{
  const auto fields = splitAndTrim(std::string(str), ';');
  if (fields.size() < 7)
  {
    throw BT::RuntimeError("PoseStamped: expected 'frame_id;x;y;z;roll;pitch;yaw' or "
                           "'frame_id;x;y;z;qx;qy;qz;qw', got " +
                           std::to_string(fields.size()) + " field(s)");
  }
  if (fields[0].empty())
  {
    // Defaulting the frame is how a target silently ends up in the wrong place.
    throw BT::RuntimeError("PoseStamped: frame_id must not be empty");
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = fields[0];
  pose.pose.position.x = toDouble(fields[1], "PoseStamped");
  pose.pose.position.y = toDouble(fields[2], "PoseStamped");
  pose.pose.position.z = toDouble(fields[3], "PoseStamped");
  setOrientation(pose.pose.orientation, fields, 4, "PoseStamped");
  return pose;
}

template <>
geometry_msgs::msg::Pose convertFromString<geometry_msgs::msg::Pose>(StringView str)
{
  const auto fields = splitAndTrim(std::string(str), ';');
  if (fields.size() < 6)
  {
    throw BT::RuntimeError("Pose: expected 'x;y;z;roll;pitch;yaw' or 'x;y;z;qx;qy;qz;qw', got " +
                           std::to_string(fields.size()) + " field(s)");
  }
  geometry_msgs::msg::Pose pose;
  pose.position.x = toDouble(fields[0], "Pose");
  pose.position.y = toDouble(fields[1], "Pose");
  pose.position.z = toDouble(fields[2], "Pose");
  setOrientation(pose.orientation, fields, 3, "Pose");
  return pose;
}

template <>
geometry_msgs::msg::Vector3 convertFromString<geometry_msgs::msg::Vector3>(StringView str)
{
  const auto fields = splitAndTrim(std::string(str), ';');
  if (fields.size() != 3)
  {
    throw BT::RuntimeError("Vector3: expected 'x;y;z', got " + std::to_string(fields.size()) + " field(s)");
  }
  geometry_msgs::msg::Vector3 vector;
  vector.x = toDouble(fields[0], "Vector3");
  vector.y = toDouble(fields[1], "Vector3");
  vector.z = toDouble(fields[2], "Vector3");
  return vector;
}

template <>
std::vector<geometry_msgs::msg::PoseStamped>
convertFromString<std::vector<geometry_msgs::msg::PoseStamped>>(StringView str)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  for (const auto& element : splitAndTrim(std::string(str), '|'))
  {
    if (element.empty())
    {
      continue;
    }
    poses.push_back(convertFromString<geometry_msgs::msg::PoseStamped>(element));
  }
  return poses;
}

template <>
moveit_msgs::msg::RobotTrajectory convertFromString<moveit_msgs::msg::RobotTrajectory>(StringView)
{
  blackboardOnly("RobotTrajectory");
}

template <>
moveit_msgs::msg::RobotState convertFromString<moveit_msgs::msg::RobotState>(StringView)
{
  blackboardOnly("RobotState");
}

template <>
moveit_msgs::msg::PlanningScene convertFromString<moveit_msgs::msg::PlanningScene>(StringView)
{
  blackboardOnly("PlanningScene");
}

}  // namespace BT
