// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_waypoints/waypoint_store.hpp>

#include <moveit2_extended_core/path_utils.hpp>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <sstream>

namespace moveit2_extended::waypoints
{
namespace
{
void setError(std::string* error, std::string message)
{
  if (error)
  {
    *error = std::move(message);
  }
}

std::vector<std::string> stringList(const YAML::Node& node)
{
  std::vector<std::string> out;
  if (node && node.IsSequence())
  {
    for (const auto& item : node)
    {
      out.push_back(item.as<std::string>());
    }
  }
  return out;
}

ArmWaypoint parseWaypoint(const YAML::Node& node, const WaypointStore& defaults)
{
  ArmWaypoint waypoint;
  waypoint.name = node["name"].as<std::string>();
  waypoint.group = node["group"] ? node["group"].as<std::string>() : defaults.default_group;
  waypoint.description = node["description"] ? node["description"].as<std::string>() : "";
  waypoint.tags = stringList(node["tags"]);
  waypoint.tool = node["tool"] ? node["tool"].as<std::string>() : "";

  if (const YAML::Node joints = node["joint_state"]; joints && joints.IsMap())
  {
    waypoint.has_joint_state = true;
    waypoint.joint_state.name = stringList(joints["joint_names"]);
    if (const YAML::Node positions = joints["positions"]; positions && positions.IsSequence())
    {
      for (const auto& value : positions)
      {
        waypoint.joint_state.position.push_back(value.as<double>());
      }
    }
  }

  if (const YAML::Node pose = node["pose"]; pose && pose.IsMap())
  {
    waypoint.has_pose = true;
    waypoint.pose.header.frame_id = pose["frame_id"] ? pose["frame_id"].as<std::string>() : defaults.default_frame_id;
    waypoint.pose_link = pose["link"] ? pose["link"].as<std::string>() : defaults.default_pose_link;
    if (const YAML::Node position = pose["position"]; position && position.IsMap())
    {
      waypoint.pose.pose.position.x = position["x"].as<double>();
      waypoint.pose.pose.position.y = position["y"].as<double>();
      waypoint.pose.pose.position.z = position["z"].as<double>();
    }
    if (const YAML::Node orientation = pose["orientation"]; orientation && orientation.IsMap())
    {
      waypoint.pose.pose.orientation.x = orientation["x"].as<double>();
      waypoint.pose.pose.orientation.y = orientation["y"].as<double>();
      waypoint.pose.pose.orientation.z = orientation["z"].as<double>();
      waypoint.pose.pose.orientation.w = orientation["w"].as<double>();
    }
  }

  if (const YAML::Node carrier = node["carrier"]; carrier && carrier.IsMap())
  {
    waypoint.has_carrier = true;
    auto& diagnostics = waypoint.carrier;
    diagnostics.carrier_name = carrier["carrier_name"] ? carrier["carrier_name"].as<std::string>() : "";
    diagnostics.feasible = carrier["feasible"] ? carrier["feasible"].as<bool>() : false;
    diagnostics.min_bend_radius = carrier["min_bend_radius"] ? carrier["min_bend_radius"].as<double>() : 0.0;
    diagnostics.bend_utilisation = carrier["bend_utilisation"] ? carrier["bend_utilisation"].as<double>() : 0.0;
    diagnostics.twist_utilisation = carrier["twist_utilisation"] ? carrier["twist_utilisation"].as<double>() : 0.0;
    diagnostics.cable_bend_ratio = carrier["cable_bend_ratio"] ? carrier["cable_bend_ratio"].as<double>() : 0.0;
    diagnostics.required_cable_bend_ratio =
        carrier["required_cable_bend_ratio"] ? carrier["required_cable_bend_ratio"].as<double>() : 0.0;
    diagnostics.worst_cable = carrier["worst_cable"] ? carrier["worst_cable"].as<std::string>() : "";
    diagnostics.cables_within_limit =
        carrier["cables_within_limit"] ? carrier["cables_within_limit"].as<bool>() : true;
  }
  return waypoint;
}
}  // namespace

bool WaypointStore::loadFromText(const std::string& yaml_text, std::string* error)
{
  setError(error, {});
  if (yaml_text.empty())
  {
    return true;  // an empty file is an empty store, not a failure
  }

  YAML::Node root;
  try
  {
    root = YAML::Load(yaml_text);
  }
  catch (const YAML::Exception& exc)
  {
    setError(error, std::string("cannot parse the waypoint file: ") + exc.what());
    return false;
  }
  if (!root || !root.IsMap())
  {
    setError(error, "the waypoint file must be a map");
    return false;
  }

  // Parse into a temporary first: a file that is half-readable must not leave the store holding
  // half of one set of targets and half of another.
  WaypointStore parsed;
  parsed.default_group = root["default_group"] ? root["default_group"].as<std::string>() : default_group;
  parsed.default_frame_id = root["default_frame_id"] ? root["default_frame_id"].as<std::string>() : default_frame_id;
  parsed.default_pose_link = root["default_pose_link"] ? root["default_pose_link"].as<std::string>() : default_pose_link;
  parsed.robot_name = root["robot"] ? root["robot"].as<std::string>() : robot_name;

  const YAML::Node list = root["waypoints"];
  if (list && list.IsSequence())
  {
    for (const auto& node : list)
    {
      if (!node.IsMap() || !node["name"])
      {
        setError(error, "every waypoint needs a name");
        return false;
      }
      try
      {
        ArmWaypoint waypoint = parseWaypoint(node, parsed);
        if (parsed.waypoints_.count(waypoint.name))
        {
          setError(error, "duplicate waypoint name '" + waypoint.name + "'");
          return false;
        }
        parsed.waypoints_[waypoint.name] = std::move(waypoint);
      }
      catch (const YAML::Exception& exc)
      {
        setError(error, std::string("malformed waypoint: ") + exc.what());
        return false;
      }
    }
  }

  waypoints_ = std::move(parsed.waypoints_);
  default_group = parsed.default_group;
  default_frame_id = parsed.default_frame_id;
  default_pose_link = parsed.default_pose_link;
  robot_name = parsed.robot_name;
  return true;
}

bool WaypointStore::loadFromFile(const std::string& path, std::string* error)
{
  setError(error, {});
  std::string resolve_error;
  const std::string resolved = resolvePackageUri(path, &resolve_error);
  if (resolved.empty())
  {
    setError(error, resolve_error);
    return false;
  }

  std::error_code ec;
  if (!std::filesystem::exists(resolved, ec))
  {
    // Not an error: this is what an empty store looks like before anything has been taught.
    waypoints_.clear();
    return true;
  }

  std::string read_error;
  const std::string text = readFile(resolved, &read_error);
  if (text.empty() && !read_error.empty())
  {
    setError(error, read_error);
    return false;
  }
  return loadFromText(text, error);
}

std::string WaypointStore::toYaml() const
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "version" << YAML::Value << 1;
  if (!robot_name.empty())
  {
    out << YAML::Key << "robot" << YAML::Value << robot_name;
  }
  out << YAML::Key << "default_group" << YAML::Value << default_group;
  if (!default_frame_id.empty())
  {
    out << YAML::Key << "default_frame_id" << YAML::Value << default_frame_id;
  }
  if (!default_pose_link.empty())
  {
    out << YAML::Key << "default_pose_link" << YAML::Value << default_pose_link;
  }

  out << YAML::Key << "waypoints" << YAML::Value << YAML::BeginSeq;
  for (const auto& entry : waypoints_)
  {
    const ArmWaypoint& waypoint = entry.second;
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << waypoint.name;
    out << YAML::Key << "group" << YAML::Value << waypoint.group;
    if (!waypoint.description.empty())
    {
      out << YAML::Key << "description" << YAML::Value << waypoint.description;
    }
    if (!waypoint.tags.empty())
    {
      out << YAML::Key << "tags" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (const auto& tag : waypoint.tags)
      {
        out << tag;
      }
      out << YAML::EndSeq;
    }
    if (!waypoint.tool.empty())
    {
      out << YAML::Key << "tool" << YAML::Value << waypoint.tool;
    }

    if (waypoint.has_joint_state)
    {
      out << YAML::Key << "joint_state" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "joint_names" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (const auto& name : waypoint.joint_state.name)
      {
        out << name;
      }
      out << YAML::EndSeq;
      out << YAML::Key << "positions" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (double position : waypoint.joint_state.position)
      {
        out << position;
      }
      out << YAML::EndSeq;
      out << YAML::EndMap;
    }

    if (waypoint.has_pose)
    {
      out << YAML::Key << "pose" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "frame_id" << YAML::Value << waypoint.pose.header.frame_id;
      out << YAML::Key << "link" << YAML::Value << waypoint.pose_link;
      out << YAML::Key << "position" << YAML::Value << YAML::Flow << YAML::BeginMap
          << YAML::Key << "x" << YAML::Value << waypoint.pose.pose.position.x
          << YAML::Key << "y" << YAML::Value << waypoint.pose.pose.position.y
          << YAML::Key << "z" << YAML::Value << waypoint.pose.pose.position.z << YAML::EndMap;
      out << YAML::Key << "orientation" << YAML::Value << YAML::Flow << YAML::BeginMap
          << YAML::Key << "x" << YAML::Value << waypoint.pose.pose.orientation.x
          << YAML::Key << "y" << YAML::Value << waypoint.pose.pose.orientation.y
          << YAML::Key << "z" << YAML::Value << waypoint.pose.pose.orientation.z
          << YAML::Key << "w" << YAML::Value << waypoint.pose.pose.orientation.w << YAML::EndMap;
      out << YAML::EndMap;
    }

    if (waypoint.has_carrier)
    {
      out << YAML::Key << "carrier" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "carrier_name" << YAML::Value << waypoint.carrier.carrier_name;
      out << YAML::Key << "feasible" << YAML::Value << waypoint.carrier.feasible;
      out << YAML::Key << "min_bend_radius" << YAML::Value << waypoint.carrier.min_bend_radius;
      out << YAML::Key << "bend_utilisation" << YAML::Value << waypoint.carrier.bend_utilisation;
      out << YAML::Key << "twist_utilisation" << YAML::Value << waypoint.carrier.twist_utilisation;
      out << YAML::Key << "cable_bend_ratio" << YAML::Value << waypoint.carrier.cable_bend_ratio;
      out << YAML::Key << "required_cable_bend_ratio" << YAML::Value
          << waypoint.carrier.required_cable_bend_ratio;
      out << YAML::Key << "worst_cable" << YAML::Value << waypoint.carrier.worst_cable;
      out << YAML::Key << "cables_within_limit" << YAML::Value << waypoint.carrier.cables_within_limit;
      out << YAML::EndMap;
    }

    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;

  return std::string(out.c_str()) + "\n";
}

bool WaypointStore::saveToFile(const std::string& path, std::string* error) const
{
  return writeFile(path, toYaml(), error);
}

bool WaypointStore::add(const ArmWaypoint& waypoint, bool overwrite, std::string* error)
{
  setError(error, {});
  if (waypoint.name.empty())
  {
    setError(error, "a waypoint needs a name");
    return false;
  }
  if (!overwrite && waypoints_.count(waypoint.name))
  {
    setError(error, "'" + waypoint.name + "' already exists; pass overwrite to replace it");
    return false;
  }
  waypoints_[waypoint.name] = waypoint;
  return true;
}

bool WaypointStore::remove(const std::string& name, std::string* error)
{
  setError(error, {});
  if (!waypoints_.erase(name))
  {
    setError(error, "no waypoint named '" + name + "'");
    return false;
  }
  return true;
}

bool WaypointStore::rename(const std::string& from, const std::string& to, bool overwrite, std::string* error)
{
  setError(error, {});
  const auto it = waypoints_.find(from);
  if (it == waypoints_.end())
  {
    setError(error, "no waypoint named '" + from + "'");
    return false;
  }
  if (to.empty())
  {
    setError(error, "the new name must not be empty");
    return false;
  }
  if (from == to)
  {
    return true;
  }
  if (!overwrite && waypoints_.count(to))
  {
    setError(error, "'" + to + "' already exists; pass overwrite to replace it");
    return false;
  }

  ArmWaypoint moved = it->second;
  moved.name = to;
  waypoints_.erase(it);
  waypoints_[to] = std::move(moved);
  return true;
}

const ArmWaypoint* WaypointStore::find(const std::string& name) const
{
  const auto it = waypoints_.find(name);
  return it == waypoints_.end() ? nullptr : &it->second;
}

std::vector<ArmWaypoint> WaypointStore::list(const std::string& tag_filter) const
{
  std::vector<ArmWaypoint> out;
  for (const auto& entry : waypoints_)  // std::map keeps this sorted by name
  {
    if (!tag_filter.empty())
    {
      const auto& tags = entry.second.tags;
      if (std::find(tags.begin(), tags.end(), tag_filter) == tags.end())
      {
        continue;
      }
    }
    out.push_back(entry.second);
  }
  return out;
}

}  // namespace moveit2_extended::waypoints
