// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_tools/tool_registry.hpp>

#include <moveit2_extended_core/path_utils.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

namespace moveit2_extended::tools
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

geometry_msgs::msg::Pose parsePose(const YAML::Node& node)
{
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  if (!node || !node.IsMap())
  {
    return pose;
  }
  if (const YAML::Node xyz = node["xyz"]; xyz && xyz.IsSequence() && xyz.size() == 3)
  {
    pose.position.x = xyz[0].as<double>();
    pose.position.y = xyz[1].as<double>();
    pose.position.z = xyz[2].as<double>();
  }
  if (const YAML::Node rpy = node["rpy"]; rpy && rpy.IsSequence() && rpy.size() == 3)
  {
    tf2::Quaternion q;
    q.setRPY(rpy[0].as<double>(), rpy[1].as<double>(), rpy[2].as<double>());
    q.normalize();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
  }
  return pose;
}

bool parseShape(const YAML::Node& node, ToolShape& out, std::string* error)
{
  const std::string type = node["type"] ? node["type"].as<std::string>() : "";
  std::vector<double> dimensions;
  if (const YAML::Node dims = node["dimensions"]; dims && dims.IsSequence())
  {
    for (const auto& value : dims)
    {
      dimensions.push_back(value.as<double>());
    }
  }

  size_t expected = 0;
  if (type == "box")
  {
    out.primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    expected = 3;
  }
  else if (type == "cylinder")
  {
    out.primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    expected = 2;
  }
  else if (type == "sphere")
  {
    out.primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    expected = 1;
  }
  else
  {
    setError(error, "unknown shape type '" + type + "'; use box, cylinder or sphere");
    return false;
  }

  if (dimensions.size() != expected)
  {
    setError(error, "a " + type + " needs " + std::to_string(expected) + " dimension(s), got " +
                        std::to_string(dimensions.size()));
    return false;
  }
  for (double dimension : dimensions)
  {
    if (!(dimension > 0.0))
    {
      // A zero-sized collision shape is worse than none: it registers as a shape and collides with
      // nothing, so the planner cheerfully drives the tool through the vines.
      setError(error, "every dimension must be greater than zero");
      return false;
    }
  }

  out.primitive.dimensions.resize(dimensions.size());
  std::copy(dimensions.begin(), dimensions.end(), out.primitive.dimensions.begin());
  out.pose = parsePose(node["origin"]);
  return true;
}
}  // namespace

bool ToolRegistry::loadFromText(const std::string& yaml_text, std::string* error)
{
  setError(error, {});
  YAML::Node root;
  try
  {
    root = YAML::Load(yaml_text);
  }
  catch (const YAML::Exception& exc)
  {
    setError(error, std::string("cannot parse the tool registry: ") + exc.what());
    return false;
  }
  if (!root || !root.IsMap())
  {
    setError(error, "the tool registry must be a map");
    return false;
  }

  // Parsed into a temporary so a bad entry cannot leave the registry holding half of one
  // configuration and half of another.
  std::map<std::string, Tool> parsed;
  const std::string mount = root["mount_link"] ? root["mount_link"].as<std::string>() : "";
  if (mount.empty())
  {
    setError(error, "mount_link is required: without it, a tool has nothing to be attached to");
    return false;
  }

  const YAML::Node list = root["tools"];
  if (!list || !list.IsSequence())
  {
    setError(error, "the registry needs a 'tools' sequence");
    return false;
  }

  for (const auto& node : list)
  {
    if (!node.IsMap() || !node["name"])
    {
      setError(error, "every tool needs a name");
      return false;
    }
    Tool tool;
    tool.info.name = node["name"].as<std::string>();
    tool.info.description = node["description"] ? node["description"].as<std::string>() : "";
    tool.info.package_name = node["package_name"] ? node["package_name"].as<std::string>() : "";
    tool.info.urdf_file_path = node["urdf_file_path"] ? node["urdf_file_path"].as<std::string>() : "";
    tool.info.carrier_config = node["carrier_config"] ? node["carrier_config"].as<std::string>() : "";
    tool.info.mass = node["mass"] ? node["mass"].as<double>() : 0.0;
    tool.provenance = node["provenance"] ? node["provenance"].as<std::string>() : "estimate";

    tool.info.tcp_offset = parsePose(node["tcp_offset"]);

    if (const YAML::Node allowed = node["allowed_collision_links"]; allowed && allowed.IsSequence())
    {
      for (const auto& link : allowed)
      {
        tool.info.allowed_collision_links.push_back(link.as<std::string>());
      }
    }

    if (const YAML::Node shapes = node["collision"]; shapes && shapes.IsSequence())
    {
      for (const auto& shape_node : shapes)
      {
        ToolShape shape;
        std::string shape_error;
        if (!parseShape(shape_node, shape, &shape_error))
        {
          setError(error, "tool '" + tool.info.name + "': " + shape_error);
          return false;
        }
        tool.shapes.push_back(std::move(shape));
      }
    }
    if (tool.shapes.empty())
    {
      // A tool with no collision geometry would be invisible to the planner, which would then
      // happily plan straight through whatever it is holding.
      setError(error, "tool '" + tool.info.name + "' declares no collision shapes");
      return false;
    }

    if (parsed.count(tool.info.name))
    {
      setError(error, "duplicate tool name '" + tool.info.name + "'");
      return false;
    }
    parsed[tool.info.name] = std::move(tool);
  }

  tools_ = std::move(parsed);
  mount_link = mount;
  return true;
}

bool ToolRegistry::loadFromFile(const std::string& path, std::string* error)
{
  setError(error, {});
  std::string read_error;
  const std::string text = readFile(path, &read_error);
  if (text.empty())
  {
    setError(error, read_error.empty() ? ("tool registry is empty: " + path) : read_error);
    return false;
  }
  return loadFromText(text, error);
}

const Tool* ToolRegistry::find(const std::string& name) const
{
  const auto it = tools_.find(name);
  return it == tools_.end() ? nullptr : &it->second;
}

std::vector<ToolInfo> ToolRegistry::list() const
{
  std::vector<ToolInfo> out;
  out.reserve(tools_.size());
  for (const auto& entry : tools_)
  {
    out.push_back(entry.second.info);
  }
  return out;
}

std::vector<std::string> ToolRegistry::names() const
{
  std::vector<std::string> out;
  out.reserve(tools_.size());
  for (const auto& entry : tools_)
  {
    out.push_back(entry.first);
  }
  return out;
}

}  // namespace moveit2_extended::tools
