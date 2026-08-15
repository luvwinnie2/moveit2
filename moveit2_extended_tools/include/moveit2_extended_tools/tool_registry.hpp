// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

// What end-effectors the robot can be fitted with, and what each one means for planning.
//
// HOW A TOOL IS REPRESENTED, and why:
//
// MoveIt cannot change a robot description at run time. An SRDF group's tip link is fixed when
// move_group starts, and restarting move_group to swap a gripper is not a tool change, it is an
// outage. So a tool is an ATTACHED COLLISION OBJECT on the mount link, plus a TCP offset that the
// Cartesian Behaviors compose away before solving.
//
// That second half is not a workaround bolted on here -- it is already required on this robot for
// an unrelated reason: crx_kinematics always solves to `flange` and ignores the SRDF tip. The
// machinery that corrects for the cutter's offset is exactly the machinery that lets the TCP move
// when the tool changes.

#include <moveit2_extended_msgs/msg/tool_info.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <map>
#include <string>
#include <vector>

namespace moveit2_extended::tools
{

using moveit2_extended_msgs::msg::ToolInfo;

/** One collision shape making up a tool, in the mount link's frame. */
struct ToolShape
{
  shape_msgs::msg::SolidPrimitive primitive;
  geometry_msgs::msg::Pose pose;
};

struct Tool
{
  ToolInfo info;
  std::vector<ToolShape> shapes;
  /** Where each dimension came from. This repository already uses the convention in
   *  carrier_types.yaml, and it earns its keep: a number nobody measured should not look like one
   *  that somebody did.
   *
   *    measured   taken off the real part or its CAD
   *    datasheet  from the manufacturer's published drawing
   *    estimate   inferred, and conservative -- an over-sized collision volume keeps the planner
   *               away, which is the safe direction to be wrong in */
  std::string provenance = "estimate";
};

class ToolRegistry
{
public:
  /** Replace the contents from YAML text. Leaves the registry untouched on a malformed file. */
  bool loadFromText(const std::string& yaml_text, std::string* error = nullptr);
  bool loadFromFile(const std::string& path, std::string* error = nullptr);

  const Tool* find(const std::string& name) const;
  std::vector<ToolInfo> list() const;
  std::vector<std::string> names() const;

  size_t size() const
  {
    return tools_.size();
  }

  /** The link a tool is attached to. One mount for the whole registry: a robot with two mounts
   *  needs two registries, and pretending otherwise would let a tool be fitted to the wrong one. */
  std::string mount_link;

private:
  std::map<std::string, Tool> tools_;
};

}  // namespace moveit2_extended::tools
