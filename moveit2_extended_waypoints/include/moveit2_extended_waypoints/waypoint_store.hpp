// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

// A YAML-backed store of named arm targets.
//
// Deliberately ROS-free apart from the message type, so the file format, the rename/overwrite
// rules and the round-trip can be tested without a node, a robot or a running graph.
//
// The existing scripts/waypoint_manager.py in this workspace is for the MOBILE BASE and speaks
// Nav2's FollowWaypoints. It is a different thing with a different frame semantics, and sharing a
// format between them would only make it easier to replay a base waypoint on the arm.

#include <moveit2_extended_msgs/msg/arm_waypoint.hpp>

#include <map>
#include <string>
#include <vector>

namespace moveit2_extended::waypoints
{

using moveit2_extended_msgs::msg::ArmWaypoint;

class WaypointStore
{
public:
  /** Replace the contents from YAML text. Returns false and leaves the store UNTOUCHED on a
   *  malformed file -- a half-loaded set of targets is worse than the previous one. */
  bool loadFromText(const std::string& yaml_text, std::string* error = nullptr);

  /** Load from a file (package:// accepted). A missing file is not an error: it is what an empty
   *  store looks like before anything has been taught. */
  bool loadFromFile(const std::string& path, std::string* error = nullptr);

  std::string toYaml() const;
  bool saveToFile(const std::string& path, std::string* error = nullptr) const;

  /** Returns false when `name` exists and `overwrite` is false. */
  bool add(const ArmWaypoint& waypoint, bool overwrite, std::string* error = nullptr);
  bool remove(const std::string& name, std::string* error = nullptr);
  bool rename(const std::string& from, const std::string& to, bool overwrite, std::string* error = nullptr);

  const ArmWaypoint* find(const std::string& name) const;
  /** Sorted by name. `tag_filter` empty returns everything. */
  std::vector<ArmWaypoint> list(const std::string& tag_filter = "") const;

  size_t size() const
  {
    return waypoints_.size();
  }
  void clear()
  {
    waypoints_.clear();
  }

  /** Defaults written into the file header and applied to waypoints that omit them. */
  std::string default_group = "arm";
  std::string default_frame_id = "";
  std::string default_pose_link = "";
  std::string robot_name = "";

private:
  std::map<std::string, ArmWaypoint> waypoints_;
};

}  // namespace moveit2_extended::waypoints
