// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace moveit2_extended
{

/** Thrown when a Behavior asks for configuration it cannot run without. */
class BehaviorConfigError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/** Constant configuration for one *type* of Behavior.
 *
 *  This is the distinction MoveIt Pro draws between a "Behavior Configuration Parameter" and a
 *  "Behavior Data Port", and it is worth keeping:
 *
 *    configuration parameter        data port
 *    -----------------------        ---------
 *    from a YAML file               from the BT XML attribute or {blackboard}
 *    bound once, at construction    read on every tick
 *    same for every instance        per instance
 *    "which planning pipeline"      "which pose"
 *
 *  Anything that describes how this deployment is wired belongs here; anything that changes while
 *  a task runs belongs on a port. */
class BehaviorParameters
{
public:
  BehaviorParameters() = default;
  explicit BehaviorParameters(YAML::Node node) : node_(std::move(node))
  {
  }

  bool has(const std::string& key) const;

  template <typename T>
  T get(const std::string& key, const T& fallback) const
  {
    if (!node_ || !node_.IsMap())
    {
      return fallback;
    }
    const YAML::Node child = node_[key];
    if (!child || child.IsNull())
    {
      return fallback;
    }
    try
    {
      return child.as<T>();
    }
    catch (const YAML::Exception&)
    {
      return fallback;
    }
  }

  /** Like get(), but throws instead of substituting a default.
   *
   *  Use this for anything with no defensible default. Quietly defaulting a planning group is how
   *  a Behavior ends up moving the wrong arm, and the failure shows up as a motion rather than as
   *  an error. */
  template <typename T>
  T require(const std::string& key) const
  {
    if (!node_ || !node_.IsMap() || !node_[key] || node_[key].IsNull())
    {
      throw BehaviorConfigError("required behavior parameter '" + key + "' is not set");
    }
    try
    {
      return node_[key].as<T>();
    }
    catch (const YAML::Exception& exc)
    {
      throw BehaviorConfigError("behavior parameter '" + key + "' has the wrong type: " + exc.what());
    }
  }

  /** Nested block, e.g. params.sub("ompl").get<double>("range", 0.0). Empty when absent. */
  BehaviorParameters sub(const std::string& key) const;

  std::vector<std::string> keys() const;
  bool empty() const;
  const YAML::Node& raw() const
  {
    return node_;
  }

private:
  YAML::Node node_;
};

/** Every Behavior's configuration, keyed by registration name. */
class BehaviorParameterMap
{
public:
  /** Parse YAML text. Expects a top-level `behavior_parameters:` map; a file without it is
   *  accepted as the map itself, so a per-Behavior fragment can be dropped in as-is. */
  static BehaviorParameterMap fromText(const std::string& yaml_text, std::string* error = nullptr);

  /** Load and merge several files, in order. Paths may be package:// URIs.
   *
   *  Merging replaces a Behavior's whole block rather than deep-merging it. Deep-merging YAML
   *  produces configurations that exist in no single file and that nobody can read off the disk,
   *  which is a bad property for something that decides how a robot moves. */
  static BehaviorParameterMap fromFiles(const std::vector<std::string>& paths, std::string* error = nullptr);

  void merge(const BehaviorParameterMap& other);

  /** Empty parameters when the Behavior has no block. Absence is legal: most Behaviors are fully
   *  described by their ports. */
  BehaviorParameters forBehavior(const std::string& registration_name) const;

  std::vector<std::string> behaviorNames() const;
  bool empty() const
  {
    return by_behavior_.empty();
  }

private:
  std::unordered_map<std::string, BehaviorParameters> by_behavior_;
};

}  // namespace moveit2_extended
