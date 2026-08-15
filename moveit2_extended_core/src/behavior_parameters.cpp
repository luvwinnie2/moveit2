// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_core/behavior_parameters.hpp>
#include <moveit2_extended_core/path_utils.hpp>

#include <algorithm>

namespace moveit2_extended
{
namespace
{
constexpr const char* kRootKey = "behavior_parameters";

void setError(std::string* error, std::string message)
{
  if (error)
  {
    *error = std::move(message);
  }
}
}  // namespace

bool BehaviorParameters::has(const std::string& key) const
{
  return node_ && node_.IsMap() && node_[key] && !node_[key].IsNull();
}

BehaviorParameters BehaviorParameters::sub(const std::string& key) const
{
  if (!has(key))
  {
    return BehaviorParameters{};
  }
  return BehaviorParameters(node_[key]);
}

std::vector<std::string> BehaviorParameters::keys() const
{
  std::vector<std::string> out;
  if (node_ && node_.IsMap())
  {
    for (const auto& kv : node_)
    {
      out.push_back(kv.first.as<std::string>());
    }
    std::sort(out.begin(), out.end());
  }
  return out;
}

bool BehaviorParameters::empty() const
{
  return !node_ || !node_.IsMap() || node_.size() == 0;
}

BehaviorParameterMap BehaviorParameterMap::fromText(const std::string& yaml_text, std::string* error)
{
  setError(error, {});
  BehaviorParameterMap map;
  if (yaml_text.empty())
  {
    return map;
  }

  YAML::Node root;
  try
  {
    root = YAML::Load(yaml_text);
  }
  catch (const YAML::Exception& exc)
  {
    setError(error, std::string("cannot parse behavior parameters: ") + exc.what());
    return map;
  }

  if (!root || root.IsNull())
  {
    return map;
  }
  // Accept both the wrapped form and a bare per-Behavior map, so a fragment can be pasted in
  // without knowing which one the loader expects.
  const YAML::Node behaviors = (root.IsMap() && root[kRootKey]) ? root[kRootKey] : root;
  if (!behaviors.IsMap())
  {
    setError(error, "behavior parameters must be a map of behavior name -> parameters");
    return map;
  }

  for (const auto& kv : behaviors)
  {
    map.by_behavior_[kv.first.as<std::string>()] = BehaviorParameters(kv.second);
  }
  return map;
}

BehaviorParameterMap BehaviorParameterMap::fromFiles(const std::vector<std::string>& paths, std::string* error)
{
  setError(error, {});
  BehaviorParameterMap merged;
  for (const auto& path : paths)
  {
    if (path.empty())
    {
      continue;
    }
    std::string read_error;
    const std::string text = readFile(path, &read_error);
    if (text.empty() && !read_error.empty())
    {
      setError(error, read_error);
      return merged;
    }
    std::string parse_error;
    const BehaviorParameterMap one = fromText(text, &parse_error);
    if (!parse_error.empty())
    {
      setError(error, path + ": " + parse_error);
      return merged;
    }
    merged.merge(one);
  }
  return merged;
}

void BehaviorParameterMap::merge(const BehaviorParameterMap& other)
{
  for (const auto& kv : other.by_behavior_)
  {
    by_behavior_[kv.first] = kv.second;  // whole-block replace; see the header for why
  }
}

BehaviorParameters BehaviorParameterMap::forBehavior(const std::string& registration_name) const
{
  const auto it = by_behavior_.find(registration_name);
  return it == by_behavior_.end() ? BehaviorParameters{} : it->second;
}

std::vector<std::string> BehaviorParameterMap::behaviorNames() const
{
  std::vector<std::string> out;
  out.reserve(by_behavior_.size());
  for (const auto& kv : by_behavior_)
  {
    out.push_back(kv.first);
  }
  std::sort(out.begin(), out.end());
  return out;
}

}  // namespace moveit2_extended
