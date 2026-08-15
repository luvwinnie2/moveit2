// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_core/objective_library.hpp>
#include <moveit2_extended_core/path_utils.hpp>
#include <moveit2_extended_core/tree_introspection.hpp>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <set>

namespace moveit2_extended
{
namespace fs = std::filesystem;

namespace
{
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
}  // namespace

void ObjectiveLibrary::loadSidecar(Entry& entry) const
{
  const fs::path yaml = fs::path(entry.file).replace_extension(".yaml");
  std::error_code ec;
  if (!fs::exists(yaml, ec))
  {
    return;
  }
  entry.sidecar = yaml.string();

  try
  {
    const YAML::Node root = YAML::LoadFile(entry.sidecar);
    if (!root || !root.IsMap())
    {
      return;
    }
    entry.description = root["description"] ? root["description"].as<std::string>() : "";
    entry.required_parameters = stringList(root["required_parameters"]);
    entry.optional_parameters = stringList(root["optional_parameters"]);
  }
  catch (const YAML::Exception&)
  {
    // A malformed sidecar costs documentation, not the Objective. Leave the metadata empty.
    entry.sidecar.clear();
  }
}

bool ObjectiveLibrary::load(BtFactory& factory, const std::vector<std::string>& directories, bool recursive,
                            std::vector<std::string>* per_file_errors)
{
  bool all_ok = true;
  const auto report = [&](std::string message) {
    all_ok = false;
    if (per_file_errors)
    {
      per_file_errors->push_back(std::move(message));
    }
  };

  for (const auto& directory : directories)
  {
    if (directory.empty())
    {
      continue;
    }
    std::string list_error;
    const auto files = listFiles(directory, ".xml", recursive, &list_error);
    if (!list_error.empty())
    {
      report("objective directory '" + directory + "': " + list_error);
      continue;
    }

    for (const auto& file : files)
    {
      std::string read_error;
      const std::string xml = readFile(file, &read_error);
      if (xml.empty())
      {
        report(file + ": " + (read_error.empty() ? "file is empty" : read_error));
        continue;
      }

      const auto tree_ids = treeIdsInXml(xml);
      if (tree_ids.empty())
      {
        report(file + ": no <BehaviorTree ID=...> found");
        continue;
      }

      // BehaviorTree.CPP verifies the XML against the registered node types when a tree is
      // registered, and throws if it references anything unknown. So an Objective that needs a
      // Behavior from a package that failed to load cannot be registered at all.
      //
      // Registering it is not the same as knowing about it. We still record the entry, with its
      // missing Behaviors, so ListObjectives can show it greyed out and say what it needs --
      // rather than the Objective simply disappearing and the operator wondering where it went.
      const auto missing = missingBehaviorsInXml(factory, xml);
      bool registered = false;
      if (missing.empty())
      {
        try
        {
          factory.registerBehaviorTreeFromText(xml);
          registered = true;
        }
        catch (const std::exception& exc)
        {
          report(file + ": " + exc.what());
          continue;
        }
      }
      else
      {
        std::string names;
        for (size_t i = 0; i < missing.size(); ++i)
        {
          names += (i ? ", " : "") + missing[i];
        }
        report(file + ": not runnable, references unregistered behavior(s): " + names);
      }

      // Every tree in the file becomes addressable, so an Objective can be composed of subtrees
      // that live beside it. The first ID is treated as the file's main tree.
      for (size_t i = 0; i < tree_ids.size(); ++i)
      {
        Entry entry;
        entry.name = tree_ids[i];
        entry.file = file;
        entry.xml = xml;
        entry.subtree_ids = tree_ids;
        entry.subtree_ids.erase(entry.subtree_ids.begin() + static_cast<long>(i));
        entry.referenced_behaviors = referencedBehaviorsInXml(xml);
        entry.runnable = registered;
        if (i == 0)
        {
          loadSidecar(entry);
        }

        if (entries_.count(entry.name))
        {
          report(file + ": tree ID '" + entry.name + "' is already defined in " + entries_[entry.name].file);
          continue;
        }
        entries_[entry.name] = std::move(entry);
      }
    }
  }
  return all_ok;
}

void ObjectiveLibrary::clear(BtFactory& factory)
{
  factory.clearRegisteredBehaviorTrees();
  entries_.clear();
}

std::vector<std::string> ObjectiveLibrary::names() const
{
  std::vector<std::string> out;
  out.reserve(entries_.size());
  for (const auto& kv : entries_)
  {
    out.push_back(kv.first);
  }
  return out;  // std::map keeps this sorted
}

const ObjectiveLibrary::Entry* ObjectiveLibrary::find(const std::string& name) const
{
  const auto it = entries_.find(name);
  return it == entries_.end() ? nullptr : &it->second;
}

moveit2_extended_msgs::msg::ObjectiveInfo ObjectiveLibrary::describe(const BtFactory& factory,
                                                                     const Entry& entry) const
{
  moveit2_extended_msgs::msg::ObjectiveInfo info;
  info.name = entry.name;
  info.description = entry.description;
  info.source_file = entry.file;
  info.required_parameters = entry.required_parameters;
  info.optional_parameters = entry.optional_parameters;
  info.subtree_ids = entry.subtree_ids;
  info.referenced_behaviors = entry.referenced_behaviors;
  info.missing_behaviors = missingBehaviorsInXml(factory, entry.xml);
  return info;
}

std::vector<std::string> ObjectiveLibrary::missingBehaviors(const BtFactory& factory, const std::string& name) const
{
  const Entry* entry = find(name);
  if (!entry)
  {
    return {};
  }
  return missingBehaviorsInXml(factory, entry->xml);
}

}  // namespace moveit2_extended
