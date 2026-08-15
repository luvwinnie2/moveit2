// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_core/bt_compat.hpp>

#include <moveit2_extended_msgs/msg/objective_info.hpp>

#include <map>
#include <string>
#include <vector>

namespace moveit2_extended
{

/** Reads Objective XML files from directories and registers them with a factory.
 *
 *  Objective metadata -- description, which parameters it expects -- lives in a SIDECAR YAML next
 *  to the XML, not inside it. BehaviorTree.CPP v3's VerifyXML rejects any child of <root> other
 *  than BehaviorTree, TreeNodesModel and include, so there is nowhere legal in the XML to put it. */
class ObjectiveLibrary
{
public:
  struct Entry
  {
    std::string name;      ///< the <BehaviorTree ID> this file's main tree carries
    std::string file;      ///< absolute path to the .xml
    std::string sidecar;   ///< absolute path to the .yaml, empty when absent
    std::string xml;       ///< cached file contents, so GetObjective needs no disk read
    std::string description;
    std::vector<std::string> required_parameters;
    std::vector<std::string> optional_parameters;
    std::vector<std::string> subtree_ids;          ///< other trees defined in the same file
    std::vector<std::string> referenced_behaviors; ///< every node ID the XML mentions
    /** False when the XML references a Behavior no loaded package provides.
     *
     *  BehaviorTree.CPP refuses to register such a tree at all, so it is known about but not
     *  buildable. It is still listed, with its missing Behaviors, rather than silently vanishing
     *  and leaving the operator to wonder where their Objective went. */
    bool runnable = false;
  };

  /** Scan `directories` (package:// accepted) for *.xml and register each with the factory.
   *
   *  Registered from text rather than from file so that one Objective can <SubTree ID="..."/> a
   *  tree defined in another file: all definitions land in the same factory.
   *
   *  A file that fails to parse is skipped and reported through `per_file_errors` -- one bad XML
   *  must not stop the server coming up, because then a typo takes every Objective down with it.
   *  Returns false when any file was skipped. */
  bool load(BtFactory& factory, const std::vector<std::string>& directories, bool recursive,
            std::vector<std::string>* per_file_errors = nullptr);

  /** Drop every registered tree from the factory and forget the entries. */
  void clear(BtFactory& factory);

  std::vector<std::string> names() const;
  const Entry* find(const std::string& name) const;

  /** Fill in an ObjectiveInfo, including which of its Behaviors the factory cannot build. */
  moveit2_extended_msgs::msg::ObjectiveInfo describe(const BtFactory& factory, const Entry& entry) const;

  /** Node IDs in `name` that the factory has no builder for. Empty means createTree() will work.
   *
   *  Checked before createTree() so the caller can report the missing names rather than a raw
   *  BehaviorTree.CPP exception. */
  std::vector<std::string> missingBehaviors(const BtFactory& factory, const std::string& name) const;

  size_t size() const
  {
    return entries_.size();
  }

private:
  /** Read the optional <file>.yaml beside an Objective. */
  void loadSidecar(Entry& entry) const;

  std::map<std::string, Entry> entries_;
};

}  // namespace moveit2_extended
