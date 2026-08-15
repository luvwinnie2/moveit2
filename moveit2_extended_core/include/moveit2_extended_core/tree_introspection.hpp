// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_core/bt_compat.hpp>

#include <moveit2_extended_msgs/msg/behavior_info.hpp>
#include <moveit2_extended_msgs/msg/behavior_status.hpp>
#include <moveit2_extended_msgs/msg/tree_structure.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace moveit2_extended
{

uint8_t statusToMsg(BtStatus status);
uint8_t nodeTypeToMsg(BtNodeType type);

/** Reconstruct the parent/child graph of a built tree.
 *
 *  BehaviorTree.CPP v3's Tree exposes only a flat `nodes` vector with no parent links, so this
 *  walks the tree with applyRecursiveVisitor and recovers the structure by asking each node
 *  whether it is a ControlNode (many children) or a DecoratorNode (one child). Without this a UI
 *  can list the nodes but cannot draw the tree -- which is most of what Groot was for. */
moveit2_extended_msgs::msg::TreeStructure describeTree(const BtTree& tree, const std::string& objective_name,
                                                       const std::string& xml, const std::string& instance_id);

/** The Behavior that actually failed, from the status transitions of the tick that failed.
 *
 *  It has to come from the event log rather than from the tree, because by the time a tick
 *  returns FAILURE the tree no longer knows: a Sequence calls haltChildren() when a child fails,
 *  which resets those children to IDLE, and Tree::tickRoot() then resets the root to IDLE as well.
 *  Inspecting node statuses afterwards therefore reports nothing failed, for every run that
 *  failed.
 *
 *  Within one tick the children are ticked before their parent reports, so the FIRST transition
 *  to FAILURE in the tick is the deepest cause -- the Sequence that propagated it upwards comes
 *  later in the same list. Passing only the failing tick's events (not the whole run) matters:
 *  across a run, a Fallback's first child failing and being recovered is normal, and would
 *  otherwise be reported as the culprit.
 *
 *  Returns false when no failure is present in `events`. */
bool firstFailureIn(const std::vector<moveit2_extended_msgs::msg::BehaviorStatus>& events,
                    moveit2_extended_msgs::msg::BehaviorStatus& out);

/** The deepest RUNNING leaf: the one-line "what is it doing right now".
 *
 *  Unlike failure, this one can be read off the tree: RUNNING nodes keep their status between
 *  ticks, which is what makes them resumable in the first place. */
const BT::TreeNode* runningLeaf(const BtTree& tree);

/** Every Behavior the factory can build, with its ports -- the editor's node palette.
 *
 *  `loader_of_behavior` maps a registration name to the package whose loader registered it, so a
 *  collision between two packages is diagnosable from the command line. */
std::vector<moveit2_extended_msgs::msg::BehaviorInfo>
describeBehaviors(const BtFactory& factory, const std::unordered_map<std::string, std::string>& loader_of_behavior,
                  bool include_builtin = true);

/** Node IDs referenced by an Objective XML that the factory has no builder for.
 *
 *  Called before createTree() so the failure can name the missing Behaviors, instead of surfacing
 *  a raw BehaviorTree.CPP exception string that says only that something went wrong. */
std::vector<std::string> missingBehaviorsInXml(const BtFactory& factory, const std::string& xml);

/** Ports named on a node in the XML that the node does not declare.
 *
 *  BehaviorTree.CPP tolerates these silently, which means a typo in a port name shows up as a
 *  Behavior that reads a default instead of the value you thought you passed. Entries are
 *  formatted "NodeInstanceName.port_name" so an editor can point at the field. */
std::vector<std::string> unknownPortsInXml(const BtFactory& factory, const std::string& xml);

/** Well-formedness only: does this text parse as XML at all?
 *
 *  Separate from BT::VerifyXML on purpose. VerifyXML also rejects unknown node types, so using it
 *  as the syntax check would report "your XML is broken" for a tree whose only problem is that a
 *  Behavior package is not installed -- which sends the reader looking for a missing bracket that
 *  is not there. Returns true when the text parses, and sets `error` when it does not. */
bool isWellFormedXml(const std::string& xml, std::string& error);

/** Every <BehaviorTree ID="..."> in the XML. */
std::vector<std::string> treeIdsInXml(const std::string& xml);

/** Every node registration ID the XML refers to, whether or not the factory can build it, and
 *  excluding trees defined in the same file. Sorted and de-duplicated. */
std::vector<std::string> referencedBehaviorsInXml(const std::string& xml);

}  // namespace moveit2_extended
