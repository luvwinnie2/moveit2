// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

// The BehaviorTree.CPP version firewall.
//
// We build against v3.8.7 because that is what Nav2 already pulls into this system. v3 and v4
// share the namespace BT with incompatible layouts for TreeNode, Blackboard and
// NodeConfig(uration), and they ship as two separate shared libraries -- so a process that ends up
// linking both does not fail to link, it corrupts the heap. Nav2's BT nodes are compiled against
// v3, so v4 is not an option here.
//
// Everything in this project that would otherwise spell a v3-specific name spells it through this
// header instead. Migrating to v4 later is then this file plus an XML attribute sweep, rather than
// a search across every Behavior.
//
// RULE, enforced by test/test_no_raw_bt_api.cpp: outside this header, do not write
// BT::NodeConfiguration, BT::Optional, tickRoot() or tickRootWhileRunning().

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/abstract_logger.h>
#include <behaviortree_cpp_v3/xml_parsing.h>

#include <chrono>

namespace moveit2_extended
{

// ---- names that differ between v3 and v4 -----------------------------------------------------
using NodeConfig = BT::NodeConfiguration;  // v4: BT::NodeConfig
using BtFactory = BT::BehaviorTreeFactory;
using BtTree = BT::Tree;
using BtStatus = BT::NodeStatus;
using BtNodeType = BT::NodeType;
using BtBlackboard = BT::Blackboard;

/** v4 renames this to BT::Expected. Same shape either way: a value or an error string. */
template <typename T>
using BtExpected = BT::Optional<T>;

// ---- calls that differ between v3 and v4 -----------------------------------------------------

/** One tick of the root. v3: Tree::tickRoot(). v4: Tree::tickOnce(). */
inline BtStatus tickOnce(BtTree& tree)
{
  return tree.tickRoot();
}

/** Halt the whole tree. Must be called from the thread that ticks it. */
inline void haltTree(BtTree& tree)
{
  tree.haltTree();
}

/** Sleep between ticks, interrupted early when a node calls emitStateChanged(). Using this rather
 *  than a plain sleep is what lets an asynchronous Behavior wake the tree the moment its result
 *  lands, instead of the tree sitting out the rest of the tick period. */
inline void sleepBetweenTicks(BtTree& tree, std::chrono::milliseconds duration)
{
  tree.sleep(duration);
}

}  // namespace moveit2_extended
