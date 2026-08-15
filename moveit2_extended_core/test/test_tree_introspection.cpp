// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// The XML checks here are what the browser editor relies on to refuse a bad save, and what turns
// "the objective failed" into "you referenced a Behavior that does not exist". Built with plain
// BehaviorTree.CPP built-ins so no ROS node is needed.
#include <moveit2_extended_core/status_logger.hpp>
#include <moveit2_extended_core/tree_introspection.hpp>

#include <gtest/gtest.h>
#include <rclcpp/clock.hpp>

#include <algorithm>

using moveit2_extended::BtFactory;
using moveit2_extended::describeTree;
using moveit2_extended::invalidPortValuesInXml;
using moveit2_extended::missingBehaviorsInXml;
using moveit2_extended::referencedBehaviorsInXml;
using moveit2_extended::runningLeaf;
using moveit2_extended::treeIdsInXml;
using moveit2_extended::unknownPortsInXml;

namespace
{
/** A leaf with one declared port, so the unknown-port check has something to be right about. */
class Echo : public BT::SyncActionNode
{
public:
  Echo(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    // `times` is typed on purpose: a string port can never hold an invalid value, so without a
    // non-string port there is nothing for invalidPortValuesInXml to be right about.
    return { BT::InputPort<std::string>("text", "", "what to echo"),
             BT::InputPort<int>("times", 1, "how many times"),
             BT::OutputPort<int>("count", "how many were echoed") };
  }
  BT::NodeStatus tick() override
  {
    return BT::NodeStatus::SUCCESS;
  }
};

class AlwaysFail : public BT::SyncActionNode
{
public:
  AlwaysFail(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return {};
  }
  BT::NodeStatus tick() override
  {
    return BT::NodeStatus::FAILURE;
  }
};

/** Stays RUNNING for ever. Must be a StatefulActionNode: BehaviorTree.CPP throws
 *  "SyncActionNode MUST never return RUNNING" if a synchronous node tries this. */
class Forever : public BT::StatefulActionNode
{
public:
  Forever(const std::string& name, const BT::NodeConfiguration& config) : BT::StatefulActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return {};
  }
  BT::NodeStatus onStart() override
  {
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override
  {
  }
};

BtFactory makeFactory()
{
  BtFactory factory;
  factory.registerNodeType<Echo>("Echo");
  factory.registerNodeType<AlwaysFail>("AlwaysFail");
  factory.registerNodeType<Forever>("Forever");
  return factory;
}

constexpr const char* kSimpleXml = R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence name="root">
      <Echo name="first" text="hello"/>
      <Fallback name="recovery">
        <AlwaysFail name="doomed"/>
        <Echo name="second" text="world"/>
      </Fallback>
    </Sequence>
  </BehaviorTree>
</root>)";
}  // namespace

TEST(TreeIntrospection, TreeIdsAndReferencedBehaviors)
{
  EXPECT_EQ(treeIdsInXml(kSimpleXml), (std::vector<std::string>{ "Main" }));

  const auto referenced = referencedBehaviorsInXml(kSimpleXml);
  EXPECT_NE(std::find(referenced.begin(), referenced.end(), "Echo"), referenced.end());
  EXPECT_NE(std::find(referenced.begin(), referenced.end(), "AlwaysFail"), referenced.end());
  EXPECT_NE(std::find(referenced.begin(), referenced.end(), "Sequence"), referenced.end());
}

TEST(TreeIntrospection, MissingBehaviorsAreNamed)
{
  const auto factory = makeFactory();
  EXPECT_TRUE(missingBehaviorsInXml(factory, kSimpleXml).empty());

  constexpr const char* xml = R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence>
      <NoSuchBehavior name="a"/>
      <AlsoMissing name="b"/>
    </Sequence>
  </BehaviorTree>
</root>)";
  EXPECT_EQ(missingBehaviorsInXml(factory, xml), (std::vector<std::string>{ "AlsoMissing", "NoSuchBehavior" }));
}

/** A subtree defined in the same file is not a missing Behavior, however much it looks like one to
 *  a naive check. */
TEST(TreeIntrospection, LocalSubtreesAreNotMissing)
{
  const auto factory = makeFactory();
  constexpr const char* xml = R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence>
      <SubTree ID="Helper"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="Helper">
    <Echo name="inner" text="hi"/>
  </BehaviorTree>
</root>)";
  EXPECT_TRUE(missingBehaviorsInXml(factory, xml).empty());
  EXPECT_EQ(treeIdsInXml(xml), (std::vector<std::string>{ "Main", "Helper" }));
}

/** BehaviorTree.CPP tolerates an attribute that matches no port, so a typo silently becomes "the
 *  Behavior read its default". That is exactly the class of bug the editor must catch at save
 *  time. */
TEST(TreeIntrospection, UnknownPortsAreNamedWithTheirNode)
{
  const auto factory = makeFactory();
  constexpr const char* xml = R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Echo name="typo_here" txet="hello"/>
  </BehaviorTree>
</root>)";
  EXPECT_EQ(unknownPortsInXml(factory, xml), (std::vector<std::string>{ "typo_here.txet" }));
}

TEST(TreeIntrospection, CorrectPortsAreNotReported)
{
  const auto factory = makeFactory();
  EXPECT_TRUE(unknownPortsInXml(factory, kSimpleXml).empty());
}

TEST(TreeIntrospection, NameAndIdAttributesAreNotPorts)
{
  const auto factory = makeFactory();
  constexpr const char* xml = R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Echo name="labelled" text="x"/>
  </BehaviorTree>
</root>)";
  EXPECT_TRUE(unknownPortsInXml(factory, xml).empty());
}

TEST(TreeIntrospection, DescribeTreeRecoversTheParentChildGraph)
{
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(kSimpleXml);

  const auto structure = describeTree(tree, "Main", kSimpleXml, "instance_1");
  EXPECT_EQ(structure.objective_name, "Main");
  EXPECT_EQ(structure.tree_instance_id, "instance_1");
  EXPECT_FALSE(structure.xml.empty());
  ASSERT_GE(structure.nodes.size(), 5u);

  // Root first, with no parent.
  EXPECT_EQ(structure.nodes.front().instance_name, "root");
  EXPECT_EQ(structure.nodes.front().parent_uid, 0u);
  EXPECT_EQ(structure.nodes.front().children_uids.size(), 2u);

  // Every non-root node must be somebody's child, otherwise the graph cannot be drawn.
  const auto uid_of = [&](const std::string& name) -> uint16_t {
    for (const auto& node : structure.nodes)
    {
      if (node.instance_name == name)
      {
        return node.uid;
      }
    }
    return 0;
  };
  const uint16_t fallback_uid = uid_of("recovery");
  ASSERT_NE(fallback_uid, 0u);
  bool doomed_is_child_of_fallback = false;
  for (const auto& node : structure.nodes)
  {
    if (node.instance_name == "doomed")
    {
      doomed_is_child_of_fallback = (node.parent_uid == fallback_uid);
    }
  }
  EXPECT_TRUE(doomed_is_child_of_fallback);
}

/** Reporting the root, or the Sequence that propagated the failure, tells an operator nothing
 *  about what to fix. The deepest failing Behavior is the answer -- and it has to come from the
 *  event log, because the tree itself forgets. */
TEST(TreeIntrospection, FirstFailureInTheTickIsTheCulprit)
{
  auto factory = makeFactory();
  constexpr const char* xml = R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence name="outer">
      <Sequence name="inner">
        <AlwaysFail name="the_culprit"/>
      </Sequence>
    </Sequence>
  </BehaviorTree>
</root>)";
  auto tree = factory.createTreeFromText(xml);

  moveit2_extended::ObjectiveStatusLogger logger(tree, std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME));
  ASSERT_EQ(moveit2_extended::tickOnce(tree), BT::NodeStatus::FAILURE);

  const auto events = logger.drain();
  ASSERT_FALSE(events.empty());

  moveit2_extended_msgs::msg::BehaviorStatus culprit;
  ASSERT_TRUE(moveit2_extended::firstFailureIn(events, culprit));
  EXPECT_EQ(culprit.instance_name, "the_culprit");
  EXPECT_EQ(culprit.registration_name, "AlwaysFail");
}

/** The reason the event log is needed at all: after the tick, nothing in the tree still says a
 *  failure happened. If this ever starts failing, BehaviorTree.CPP changed its halt semantics and
 *  the simpler status-based implementation becomes possible again. */
TEST(TreeIntrospection, TheTreeItselfForgetsThatAnythingFailed)
{
  auto factory = makeFactory();
  auto tree = factory.createTreeFromText(R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence name="outer"><AlwaysFail name="doomed"/></Sequence>
  </BehaviorTree>
</root>)");
  ASSERT_EQ(moveit2_extended::tickOnce(tree), BT::NodeStatus::FAILURE);

  for (const auto& node : tree.nodes)
  {
    EXPECT_NE(node->status(), BT::NodeStatus::FAILURE)
        << "'" << node->name() << "' still reports FAILURE; the tree no longer forgets";
  }
}

TEST(TreeIntrospection, NoFailureInTheEventsIsReportedAsSuch)
{
  moveit2_extended_msgs::msg::BehaviorStatus culprit;
  EXPECT_FALSE(moveit2_extended::firstFailureIn({}, culprit));

  moveit2_extended_msgs::msg::BehaviorStatus success;
  success.current_status = moveit2_extended_msgs::msg::BehaviorStatus::SUCCESS;
  EXPECT_FALSE(moveit2_extended::firstFailureIn({ success }, culprit));
}

TEST(TreeIntrospection, RunningLeafIsTheDeepestRunningNode)
{
  auto factory = makeFactory();
  constexpr const char* xml = R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence name="outer">
      <Forever name="the_busy_one"/>
    </Sequence>
  </BehaviorTree>
</root>)";
  auto tree = factory.createTreeFromText(xml);
  EXPECT_EQ(moveit2_extended::tickOnce(tree), BT::NodeStatus::RUNNING);

  const auto* leaf = runningLeaf(tree);
  ASSERT_NE(leaf, nullptr);
  EXPECT_EQ(leaf->name(), "the_busy_one");
}

TEST(TreeIntrospection, MalformedXmlDoesNotThrow)
{
  const auto factory = makeFactory();
  const std::string broken = "<root><BehaviorTree ID=\"Main\"><Sequence></root>";
  EXPECT_NO_THROW(missingBehaviorsInXml(factory, broken));
  EXPECT_NO_THROW(unknownPortsInXml(factory, broken));
  EXPECT_NO_THROW(treeIdsInXml(broken));
}

// --- invalidPortValuesInXml -------------------------------------------------------------------
//
// The fourth validation check, and the one that exists because BehaviorTree.CPP parses port
// literals LAZILY: a tree with times="soon" builds without complaint and then fails halfway
// through a run, having already moved the arm. Catching it at save time is the whole point.

TEST(TreeIntrospection, AValidPortValuePasses)
{
  const auto factory = makeFactory();
  constexpr const char* xml = R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main"><Echo name="ok" text="hi" times="3"/></BehaviorTree>
</root>)";
  EXPECT_TRUE(invalidPortValuesInXml(factory, xml).empty());
}

TEST(TreeIntrospection, AnUnparseableValueIsReportedWithTheTypeAndWhatWasWritten)
{
  const auto factory = makeFactory();
  constexpr const char* xml = R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main"><Echo name="wrong" times="soon"/></BehaviorTree>
</root>)";
  const auto invalid = invalidPortValuesInXml(factory, xml);
  ASSERT_EQ(invalid.size(), 1u);
  // The message has to name the node, the port, the offending text and the expected type. "stod"
  // -- which is what the standard converter's what() says on its own -- is useless to the person
  // editing the tree.
  EXPECT_NE(invalid[0].find("wrong.times"), std::string::npos) << invalid[0];
  EXPECT_NE(invalid[0].find("'soon'"), std::string::npos) << invalid[0];
  EXPECT_NE(invalid[0].find("int"), std::string::npos) << invalid[0];
}

TEST(TreeIntrospection, ABlackboardReferenceIsNotParsedAsALiteral)
{
  // "{key}" is resolved at run time against whatever type the entry holds, so there is nothing to
  // check here. Reporting it would make every well-wired tree unsaveable.
  const auto factory = makeFactory();
  constexpr const char* xml = R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main"><Echo name="wired" times="{how_many}"/></BehaviorTree>
</root>)";
  EXPECT_TRUE(invalidPortValuesInXml(factory, xml).empty());
}

TEST(TreeIntrospection, AnOutputPortTakesADestinationNotAValue)
{
  const auto factory = makeFactory();
  constexpr const char* xml = R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main"><Echo name="out" count="{tally}"/></BehaviorTree>
</root>)";
  EXPECT_TRUE(invalidPortValuesInXml(factory, xml).empty());
}

TEST(TreeIntrospection, AnUnknownPortIsLeftToTheUnknownPortCheck)
{
  // Each check reports one kind of problem. Reporting a typo'd port name here as well would show
  // the user the same mistake twice under two different headings.
  const auto factory = makeFactory();
  constexpr const char* xml = R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main"><Echo name="typo" tiems="3"/></BehaviorTree>
</root>)";
  EXPECT_TRUE(invalidPortValuesInXml(factory, xml).empty());
  EXPECT_FALSE(unknownPortsInXml(factory, xml).empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
