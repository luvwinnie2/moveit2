// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// THE checkpoint test for this package.
//
// It proves the seam that everything downstream depends on: a Behavior with a three-argument
// constructor, registered through registerBuilder with a capturing lambda, built from XML by the
// factory, ticked to completion, and reading a value that was seeded onto the blackboard as a
// string. If this passes, adding Behaviors is routine. If it does not, nothing else in the project
// can work -- which is why it deliberately involves no MoveIt and no robot.
#include <moveit2_extended_core/behavior_loader_base.hpp>
#include <moveit2_extended_core/behaviors/check_blackboard_value.hpp>
#include <moveit2_extended_core/behaviors/log_message.hpp>
#include <moveit2_extended_core/behaviors/wait_for_duration.hpp>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <thread>

using moveit2_extended::BehaviorContext;
using moveit2_extended::BehaviorContextConfig;
using moveit2_extended::BehaviorContextPtr;
using moveit2_extended::BehaviorParameterMap;
using moveit2_extended::BtFactory;
using moveit2_extended::BtStatus;
using moveit2_extended::registerBehavior;
using moveit2_extended::tickOnce;

namespace
{
BehaviorContextPtr makeContext(const std::string& behavior_yaml = "")
{
  auto node = std::make_shared<rclcpp::Node>("test_behavior_registration");
  BehaviorContextConfig config;
  // No move_group in a unit test, and none needed: the point is that the seam works without it.
  config.start_planning_scene_monitor = false;
  return std::make_shared<BehaviorContext>(node, config, BehaviorParameterMap::fromText(behavior_yaml));
}

BtFactory makeFactory(const BehaviorContextPtr& context)
{
  BtFactory factory;
  registerBehavior<moveit2_extended::behaviors::LogMessage>(factory, "LogMessage", context);
  registerBehavior<moveit2_extended::behaviors::WaitForDuration>(factory, "WaitForDuration", context);
  registerBehavior<moveit2_extended::behaviors::CheckBlackboardValue>(factory, "CheckBlackboardValue", context);
  return factory;
}

/** Tick until the tree settles, with a wall-clock bound so a stuck tree fails the test instead of
 *  hanging the suite. */
BtStatus runToCompletion(moveit2_extended::BtTree& tree, std::chrono::seconds limit = std::chrono::seconds(5))
{
  const auto deadline = std::chrono::steady_clock::now() + limit;
  BtStatus status = BtStatus::RUNNING;
  while (std::chrono::steady_clock::now() < deadline)
  {
    status = tickOnce(tree);
    if (status != BtStatus::RUNNING)
    {
      return status;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return status;
}
}  // namespace

TEST(BehaviorRegistration, ContextIsInjectedThroughTheBuilder)
{
  auto context = makeContext();
  auto factory = makeFactory(context);

  // If the capturing-lambda builder did not work, this would throw at construction.
  auto tree = factory.createTreeFromText(R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <LogMessage message="constructed" level="info"/>
  </BehaviorTree>
</root>)");
  EXPECT_EQ(tickOnce(tree), BtStatus::SUCCESS);
}

/** registerBuilder does not check providedPorts(), so a Behavior that forgot it registers happily
 *  and then fails every getInput() at runtime. Our wrapper turns that into a compile error; here we
 *  simply confirm the ports really did make it into the manifest. */
TEST(BehaviorRegistration, PortsReachTheManifest)
{
  auto context = makeContext();
  auto factory = makeFactory(context);

  const auto& manifests = factory.manifests();
  ASSERT_TRUE(manifests.count("LogMessage"));
  const auto& ports = manifests.at("LogMessage").ports;
  EXPECT_TRUE(ports.count("message"));
  EXPECT_TRUE(ports.count("level"));

  ASSERT_TRUE(manifests.count("WaitForDuration"));
  EXPECT_TRUE(manifests.at("WaitForDuration").ports.count("duration"));
}

/** Blackboard entries written as strings must be readable through typed ports. This is what lets
 *  the server accept "ros2 action send_goal" parameters for ports it has never heard of. */
TEST(BehaviorRegistration, StringBlackboardEntriesConvertToTypedPorts)
{
  auto context = makeContext();
  auto factory = makeFactory(context);

  auto blackboard = moveit2_extended::BtBlackboard::create();
  blackboard->set("wait_seconds", std::string("0.05"));  // a string, read through a double port
  blackboard->set("caller", std::string("cli"));

  auto tree = factory.createTreeFromText(R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence>
      <WaitForDuration duration="{wait_seconds}"/>
      <CheckBlackboardValue value="{caller}" equals="cli"/>
    </Sequence>
  </BehaviorTree>
</root>)",
                                          blackboard);

  EXPECT_EQ(runToCompletion(tree), BtStatus::SUCCESS);
}

TEST(BehaviorRegistration, CheckBlackboardValueFailsOnMismatchAndInverts)
{
  auto context = makeContext();
  auto factory = makeFactory(context);

  auto blackboard = moveit2_extended::BtBlackboard::create();
  blackboard->set("who", std::string("alice"));

  auto mismatch = factory.createTreeFromText(R"(
<root main_tree_to_execute="M">
  <BehaviorTree ID="M"><CheckBlackboardValue value="{who}" equals="bob"/></BehaviorTree>
</root>)",
                                             blackboard);
  EXPECT_EQ(tickOnce(mismatch), BtStatus::FAILURE);

  auto inverted = factory.createTreeFromText(R"(
<root main_tree_to_execute="M">
  <BehaviorTree ID="M"><CheckBlackboardValue value="{who}" equals="bob" invert="true"/></BehaviorTree>
</root>)",
                                             blackboard);
  EXPECT_EQ(tickOnce(inverted), BtStatus::SUCCESS);
}

/** A Fallback must reach its second child when the first fails: this is the shape every recovery
 *  branch in a real Objective uses. */
TEST(BehaviorRegistration, FallbackReachesTheRecoveryBranch)
{
  auto context = makeContext();
  auto factory = makeFactory(context);

  auto blackboard = moveit2_extended::BtBlackboard::create();
  blackboard->set("caller", std::string("cli"));

  auto tree = factory.createTreeFromText(R"(
<root main_tree_to_execute="M">
  <BehaviorTree ID="M">
    <Fallback>
      <CheckBlackboardValue value="{caller}" equals="nobody"/>
      <LogMessage message="recovery ran" level="warn"/>
    </Fallback>
  </BehaviorTree>
</root>)",
                                          blackboard);
  EXPECT_EQ(runToCompletion(tree), BtStatus::SUCCESS);
}

/** Halting must actually stop a RUNNING Behavior and leave the tree idle, otherwise a cancelled
 *  Objective would keep the robot moving. */
TEST(BehaviorRegistration, HaltStopsARunningBehavior)
{
  auto context = makeContext();
  auto factory = makeFactory(context);

  auto tree = factory.createTreeFromText(R"(
<root main_tree_to_execute="M">
  <BehaviorTree ID="M"><WaitForDuration duration="30.0"/></BehaviorTree>
</root>)");

  EXPECT_EQ(tickOnce(tree), BtStatus::RUNNING);
  moveit2_extended::haltTree(tree);
  EXPECT_EQ(tree.rootNode()->status(), BtStatus::IDLE);
}

/** Behavior configuration parameters come from YAML and are separate from ports. */
TEST(BehaviorRegistration, ConfigurationParametersAreVisibleToTheBehavior)
{
  auto context = makeContext(R"(
behavior_parameters:
  LogMessage:
    level: error
)");
  EXPECT_EQ(context->parametersFor("LogMessage").get<std::string>("level", "info"), "error");
  EXPECT_TRUE(context->parametersFor("WaitForDuration").empty());
}

TEST(BehaviorRegistration, CallbackIslandIsNotInTheNodesExecutor)
{
  auto context = makeContext();
  auto island = context->makeCallbackIsland();
  ASSERT_TRUE(island.valid());
  // Pumping an island with nothing pending must return promptly rather than block.
  const auto before = std::chrono::steady_clock::now();
  island.pump(std::chrono::milliseconds(1));
  EXPECT_LT(std::chrono::steady_clock::now() - before, std::chrono::milliseconds(500));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
