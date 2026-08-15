// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// The risk this test exists for:
//
// Behaviors are registered by a plugin (.so A) into a factory that lives in the core (.so B), and
// they exchange typed values through BT::Any, which compares std::type_index across that boundary.
// If either library were built with hidden visibility, each would get its own type_info for
// geometry_msgs::msg::PoseStamped, the comparison would fail, and EVERY cross-library port read
// would throw at tick time -- with an error about types that look identical.
//
// It is the kind of failure that is trivial to prevent and miserable to diagnose, so it is checked
// on the real pluginlib path: load the loader exactly as the Objective server does, build a tree
// from XML, and pass a message from one Behavior to another through the blackboard.
#include <moveit2_extended_core/behavior_context.hpp>
#include <moveit2_extended_core/behavior_loader_base.hpp>
#include <moveit2_extended_core/bt_compat.hpp>
#include <moveit2_extended_core/tree_introspection.hpp>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

using moveit2_extended::BehaviorContext;
using moveit2_extended::BehaviorContextConfig;
using moveit2_extended::BehaviorContextPtr;
using moveit2_extended::BehaviorLoaderBase;
using moveit2_extended::BehaviorParameterMap;
using moveit2_extended::BtFactory;
using moveit2_extended::BtStatus;
using moveit2_extended::tickOnce;

namespace
{
struct Loaded
{
  std::shared_ptr<pluginlib::ClassLoader<BehaviorLoaderBase>> class_loader;
  std::shared_ptr<BehaviorLoaderBase> loader;
  BehaviorContextPtr context;
  std::unique_ptr<BtFactory> factory;
};

/** Load the Behaviors the same way the Objective server does: through pluginlib, into a factory
 *  that lives in a different shared library. */
Loaded loadViaPluginlib()
{
  Loaded out;
  out.class_loader = std::make_shared<pluginlib::ClassLoader<BehaviorLoaderBase>>(
      "moveit2_extended_core", "moveit2_extended::BehaviorLoaderBase");

  auto node = std::make_shared<rclcpp::Node>("test_cross_library_ports");
  BehaviorContextConfig config;
  config.start_planning_scene_monitor = false;  // no move_group in a unit test
  out.context = std::make_shared<BehaviorContext>(node, config, BehaviorParameterMap{});

  out.factory = std::make_unique<BtFactory>();
  out.loader = out.class_loader->createSharedInstance("moveit2_extended::behaviors::GeneralBehaviorsLoader");
  out.loader->registerBehaviors(*out.factory, out.context);
  return out;
}
}  // namespace

TEST(CrossLibraryPorts, TheLoaderRegistersItsBehaviors)
{
  auto loaded = loadViaPluginlib();
  const auto& builders = loaded.factory->builders();

  // A representative from each family, so a whole group going missing is caught.
  for (const char* name : { "MoveToJointState", "MoveToPose", "PlanCartesianPath", "RetimeTrajectory",
                            "ExecuteTrajectory", "ComputeInverseKinematics", "StopMotion",
                            "GetCurrentPlanningScene", "AddVirtualObjectToPlanningScene",
                            "ModifyObjectInPlanningScene", "ClearSceneObjects", "CreatePoseStamped",
                            "TransformPose", "GetLatestTransform", "GetJointState",
                            "LoadObjectiveParameters", "GetElementOfVector", "IsPoseNearIdentity",
                            "GetTextFromUser", "WaitForUserTrajectoryApproval", "IsUserAvailable" })
  {
    EXPECT_TRUE(builders.count(name)) << name << " was not registered";
  }
}

/** Every Behavior must declare its ports. registerBuilder does not check, and a Behavior with an
 *  empty port list registers happily and then fails every getInput() at run time. */
TEST(CrossLibraryPorts, EveryBehaviorDeclaresPorts)
{
  auto loaded = loadViaPluginlib();
  const auto& builtin = loaded.factory->builtinNodes();

  for (const auto& entry : loaded.factory->manifests())
  {
    if (builtin.count(entry.first))
    {
      continue;  // BehaviorTree.CPP's own control nodes legitimately have none
    }
    EXPECT_FALSE(entry.second.ports.empty()) << entry.first << " declares no ports";
  }
}

/** THE test. A PoseStamped written by one Behavior and read by another, through a blackboard whose
 *  machinery lives in a different shared library. */
TEST(CrossLibraryPorts, AMessageSurvivesTheBlackboardAcrossLibraries)
{
  auto loaded = loadViaPluginlib();

  auto tree = loaded.factory->createTreeFromText(R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence>
      <CreatePoseStamped frame_id="robot_base" x="0.5" y="0.1" z="0.6" pose="{start}"/>
      <TransformPose pose="{start}" dx="0.05" local="false" transformed_pose="{moved}"/>
    </Sequence>
  </BehaviorTree>
</root>)");

  ASSERT_EQ(tickOnce(tree), BtStatus::SUCCESS)
      << "a message did not survive the blackboard across the library boundary; check that neither "
         "library is built with hidden visibility";

  const auto moved = tree.rootBlackboard()->get<geometry_msgs::msg::PoseStamped>("moved");
  EXPECT_EQ(moved.header.frame_id, "robot_base");
  EXPECT_NEAR(moved.pose.position.x, 0.55, 1e-9) << "the offset was not applied in the parent frame";
  EXPECT_NEAR(moved.pose.position.y, 0.1, 1e-9);
}

/** A local offset moves along the pose's own axes and a parent-frame one does not. Confusing the
 *  two is how a retreat drives the tool back into what it was retreating from. */
TEST(CrossLibraryPorts, LocalAndParentFrameOffsetsDiffer)
{
  auto loaded = loadViaPluginlib();

  // Yaw 90 degrees, then offset +x. In the parent frame that is +x; locally it is +y.
  auto tree = loaded.factory->createTreeFromText(R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence>
      <CreatePoseStamped frame_id="base" x="0" y="0" z="0" yaw="1.5707963" pose="{turned}"/>
      <TransformPose pose="{turned}" dx="0.1" local="true"  transformed_pose="{local_offset}"/>
      <TransformPose pose="{turned}" dx="0.1" local="false" transformed_pose="{parent_offset}"/>
    </Sequence>
  </BehaviorTree>
</root>)");
  ASSERT_EQ(tickOnce(tree), BtStatus::SUCCESS);

  const auto local = tree.rootBlackboard()->get<geometry_msgs::msg::PoseStamped>("local_offset");
  const auto parent = tree.rootBlackboard()->get<geometry_msgs::msg::PoseStamped>("parent_offset");

  EXPECT_NEAR(local.pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(local.pose.position.y, 0.1, 1e-6) << "a local +x offset on a yawed pose should move +y";
  EXPECT_NEAR(parent.pose.position.x, 0.1, 1e-6);
  EXPECT_NEAR(parent.pose.position.y, 0.0, 1e-6);
}

TEST(CrossLibraryPorts, PoseWithoutAFrameIsRefused)
{
  auto loaded = loadViaPluginlib();
  auto tree = loaded.factory->createTreeFromText(R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <CreatePoseStamped x="0.5" pose="{p}"/>
  </BehaviorTree>
</root>)");
  EXPECT_EQ(tickOnce(tree), BtStatus::FAILURE) << "a pose with no frame must not be silently accepted";
}

TEST(CrossLibraryPorts, GetElementOfVectorHandlesBoundsAndNegativeIndices)
{
  auto loaded = loadViaPluginlib();

  auto ok = loaded.factory->createTreeFromText(R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <GetElementOfVector vector="a;b;c" index="-1" element="{last}" size="{n}"/>
  </BehaviorTree>
</root>)");
  ASSERT_EQ(tickOnce(ok), BtStatus::SUCCESS);
  EXPECT_EQ(ok.rootBlackboard()->get<std::string>("last"), "c") << "index -1 should be the last element";
  EXPECT_EQ(ok.rootBlackboard()->get<int>("n"), 3);

  auto out_of_range = loaded.factory->createTreeFromText(R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <GetElementOfVector vector="a;b" index="7" element="{x}"/>
  </BehaviorTree>
</root>)");
  EXPECT_EQ(tickOnce(out_of_range), BtStatus::FAILURE);
}

TEST(CrossLibraryPorts, IsPoseNearIdentityDiscriminates)
{
  auto loaded = loadViaPluginlib();

  auto near = loaded.factory->createTreeFromText(R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence>
      <CreatePoseStamped frame_id="base" x="0.001" pose="{p}"/>
      <IsPoseNearIdentity pose="{p}" position_tolerance="0.005"/>
    </Sequence>
  </BehaviorTree>
</root>)");
  EXPECT_EQ(tickOnce(near), BtStatus::SUCCESS);

  auto far = loaded.factory->createTreeFromText(R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence>
      <CreatePoseStamped frame_id="base" x="0.5" pose="{p}"/>
      <IsPoseNearIdentity pose="{p}" position_tolerance="0.005"/>
    </Sequence>
  </BehaviorTree>
</root>)");
  EXPECT_EQ(tickOnce(far), BtStatus::FAILURE);
}

/** The XML-facing half of the port conversions: a pose written as an attribute must reach a
 *  PoseStamped port with the right numbers, and a bad one must be rejected at tree-build time
 *  rather than at tick time. */
TEST(CrossLibraryPorts, PoseLiteralsInXml)
{
  auto loaded = loadViaPluginlib();

  auto tree = loaded.factory->createTreeFromText(R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <IsPoseNearIdentity pose="base;0.0;0.0;0.0;0;0;0" position_tolerance="0.001"/>
  </BehaviorTree>
</root>)");
  EXPECT_EQ(tickOnce(tree), BtStatus::SUCCESS);

  // What actually happens with a malformed literal, measured rather than assumed:
  //
  //   1. Building the tree does NOT fail. BehaviorTree.CPP parses port literals lazily, on the
  //      first getInput(), not at construction.
  //   2. Ticking does NOT throw either. getInput() returns its error inside an Optional, and a
  //      well-written Behavior turns that into FAILURE with a log line.
  //
  // So a typo'd pose in an Objective is invisible until that branch is reached, and then it is a
  // quiet failure in the middle of a run -- possibly after the arm has already moved. That is the
  // whole reason invalidPortValuesInXml() exists and why the editor calls it before saving.
  auto malformed = loaded.factory->createTreeFromText(R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <IsPoseNearIdentity pose="not_enough;fields"/>
  </BehaviorTree>
</root>)");
  EXPECT_EQ(tickOnce(malformed), BtStatus::FAILURE)
      << "a malformed literal should fail the Behavior when the port is finally read";
}

/** The editor's defence against the lazy parse above: catch it at edit time, naming the field. */
TEST(CrossLibraryPorts, MalformedLiteralsAreCaughtByValidation)
{
  auto loaded = loadViaPluginlib();

  const auto bad = moveit2_extended::invalidPortValuesInXml(*loaded.factory, R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <IsPoseNearIdentity name="wrong_pose" pose="not_enough;fields"/>
  </BehaviorTree>
</root>)");
  ASSERT_EQ(bad.size(), 1u) << "the malformed pose literal was not caught";
  EXPECT_NE(bad.front().find("wrong_pose.pose"), std::string::npos) << bad.front();

  const auto good = moveit2_extended::invalidPortValuesInXml(*loaded.factory, R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <IsPoseNearIdentity name="fine" pose="base;0;0;0;0;0;0" position_tolerance="0.01"/>
  </BehaviorTree>
</root>)");
  EXPECT_TRUE(good.empty()) << "a valid literal was reported as invalid";

  // A blackboard reference has no type until run time, so it must not be judged here.
  const auto reference = moveit2_extended::invalidPortValuesInXml(*loaded.factory, R"(
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <IsPoseNearIdentity name="from_bb" pose="{some_pose}"/>
  </BehaviorTree>
</root>)");
  EXPECT_TRUE(reference.empty()) << "a {blackboard} reference must not be treated as a literal";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
