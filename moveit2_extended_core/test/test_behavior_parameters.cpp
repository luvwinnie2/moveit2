// Copyright 2026 Leow Chee Siang. Apache-2.0.
// Pure YAML, no ROS: the fastest feedback loop in the package.
#include <moveit2_extended_core/behavior_parameters.hpp>

#include <gtest/gtest.h>

using moveit2_extended::BehaviorConfigError;
using moveit2_extended::BehaviorParameterMap;
using moveit2_extended::BehaviorParameters;

TEST(BehaviorParameters, ReadsTheWrappedForm)
{
  const auto map = BehaviorParameterMap::fromText(R"(
behavior_parameters:
  MoveToNamedPose:
    planning_group: arm
    allowed_planning_time: 5.0
    num_planning_attempts: 3
    plan_only: true
)");
  const auto params = map.forBehavior("MoveToNamedPose");
  EXPECT_EQ(params.get<std::string>("planning_group", "?"), "arm");
  EXPECT_DOUBLE_EQ(params.get<double>("allowed_planning_time", 0.0), 5.0);
  EXPECT_EQ(params.get<int>("num_planning_attempts", 0), 3);
  EXPECT_TRUE(params.get<bool>("plan_only", false));
}

/** A per-Behavior fragment should be usable as-is, without knowing whether the loader wants the
 *  wrapper. Rejecting it would be a pointless papercut. */
TEST(BehaviorParameters, ReadsTheBareForm)
{
  const auto map = BehaviorParameterMap::fromText(R"(
LogMessage:
  level: warn
)");
  EXPECT_EQ(map.forBehavior("LogMessage").get<std::string>("level", "info"), "warn");
}

TEST(BehaviorParameters, AbsentBehaviorGivesEmptyParameters)
{
  const auto map = BehaviorParameterMap::fromText("behavior_parameters:\n  A:\n    x: 1\n");
  const auto params = map.forBehavior("NotThere");
  EXPECT_TRUE(params.empty());
  EXPECT_EQ(params.get<int>("x", 42), 42) << "an absent block must fall back, not throw";
}

/** Silently defaulting a planning group is how a Behavior ends up moving the wrong arm, so
 *  require() must throw rather than substitute. */
TEST(BehaviorParameters, RequireThrowsWhenMissing)
{
  const auto map = BehaviorParameterMap::fromText("behavior_parameters:\n  A:\n    present: 1\n");
  const auto params = map.forBehavior("A");
  EXPECT_EQ(params.require<int>("present"), 1);
  EXPECT_THROW(params.require<int>("absent"), BehaviorConfigError);
}

TEST(BehaviorParameters, WrongTypeFallsBackForGetAndThrowsForRequire)
{
  const auto map = BehaviorParameterMap::fromText("behavior_parameters:\n  A:\n    value: not_a_number\n");
  const auto params = map.forBehavior("A");
  EXPECT_DOUBLE_EQ(params.get<double>("value", 1.5), 1.5);
  EXPECT_THROW(params.require<double>("value"), BehaviorConfigError);
}

TEST(BehaviorParameters, NestedBlocks)
{
  const auto map = BehaviorParameterMap::fromText(R"(
behavior_parameters:
  Planner:
    ompl:
      range: 0.25
      planner_id: RRTConnect
)");
  const auto ompl = map.forBehavior("Planner").sub("ompl");
  EXPECT_DOUBLE_EQ(ompl.get<double>("range", 0.0), 0.25);
  EXPECT_EQ(ompl.get<std::string>("planner_id", ""), "RRTConnect");
  EXPECT_TRUE(map.forBehavior("Planner").sub("missing").empty());
}

/** Whole-block replace, not deep merge. Deep merging produces a configuration that exists in no
 *  single file, which nobody can then read off the disk. */
TEST(BehaviorParameters, MergeReplacesTheWholeBlock)
{
  auto base = BehaviorParameterMap::fromText(R"(
behavior_parameters:
  A:
    keep: 1
    replace: 1
  B:
    untouched: 1
)");
  const auto overlay = BehaviorParameterMap::fromText(R"(
behavior_parameters:
  A:
    replace: 2
)");
  base.merge(overlay);

  const auto a = base.forBehavior("A");
  EXPECT_EQ(a.get<int>("replace", 0), 2);
  EXPECT_EQ(a.get<int>("keep", -1), -1) << "the whole block is replaced, so 'keep' is gone";
  EXPECT_EQ(base.forBehavior("B").get<int>("untouched", 0), 1);
}

TEST(BehaviorParameters, MalformedYamlIsReportedNotThrown)
{
  std::string error;
  const auto map = BehaviorParameterMap::fromText("behavior_parameters:\n  A: [unclosed\n", &error);
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(map.empty());
}

TEST(BehaviorParameters, EmptyTextIsFine)
{
  std::string error;
  const auto map = BehaviorParameterMap::fromText("", &error);
  EXPECT_TRUE(error.empty());
  EXPECT_TRUE(map.empty());
}

TEST(BehaviorParameters, BehaviorNamesAreSorted)
{
  const auto map = BehaviorParameterMap::fromText("behavior_parameters:\n  Zeta: {}\n  Alpha: {}\n  Mid: {}\n");
  EXPECT_EQ(map.behaviorNames(), (std::vector<std::string>{ "Alpha", "Mid", "Zeta" }));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
