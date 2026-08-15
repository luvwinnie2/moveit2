// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// The store holds the targets an operator taught by hand, so the properties that matter are the
// ones about not losing them: a malformed file must not empty the store, a rename must not
// silently clobber another target, and a round trip must not quietly drop the frame a pose was
// taught in.
#include <moveit2_extended_waypoints/waypoint_store.hpp>

#include <gtest/gtest.h>

#include <filesystem>

using moveit2_extended::waypoints::ArmWaypoint;
using moveit2_extended::waypoints::WaypointStore;
namespace fs = std::filesystem;

namespace
{
ArmWaypoint makeJointWaypoint(const std::string& name)
{
  ArmWaypoint waypoint;
  waypoint.name = name;
  waypoint.group = "arm";
  waypoint.has_joint_state = true;
  waypoint.joint_state.name = { "J1", "J2", "J3" };
  waypoint.joint_state.position = { 0.1, -0.2, 0.3 };
  return waypoint;
}

ArmWaypoint makePoseWaypoint(const std::string& name, const std::string& frame)
{
  ArmWaypoint waypoint;
  waypoint.name = name;
  waypoint.group = "arm";
  waypoint.has_pose = true;
  waypoint.pose_link = "cutting_point";
  waypoint.pose.header.frame_id = frame;
  waypoint.pose.pose.position.x = 0.512;
  waypoint.pose.pose.position.y = 0.061;
  waypoint.pose.pose.position.z = 0.744;
  waypoint.pose.pose.orientation.y = 0.7071;
  waypoint.pose.pose.orientation.w = 0.7071;
  return waypoint;
}

class TempFile
{
public:
  TempFile() : path_(fs::temp_directory_path() / ("m2x_wp_" + std::to_string(::getpid()) + "_" +
                                                  std::to_string(counter_++) + ".yaml"))
  {
  }
  ~TempFile()
  {
    std::error_code ec;
    fs::remove(path_, ec);
  }
  std::string str() const
  {
    return path_.string();
  }

private:
  fs::path path_;
  static inline int counter_ = 0;
};
}  // namespace

TEST(WaypointStore, AddFindList)
{
  WaypointStore store;
  ASSERT_TRUE(store.add(makeJointWaypoint("zeta"), false));
  ASSERT_TRUE(store.add(makeJointWaypoint("alpha"), false));

  EXPECT_EQ(store.size(), 2u);
  ASSERT_NE(store.find("alpha"), nullptr);
  EXPECT_EQ(store.find("nope"), nullptr);

  const auto listed = store.list();
  ASSERT_EQ(listed.size(), 2u);
  EXPECT_EQ(listed[0].name, "alpha") << "listing should be sorted, so the order does not depend on insertion";
  EXPECT_EQ(listed[1].name, "zeta");
}

TEST(WaypointStore, AddRefusesToClobberWithoutOverwrite)
{
  WaypointStore store;
  ASSERT_TRUE(store.add(makeJointWaypoint("target"), false));

  std::string error;
  EXPECT_FALSE(store.add(makeJointWaypoint("target"), false, &error));
  EXPECT_NE(error.find("already exists"), std::string::npos) << error;

  EXPECT_TRUE(store.add(makeJointWaypoint("target"), true));
}

TEST(WaypointStore, RenameRules)
{
  WaypointStore store;
  ASSERT_TRUE(store.add(makeJointWaypoint("old"), false));
  ASSERT_TRUE(store.add(makeJointWaypoint("occupied"), false));

  std::string error;
  EXPECT_FALSE(store.rename("missing", "x", false, &error));
  EXPECT_FALSE(store.rename("old", "occupied", false, &error))
      << "renaming onto an existing name must not silently destroy it";
  EXPECT_FALSE(store.rename("old", "", false, &error));

  ASSERT_TRUE(store.rename("old", "new", false));
  EXPECT_EQ(store.find("old"), nullptr);
  ASSERT_NE(store.find("new"), nullptr);
  EXPECT_EQ(store.find("new")->name, "new") << "the stored name must be updated, not just the key";

  EXPECT_TRUE(store.rename("new", "occupied", true));
  EXPECT_EQ(store.size(), 1u);
}

TEST(WaypointStore, Remove)
{
  WaypointStore store;
  ASSERT_TRUE(store.add(makeJointWaypoint("gone"), false));
  EXPECT_TRUE(store.remove("gone"));
  EXPECT_EQ(store.size(), 0u);

  std::string error;
  EXPECT_FALSE(store.remove("gone", &error));
  EXPECT_FALSE(error.empty());
}

TEST(WaypointStore, TagFilter)
{
  WaypointStore store;
  ArmWaypoint pruning = makeJointWaypoint("row3");
  pruning.tags = { "pruning", "row3" };
  ArmWaypoint parking = makeJointWaypoint("park");
  parking.tags = { "service" };
  ASSERT_TRUE(store.add(pruning, false));
  ASSERT_TRUE(store.add(parking, false));

  EXPECT_EQ(store.list("").size(), 2u);
  ASSERT_EQ(store.list("pruning").size(), 1u);
  EXPECT_EQ(store.list("pruning").front().name, "row3");
  EXPECT_TRUE(store.list("no_such_tag").empty());
}

/** A pose is only meaningful with the frame and the link it was taught for. On a mobile base,
 *  losing the frame turns a robot-relative target into a world one. */
TEST(WaypointStore, RoundTripKeepsFrameAndLink)
{
  WaypointStore store;
  store.default_group = "arm";
  ArmWaypoint waypoint = makePoseWaypoint("row3_approach", "robot_base");
  waypoint.description = "standoff in front of the row-3 cordon";
  waypoint.tags = { "pruning", "row3" };
  waypoint.tool = "cutter";
  waypoint.has_joint_state = true;
  waypoint.joint_state.name = { "J1", "J2" };
  waypoint.joint_state.position = { 0.12, -0.94 };
  ASSERT_TRUE(store.add(waypoint, false));

  WaypointStore reloaded;
  std::string error;
  ASSERT_TRUE(reloaded.loadFromText(store.toYaml(), &error)) << error;

  const ArmWaypoint* out = reloaded.find("row3_approach");
  ASSERT_NE(out, nullptr);
  EXPECT_EQ(out->group, "arm");
  EXPECT_EQ(out->description, "standoff in front of the row-3 cordon");
  EXPECT_EQ(out->tags, (std::vector<std::string>{ "pruning", "row3" }));
  EXPECT_EQ(out->tool, "cutter");

  ASSERT_TRUE(out->has_pose);
  EXPECT_EQ(out->pose.header.frame_id, "robot_base") << "the frame must survive the round trip";
  EXPECT_EQ(out->pose_link, "cutting_point") << "the link the pose describes must survive too";
  EXPECT_NEAR(out->pose.pose.position.x, 0.512, 1e-9);
  EXPECT_NEAR(out->pose.pose.orientation.w, 0.7071, 1e-9);

  ASSERT_TRUE(out->has_joint_state);
  ASSERT_EQ(out->joint_state.name.size(), 2u);
  EXPECT_EQ(out->joint_state.name[1], "J2");
  EXPECT_NEAR(out->joint_state.position[1], -0.94, 1e-9);
}

TEST(WaypointStore, CarrierDiagnosticsSurviveTheRoundTrip)
{
  WaypointStore store;
  ArmWaypoint waypoint = makeJointWaypoint("marginal");
  waypoint.has_carrier = true;
  waypoint.carrier.carrier_name = "wrist_triflex3d";
  waypoint.carrier.feasible = true;
  waypoint.carrier.bend_utilisation = 0.62;
  waypoint.carrier.cable_bend_ratio = 6.4;
  waypoint.carrier.required_cable_bend_ratio = 10.0;
  waypoint.carrier.worst_cable = "cutter_power_signal";
  waypoint.carrier.cables_within_limit = false;
  ASSERT_TRUE(store.add(waypoint, false));

  WaypointStore reloaded;
  ASSERT_TRUE(reloaded.loadFromText(store.toYaml()));
  const ArmWaypoint* out = reloaded.find("marginal");
  ASSERT_NE(out, nullptr);
  ASSERT_TRUE(out->has_carrier);
  EXPECT_EQ(out->carrier.worst_cable, "cutter_power_signal");
  EXPECT_NEAR(out->carrier.bend_utilisation, 0.62, 1e-9);
  EXPECT_FALSE(out->carrier.cables_within_limit)
      << "a target that was already outside the cable limit must still say so after a reload";
}

/** THE property that matters most: a file that will not parse must leave what is already loaded
 *  alone. Replacing a set of hand-taught targets with nothing, because of a stray character, would
 *  be unrecoverable. */
TEST(WaypointStore, AMalformedFileLeavesTheStoreUntouched)
{
  WaypointStore store;
  ASSERT_TRUE(store.add(makeJointWaypoint("precious"), false));

  std::string error;
  EXPECT_FALSE(store.loadFromText("waypoints: [ unclosed", &error));
  EXPECT_FALSE(error.empty());
  EXPECT_EQ(store.size(), 1u) << "the previous targets were lost";
  EXPECT_NE(store.find("precious"), nullptr);
}

TEST(WaypointStore, DuplicateNamesInAFileAreRejected)
{
  WaypointStore store;
  std::string error;
  EXPECT_FALSE(store.loadFromText(R"(
waypoints:
  - name: same
    group: arm
  - name: same
    group: arm
)",
                                  &error));
  EXPECT_NE(error.find("duplicate"), std::string::npos) << error;
}

TEST(WaypointStore, AWaypointWithoutANameIsRejected)
{
  WaypointStore store;
  std::string error;
  EXPECT_FALSE(store.loadFromText("waypoints:\n  - group: arm\n", &error));
  EXPECT_FALSE(error.empty());
}

TEST(WaypointStore, AnEmptyFileIsAnEmptyStoreNotAnError)
{
  WaypointStore store;
  std::string error;
  EXPECT_TRUE(store.loadFromText("", &error)) << error;
  EXPECT_EQ(store.size(), 0u);
}

/** A file that does not exist yet is what an empty store looks like before anything is taught, so
 *  it must not be reported as a failure -- otherwise a fresh robot logs an error on every start. */
TEST(WaypointStore, AMissingFileIsNotAnError)
{
  WaypointStore store;
  std::string error;
  EXPECT_TRUE(store.loadFromFile("/tmp/definitely_not_here_m2x.yaml", &error)) << error;
  EXPECT_EQ(store.size(), 0u);
}

TEST(WaypointStore, FileRoundTrip)
{
  TempFile file;
  WaypointStore store;
  store.default_group = "arm";
  store.default_frame_id = "robot_base";
  store.default_pose_link = "cutting_point";
  ASSERT_TRUE(store.add(makePoseWaypoint("a", "robot_base"), false));
  ASSERT_TRUE(store.add(makeJointWaypoint("b"), false));

  std::string error;
  ASSERT_TRUE(store.saveToFile(file.str(), &error)) << error;

  WaypointStore reloaded;
  ASSERT_TRUE(reloaded.loadFromFile(file.str(), &error)) << error;
  EXPECT_EQ(reloaded.size(), 2u);
  EXPECT_EQ(reloaded.default_frame_id, "robot_base") << "the file's defaults must survive";
  EXPECT_EQ(reloaded.default_pose_link, "cutting_point");
}

TEST(WaypointStore, WaypointsInheritTheFileDefaults)
{
  WaypointStore store;
  std::string error;
  ASSERT_TRUE(store.loadFromText(R"(
default_group: arm
default_frame_id: robot_base
default_pose_link: cutting_point
waypoints:
  - name: inherits
    pose:
      position: {x: 0.1, y: 0.0, z: 0.2}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
)",
                                 &error))
      << error;

  const ArmWaypoint* out = store.find("inherits");
  ASSERT_NE(out, nullptr);
  EXPECT_EQ(out->group, "arm");
  EXPECT_EQ(out->pose.header.frame_id, "robot_base");
  EXPECT_EQ(out->pose_link, "cutting_point");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
