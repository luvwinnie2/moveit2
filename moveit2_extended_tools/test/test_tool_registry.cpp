// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// The registry decides what the planner believes is bolted to the end of the arm, so the tests
// that matter are the ones about refusing to describe a tool wrongly: no collision geometry, a
// zero-sized shape, or a missing mount link. Each of those, accepted, produces a planner that
// happily drives a gripper through something.
#include <moveit2_extended_tools/tool_registry.hpp>

#include <gtest/gtest.h>

#include <cmath>

using moveit2_extended::tools::ToolRegistry;

namespace
{
constexpr const char* kMinimal = R"(
mount_link: flange
tools:
  - name: gripper
    description: a gripper
    mass: 0.9
    tcp_offset:
      xyz: [0.15, 0.0, 0.02]
      rpy: [0.0, 0.0, 0.0]
    allowed_collision_links: [flange, J6_link]
    carrier_config: package://x/y.yaml
    collision:
      - type: box
        dimensions: [0.09, 0.085, 0.09]
        origin: { xyz: [0.045, 0.0, 0.0], rpy: [0.0, 0.0, 0.0] }
)";
}  // namespace

TEST(ToolRegistry, LoadsAToolWithEverything)
{
  ToolRegistry registry;
  std::string error;
  ASSERT_TRUE(registry.loadFromText(kMinimal, &error)) << error;

  EXPECT_EQ(registry.mount_link, "flange");
  ASSERT_EQ(registry.size(), 1u);

  const auto* tool = registry.find("gripper");
  ASSERT_NE(tool, nullptr);
  EXPECT_EQ(tool->info.description, "a gripper");
  EXPECT_DOUBLE_EQ(tool->info.mass, 0.9);
  EXPECT_NEAR(tool->info.tcp_offset.position.x, 0.15, 1e-9);
  EXPECT_NEAR(tool->info.tcp_offset.orientation.w, 1.0, 1e-9);
  EXPECT_EQ(tool->info.allowed_collision_links, (std::vector<std::string>{ "flange", "J6_link" }));
  EXPECT_EQ(tool->info.carrier_config, "package://x/y.yaml");

  ASSERT_EQ(tool->shapes.size(), 1u);
  EXPECT_EQ(tool->shapes.front().primitive.type, shape_msgs::msg::SolidPrimitive::BOX);
  ASSERT_EQ(tool->shapes.front().primitive.dimensions.size(), 3u);
  EXPECT_NEAR(tool->shapes.front().primitive.dimensions[0], 0.09, 1e-9);
  EXPECT_NEAR(tool->shapes.front().pose.position.x, 0.045, 1e-9);
}

TEST(ToolRegistry, RpyBecomesAQuaternion)
{
  ToolRegistry registry;
  ASSERT_TRUE(registry.loadFromText(R"(
mount_link: flange
tools:
  - name: turned
    tcp_offset: { xyz: [0.1, 0.0, 0.0], rpy: [0.0, 1.5707963, 0.0] }
    collision:
      - type: sphere
        dimensions: [0.05]
)"));
  const auto* tool = registry.find("turned");
  ASSERT_NE(tool, nullptr);
  // A 90 degree pitch is (0, sqrt(2)/2, 0, sqrt(2)/2).
  EXPECT_NEAR(tool->info.tcp_offset.orientation.y, std::sqrt(2.0) / 2.0, 1e-6);
  EXPECT_NEAR(tool->info.tcp_offset.orientation.w, std::sqrt(2.0) / 2.0, 1e-6);
}

/** A tool with no collision geometry is invisible to the planner, which will then plan straight
 *  through whatever it is holding. */
TEST(ToolRegistry, AToolWithoutCollisionGeometryIsRefused)
{
  ToolRegistry registry;
  std::string error;
  EXPECT_FALSE(registry.loadFromText(R"(
mount_link: flange
tools:
  - name: invisible
    tcp_offset: { xyz: [0.1, 0.0, 0.0] }
)",
                                     &error));
  EXPECT_NE(error.find("no collision shapes"), std::string::npos) << error;
}

/** A zero-sized shape is worse than no shape: it registers as geometry and collides with nothing. */
TEST(ToolRegistry, ZeroSizedShapesAreRefused)
{
  ToolRegistry registry;
  std::string error;
  EXPECT_FALSE(registry.loadFromText(R"(
mount_link: flange
tools:
  - name: flat
    collision:
      - type: box
        dimensions: [0.1, 0.0, 0.1]
)",
                                     &error));
  EXPECT_NE(error.find("greater than zero"), std::string::npos) << error;
}

TEST(ToolRegistry, WrongDimensionCountIsRefused)
{
  ToolRegistry registry;
  std::string error;
  EXPECT_FALSE(registry.loadFromText(R"(
mount_link: flange
tools:
  - name: wrong
    collision:
      - type: cylinder
        dimensions: [0.1]
)",
                                     &error));
  EXPECT_NE(error.find("2 dimension"), std::string::npos) << error;
}

TEST(ToolRegistry, MissingMountLinkIsRefused)
{
  ToolRegistry registry;
  std::string error;
  EXPECT_FALSE(registry.loadFromText("tools: []\n", &error));
  EXPECT_NE(error.find("mount_link"), std::string::npos) << error;
}

TEST(ToolRegistry, DuplicateToolNamesAreRefused)
{
  ToolRegistry registry;
  std::string error;
  EXPECT_FALSE(registry.loadFromText(R"(
mount_link: flange
tools:
  - name: same
    collision: [{ type: sphere, dimensions: [0.05] }]
  - name: same
    collision: [{ type: sphere, dimensions: [0.05] }]
)",
                                     &error));
  EXPECT_NE(error.find("duplicate"), std::string::npos) << error;
}

/** A bad entry must not leave the registry describing half of one configuration. */
TEST(ToolRegistry, AFailedLoadLeavesThePreviousRegistryIntact)
{
  ToolRegistry registry;
  ASSERT_TRUE(registry.loadFromText(kMinimal));
  ASSERT_EQ(registry.size(), 1u);

  EXPECT_FALSE(registry.loadFromText("mount_link: flange\ntools:\n  - name: broken\n"));
  EXPECT_EQ(registry.size(), 1u) << "the previous tools were lost";
  EXPECT_NE(registry.find("gripper"), nullptr);
}

TEST(ToolRegistry, UnknownShapeTypeIsRefused)
{
  ToolRegistry registry;
  std::string error;
  EXPECT_FALSE(registry.loadFromText(R"(
mount_link: flange
tools:
  - name: odd
    collision: [{ type: dodecahedron, dimensions: [0.05] }]
)",
                                     &error));
  EXPECT_NE(error.find("unknown shape type"), std::string::npos) << error;
}

/** The shipped registry has to load, and every entry has to declare where its numbers came from --
 *  a bounding box inferred from a datasheet must not be mistaken for a measured hull. */
TEST(ToolRegistry, TheShippedCrx5iaRegistryLoadsAndDeclaresProvenance)
{
  ToolRegistry registry;
  std::string error;
  ASSERT_TRUE(registry.loadFromFile(std::string(CONFIG_DIR) + "/crx5ia_tools.yaml", &error)) << error;

  EXPECT_EQ(registry.mount_link, "flange");
  for (const char* name : { "cutter", "robotiq_2f85", "robotiq_2f140", "robotiq_hande" })
  {
    const auto* tool = registry.find(name);
    ASSERT_NE(tool, nullptr) << name << " is missing from the shipped registry";
    EXPECT_FALSE(tool->shapes.empty()) << name << " has no collision geometry";
    EXPECT_TRUE(tool->provenance == "measured" || tool->provenance == "datasheet" ||
                tool->provenance == "estimate")
        << name << " has provenance '" << tool->provenance << "'";
  }

  // The cutter's offset is the one this robot's kinematics already depends on, so it is pinned.
  const auto* cutter = registry.find("cutter");
  ASSERT_NE(cutter, nullptr);
  EXPECT_NEAR(cutter->info.tcp_offset.position.x, 0.150, 1e-9);
  EXPECT_NEAR(cutter->info.tcp_offset.position.z, 0.028, 1e-9);
  EXPECT_EQ(cutter->provenance, "measured");

  // The 2F-140 must be bigger and heavier than the 2F-85, or the comparison it exists for is
  // meaningless.
  const auto* small = registry.find("robotiq_2f85");
  const auto* large = registry.find("robotiq_2f140");
  ASSERT_NE(small, nullptr);
  ASSERT_NE(large, nullptr);
  EXPECT_GT(large->info.mass, small->info.mass);
  EXPECT_GT(large->info.tcp_offset.position.x, small->info.tcp_offset.position.x);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
