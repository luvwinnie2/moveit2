// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// The library is what decides whether a broken Objective file takes the whole server down with
// it. It must not: a typo in one XML has to cost that one Objective and nothing else.
#include <moveit2_extended_core/objective_library.hpp>
#include <moveit2_extended_core/path_utils.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

using moveit2_extended::BtFactory;
using moveit2_extended::ObjectiveLibrary;
namespace fs = std::filesystem;

namespace
{
class Echo : public BT::SyncActionNode
{
public:
  Echo(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("text", "", "") };
  }
  BT::NodeStatus tick() override
  {
    return BT::NodeStatus::SUCCESS;
  }
};

BtFactory makeFactory()
{
  BtFactory factory;
  factory.registerNodeType<Echo>("Echo");
  return factory;
}

/** Scratch directory that cleans up after itself, so tests do not leave files behind or see each
 *  other's. */
class TempDir
{
public:
  TempDir()
  {
    path_ = fs::temp_directory_path() / fs::path("m2x_objlib_" + std::to_string(::getpid()) + "_" +
                                                 std::to_string(counter_++));
    fs::create_directories(path_);
  }
  ~TempDir()
  {
    std::error_code ec;
    fs::remove_all(path_, ec);
  }
  const fs::path& path() const
  {
    return path_;
  }
  void write(const std::string& name, const std::string& content) const
  {
    std::ofstream out(path_ / name);
    out << content;
  }

private:
  fs::path path_;
  static inline int counter_ = 0;
};

constexpr const char* kGoodXml = R"(<root main_tree_to_execute="Good">
  <BehaviorTree ID="Good">
    <Echo name="hi" text="hello"/>
  </BehaviorTree>
</root>)";
}  // namespace

TEST(ObjectiveLibrary, LoadsAndRegisters)
{
  TempDir dir;
  dir.write("good.xml", kGoodXml);

  auto factory = makeFactory();
  ObjectiveLibrary library;
  std::vector<std::string> errors;
  EXPECT_TRUE(library.load(factory, { dir.path().string() }, false, &errors));
  EXPECT_TRUE(errors.empty());

  EXPECT_EQ(library.names(), (std::vector<std::string>{ "Good" }));
  ASSERT_NE(library.find("Good"), nullptr);
  EXPECT_EQ(library.find("Good")->file, (dir.path() / "good.xml").string());

  // Registered means createTree() works, which is the only thing that matters downstream.
  EXPECT_NO_THROW({ auto tree = factory.createTree("Good"); });
}

/** One malformed file must not stop the others loading. Otherwise a single typo takes every
 *  Objective on the robot offline. */
TEST(ObjectiveLibrary, OneBadFileDoesNotStopTheRest)
{
  TempDir dir;
  dir.write("good.xml", kGoodXml);
  dir.write("broken.xml", "<root><BehaviorTree ID=\"Broken\"><Sequence></root>");

  auto factory = makeFactory();
  ObjectiveLibrary library;
  std::vector<std::string> errors;
  EXPECT_FALSE(library.load(factory, { dir.path().string() }, false, &errors))
      << "load() must report that something was skipped";
  EXPECT_FALSE(errors.empty()) << "and say which file";

  EXPECT_NE(library.find("Good"), nullptr) << "the good objective still loaded";
}

TEST(ObjectiveLibrary, MissingBehaviorsAreReportedPerObjective)
{
  TempDir dir;
  dir.write("needs.xml", R"(<root main_tree_to_execute="Needs">
  <BehaviorTree ID="Needs">
    <Sequence>
      <Echo name="ok" text="x"/>
      <NotRegistered name="nope"/>
    </Sequence>
  </BehaviorTree>
</root>)");

  auto factory = makeFactory();
  ObjectiveLibrary library;
  library.load(factory, { dir.path().string() }, false, nullptr);

  // BehaviorTree.CPP refuses to register a tree that references an unknown node, so this
  // Objective is NOT buildable. It must still be listed, with an explanation -- an Objective that
  // silently disappears because one Behavior package failed to load is far harder to diagnose
  // than one that says what it is waiting for.
  const auto* entry = library.find("Needs");
  ASSERT_NE(entry, nullptr) << "an unrunnable objective must still be known about";
  EXPECT_FALSE(entry->runnable);

  const auto missing = library.missingBehaviors(factory, "Needs");
  ASSERT_EQ(missing.size(), 1u);
  EXPECT_EQ(missing.front(), "NotRegistered");

  const auto info = library.describe(factory, *entry);
  EXPECT_EQ(info.missing_behaviors, missing);
}

TEST(ObjectiveLibrary, RunnableObjectivesAreMarkedRunnable)
{
  TempDir dir;
  dir.write("good.xml", kGoodXml);

  auto factory = makeFactory();
  ObjectiveLibrary library;
  library.load(factory, { dir.path().string() }, false, nullptr);

  ASSERT_NE(library.find("Good"), nullptr);
  EXPECT_TRUE(library.find("Good")->runnable);
}

TEST(ObjectiveLibrary, SidecarSuppliesMetadata)
{
  TempDir dir;
  dir.write("documented.xml", R"(<root main_tree_to_execute="Documented">
  <BehaviorTree ID="Documented"><Echo name="a" text="x"/></BehaviorTree>
</root>)");
  dir.write("documented.yaml", R"(
description: what this objective is for
required_parameters: [target]
optional_parameters: [speed, tolerance]
)");

  auto factory = makeFactory();
  ObjectiveLibrary library;
  library.load(factory, { dir.path().string() }, false, nullptr);

  const auto* entry = library.find("Documented");
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->description, "what this objective is for");
  EXPECT_EQ(entry->required_parameters, (std::vector<std::string>{ "target" }));
  EXPECT_EQ(entry->optional_parameters, (std::vector<std::string>{ "speed", "tolerance" }));
}

/** A malformed sidecar costs documentation, not the Objective. */
TEST(ObjectiveLibrary, BrokenSidecarDoesNotBreakTheObjective)
{
  TempDir dir;
  dir.write("x.xml", R"(<root main_tree_to_execute="X">
  <BehaviorTree ID="X"><Echo name="a" text="y"/></BehaviorTree>
</root>)");
  dir.write("x.yaml", "description: [unclosed\n");

  auto factory = makeFactory();
  ObjectiveLibrary library;
  library.load(factory, { dir.path().string() }, false, nullptr);
  EXPECT_NE(library.find("X"), nullptr);
}

TEST(ObjectiveLibrary, SubtreesInTheSameFileAreAddressable)
{
  TempDir dir;
  dir.write("composed.xml", R"(<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence><SubTree ID="Helper"/></Sequence>
  </BehaviorTree>
  <BehaviorTree ID="Helper">
    <Echo name="inner" text="z"/>
  </BehaviorTree>
</root>)");

  auto factory = makeFactory();
  ObjectiveLibrary library;
  library.load(factory, { dir.path().string() }, false, nullptr);

  EXPECT_NE(library.find("Main"), nullptr);
  EXPECT_NE(library.find("Helper"), nullptr) << "a subtree must be runnable on its own too";
  EXPECT_EQ(library.find("Main")->subtree_ids, (std::vector<std::string>{ "Helper" }));
}

TEST(ObjectiveLibrary, DuplicateTreeIdIsReported)
{
  TempDir dir;
  dir.write("a.xml", kGoodXml);
  dir.write("b.xml", kGoodXml);  // same tree ID in a second file

  auto factory = makeFactory();
  ObjectiveLibrary library;
  std::vector<std::string> errors;
  library.load(factory, { dir.path().string() }, false, &errors);

  bool mentions_duplicate = false;
  for (const auto& error : errors)
  {
    mentions_duplicate = mentions_duplicate || error.find("already defined") != std::string::npos;
  }
  EXPECT_TRUE(mentions_duplicate) << "a duplicate tree ID must be named, not silently resolved";
}

TEST(ObjectiveLibrary, MissingDirectoryIsReportedNotFatal)
{
  auto factory = makeFactory();
  ObjectiveLibrary library;
  std::vector<std::string> errors;
  EXPECT_FALSE(library.load(factory, { "/definitely/not/here" }, false, &errors));
  EXPECT_FALSE(errors.empty());
  EXPECT_EQ(library.size(), 0u);
}

TEST(ObjectiveLibrary, ClearUnregisters)
{
  TempDir dir;
  dir.write("good.xml", kGoodXml);

  auto factory = makeFactory();
  ObjectiveLibrary library;
  library.load(factory, { dir.path().string() }, false, nullptr);
  ASSERT_EQ(library.size(), 1u);

  library.clear(factory);
  EXPECT_EQ(library.size(), 0u);
  EXPECT_TRUE(factory.registeredBehaviorTrees().empty());
}

TEST(PathUtils, ResolvesNonPackagePathsUnchanged)
{
  EXPECT_EQ(moveit2_extended::resolvePackageUri("/absolute/path"), "/absolute/path");
  EXPECT_EQ(moveit2_extended::resolvePackageUri("relative/path"), "relative/path");
}

TEST(PathUtils, UnknownPackageIsReported)
{
  std::string error;
  EXPECT_TRUE(moveit2_extended::resolvePackageUri("package://no_such_package_xyz/f.xml", &error).empty());
  EXPECT_FALSE(error.empty());
}

TEST(PathUtils, ReadWriteRoundTrip)
{
  TempDir dir;
  const std::string path = (dir.path() / "sub" / "file.txt").string();
  std::string error;
  ASSERT_TRUE(moveit2_extended::writeFile(path, "hello", &error)) << error;
  EXPECT_EQ(moveit2_extended::readFile(path), "hello");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
