// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// Guards the version firewall in bt_compat.hpp.
//
// The value of that header is entirely in the discipline it enforces: if a v3-only spelling leaks
// into a Behavior, migrating to BehaviorTree.CPP v4 stops being "edit one file" and becomes a
// search across every source in the project. A comment asking people not to do it will not hold;
// this test will.
//
// Scope is src/ and include/ only. Tests are allowed to construct plain BehaviorTree.CPP nodes,
// which legitimately need BT::NodeConfiguration in their constructors.
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{
struct BannedSpelling
{
  const char* pattern;
  const char* why;
};

const std::vector<BannedSpelling> kBanned = {
  { R"(\bBT::NodeConfiguration\b)", "use moveit2_extended::NodeConfig (v4 renames this to BT::NodeConfig)" },
  { R"(\bBT::Optional\b)", "use moveit2_extended::BtExpected (v4 renames this to BT::Expected)" },
  { R"(\btickRoot\s*\()", "use moveit2_extended::tickOnce (v4 renames this to Tree::tickOnce)" },
  { R"(\btickRootWhileRunning\s*\()", "tick from the server loop via tickOnce instead" },
};

/** True for a line that is entirely a comment.
 *
 *  The rule is about code, not prose: explaining *why* tickRoot() must not be called is exactly
 *  the sort of comment this project wants, and a check that forbade naming the thing it bans
 *  would push that explanation out of the codebase. Good enough for this style, where every
 *  comment is a full line. */
bool isCommentLine(const std::string& line)
{
  const size_t first = line.find_first_not_of(" \t");
  if (first == std::string::npos)
  {
    return true;
  }
  return line.compare(first, 2, "//") == 0 || line.compare(first, 2, "/*") == 0 ||
         line.compare(first, 1, "*") == 0;
}

std::string readAll(const fs::path& path)
{
  std::ifstream in(path);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::vector<fs::path> sourcesToCheck()
{
  std::vector<fs::path> files;
  const fs::path root(SOURCE_ROOT);
  for (const char* subdir : { "src", "include" })
  {
    const fs::path dir = root / subdir;
    std::error_code ec;
    if (!fs::exists(dir, ec))
    {
      continue;
    }
    for (fs::recursive_directory_iterator it(dir, ec), end; it != end && !ec; it.increment(ec))
    {
      if (!it->is_regular_file(ec))
      {
        continue;
      }
      const auto extension = it->path().extension().string();
      if (extension != ".cpp" && extension != ".hpp" && extension != ".h")
      {
        continue;
      }
      // The firewall itself is the one place these names are allowed.
      if (it->path().filename() == "bt_compat.hpp")
      {
        continue;
      }
      files.push_back(it->path());
    }
  }
  return files;
}
}  // namespace

TEST(BtCompatFirewall, NoVersionSpecificSpellingsOutsideTheCompatHeader)
{
  const auto files = sourcesToCheck();
  ASSERT_FALSE(files.empty()) << "found no sources to check under " << SOURCE_ROOT
                              << "; the test is not actually guarding anything";

  std::vector<std::string> violations;
  for (const auto& file : files)
  {
    const std::string content = readAll(file);
    std::istringstream lines(content);
    std::string line;
    int number = 0;
    while (std::getline(lines, line))
    {
      ++number;
      if (isCommentLine(line))
      {
        continue;
      }
      for (const auto& banned : kBanned)
      {
        if (std::regex_search(line, std::regex(banned.pattern)))
        {
          violations.push_back(file.string() + ":" + std::to_string(number) + ": " + banned.why);
        }
      }
    }
  }

  std::string report;
  for (const auto& violation : violations)
  {
    report += "\n  " + violation;
  }
  EXPECT_TRUE(violations.empty()) << "BehaviorTree.CPP v3-specific spellings escaped bt_compat.hpp:" << report;
}

/** The header must actually be present and readable, or the test above passes vacuously the day
 *  somebody renames it. */
TEST(BtCompatFirewall, CompatHeaderExists)
{
  const fs::path header = fs::path(SOURCE_ROOT) / "include" / "moveit2_extended_core" / "bt_compat.hpp";
  ASSERT_TRUE(fs::exists(header)) << header << " is missing";
  const std::string content = readAll(header);
  EXPECT_NE(content.find("tickOnce"), std::string::npos);
  EXPECT_NE(content.find("NodeConfig"), std::string::npos);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
