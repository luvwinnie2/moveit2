// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// The panel's real failure modes are threading mistakes, and neither one shows up in a functional
// test: touching a Qt widget from the ROS executor thread usually survives every test you write and
// then crashes in front of the customer, and waiting on a future inside RViz deadlocks the whole
// application because RViz's executor is the thread that would have to deliver the response.
//
// Neither is catchable at run time here -- RViz is not running -- but both ARE mechanically
// checkable in the source, so that is what this does: pull out every ROS callback body and assert
// it only takes the mutex and stores a value.

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace
{
/** Widget members. Any of these appearing inside a ROS callback is the bug this test exists for. */
const std::vector<std::string> kWidgets = {
  "status_->",   "current_->",      "prompt_box_->",  "prompt_message_->",       "prompt_buttons_->",
  "tabs_->",     "objective_list_->", "objective_description_->", "parameters_->", "run_->",
  "cancel_->",   "result_->",       "tree_->",        "waypoint_list_->",        "waypoint_name_->",
  "tool_list_->", "teach_status_->", "log_->",        "appendLog(",
};

/** Anything that parks the calling thread until ROS does something. */
const std::vector<std::string> kBlocking = {
  "spin_until_future_complete", "spin_some", "spin_once", ".wait_for(", "->wait_for(", ".wait()",
};

std::string readSource()
{
  std::ifstream file(PANEL_SOURCE);
  EXPECT_TRUE(file.good()) << "cannot open " << PANEL_SOURCE;
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

/** Body of the lambda that starts at or after `from`, brace-matched. Empty if there is none. */
std::string lambdaBodyAfter(const std::string& source, size_t from)
{
  const size_t capture = source.find("[this]", from);
  if (capture == std::string::npos)
  {
    return {};
  }
  const size_t open = source.find('{', capture);
  if (open == std::string::npos)
  {
    return {};
  }
  int depth = 0;
  for (size_t i = open; i < source.size(); ++i)
  {
    if (source[i] == '{')
    {
      ++depth;
    }
    else if (source[i] == '}')
    {
      if (--depth == 0)
      {
        return source.substr(open, i - open + 1);
      }
    }
  }
  return {};
}

/** Every lambda that ROS -- not Qt -- will call. */
std::vector<std::string> rosCallbackBodies(const std::string& source)
{
  const std::vector<std::string> anchors = {
    "create_subscription<", "async_send_request(", "goal_response_callback =", "feedback_callback =",
    "result_callback =",
  };
  std::vector<std::string> bodies;
  for (const auto& anchor : anchors)
  {
    size_t at = 0;
    while ((at = source.find(anchor, at)) != std::string::npos)
    {
      const std::string body = lambdaBodyAfter(source, at + anchor.size());
      if (!body.empty())
      {
        bodies.push_back(body);
      }
      at += anchor.size();
    }
  }
  return bodies;
}
}  // namespace

TEST(PanelThreadRules, TheScannerActuallyFindsTheCallbacks)
{
  // Without this the two tests below would pass on an empty list and prove nothing.
  const auto bodies = rosCallbackBodies(readSource());
  EXPECT_GE(bodies.size(), 8u) << "found only " << bodies.size()
                               << " ROS callbacks; the anchors in this test have gone stale";
}

TEST(PanelThreadRules, NoRosCallbackTouchesAWidget)
{
  const auto bodies = rosCallbackBodies(readSource());
  for (const auto& body : bodies)
  {
    for (const auto& widget : kWidgets)
    {
      EXPECT_EQ(body.find(widget), std::string::npos)
          << "a ROS callback touches " << widget << " -- Qt widgets may only be used from the GUI "
          << "thread. Cache the value under mutex_ and let refresh() paint it.\nCallback:\n"
          << body;
    }
  }
}

TEST(PanelThreadRules, NoRosCallbackTakesTheMutexTwice)
{
  // A callback that locks, then calls a helper that locks again, deadlocks on a non-recursive
  // mutex. One lock_guard per callback is the rule.
  const auto bodies = rosCallbackBodies(readSource());
  for (const auto& body : bodies)
  {
    size_t count = 0;
    size_t at = 0;
    while ((at = body.find("lock_guard", at)) != std::string::npos)
    {
      ++count;
      at += 1;
    }
    EXPECT_LE(count, 1u) << "two lock_guards in one callback deadlocks on std::mutex:\n" << body;
  }
}

TEST(PanelThreadRules, NothingInThePanelBlocksOnRos)
{
  const std::string source = readSource();
  for (const auto& blocking : kBlocking)
  {
    EXPECT_EQ(source.find(blocking), std::string::npos)
        << "the panel calls " << blocking
        << ". RViz's executor is the thread that delivers ROS responses, so blocking on one from "
           "the GUI thread deadlocks RViz.";
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
