// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_behaviors/ui_behaviors.hpp>

#include <moveit2_extended_core/blackboard_text.hpp>

#include <moveit/robot_state/conversions.h>

#include <algorithm>
#include <random>

namespace moveit2_extended::behaviors
{
namespace
{
constexpr const char* kPromptTopic = "/objective/user_prompt";
constexpr const char* kResponseTopic = "/objective/user_response";

rclcpp::QoS latchedQos()
{
  return rclcpp::QoS(1).transient_local().reliable();
}

/** Distinct enough to tell one prompt from the next; not a security token. */
std::string makePromptId(const std::string& behavior_name, uint64_t counter)
{
  return behavior_name + "_" + std::to_string(counter);
}
}  // namespace

// ---------------------------------------------------------------------------------------------
// UserPromptBehaviorBase
// ---------------------------------------------------------------------------------------------

UserPromptBehaviorBase::UserPromptBehaviorBase(const std::string& name, const NodeConfig& config,
                                               BehaviorContextPtr shared_resources)
  : SharedResourcesNode<BT::StatefulActionNode>(name, config, std::move(shared_resources))
{
  island_ = getSharedResources()->makeCallbackIsland();
  prompt_pub_ = getNode()->create_publisher<moveit2_extended_msgs::msg::UserPrompt>(kPromptTopic, latchedQos());
}

BT::PortsList UserPromptBehaviorBase::providedBasicPorts(BT::PortsList addition)
{
  // No declared default on the list ports. BehaviorTree.CPP renders a port's default with
  // to_string(), and there is no such overload for std::vector<std::string> -- so declaring one
  // does not compile. The effective defaults live in the getInputOr calls below and are stated in
  // the descriptions instead.
  BT::PortsList basic = {
    BT::InputPort<std::string>("prompt", "", "what to ask"),
    BT::InputPort<std::vector<std::string>>("choices", "one button per entry; default 'Approve;Reject'"),
    BT::InputPort<std::vector<std::string>>("success_choices",
                                            "which answers mean SUCCESS; default 'Approve'"),
    BT::InputPort<double>("timeout", 0.0, "seconds before giving up; 0 waits for ever"),
    BT::OutputPort<std::string>("choice", "what the operator picked"),
  };
  basic.insert(addition.begin(), addition.end());
  return basic;
}

BtStatus UserPromptBehaviorBase::onStart()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    choice_.reset();
  }

  auto prompt = createPrompt();
  if (!prompt)
  {
    RCLCPP_ERROR(getLogger(), "cannot build the prompt: %s", prompt.error().c_str());
    return BtStatus::FAILURE;
  }

  static uint64_t counter = 0;
  prompt_id_ = makePromptId(name(), ++counter);
  prompt->prompt_id = prompt_id_;
  timeout_s_ = getInputOr<double>("timeout", 0.0);
  prompt->timeout = timeout_s_;

  // Subscribe before publishing, so an unusually quick answer cannot arrive before we are
  // listening for it.
  rclcpp::SubscriptionOptions options;
  options.callback_group = island_.group;
  response_sub_ = getNode()->create_subscription<moveit2_extended_msgs::msg::UserResponse>(
      kResponseTopic, rclcpp::QoS(10),
      [this](const moveit2_extended_msgs::msg::UserResponse::SharedPtr message) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          // A stale answer to a prompt we already withdrew must not be mistaken for an answer to
          // this one.
          if (message->prompt_id != prompt_id_)
          {
            return;
          }
          choice_ = message->choice;
        }
        emitStateChanged();
      },
      options);

  prompt_pub_->publish(*prompt);
  publishContext();

  RCLCPP_INFO(getLogger(), "waiting for the operator: %s", prompt->message.c_str());
  started_at_ = std::chrono::steady_clock::now();
  return BtStatus::RUNNING;
}

BtStatus UserPromptBehaviorBase::onRunning()
{
  island_.pump();

  std::optional<std::string> choice;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    choice = choice_;
  }

  if (choice)
  {
    setOutput("choice", *choice);
    withdrawPrompt();
    return processChoice(*choice);
  }

  if (timeout_s_ > 0.0)
  {
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at_).count();
    if (elapsed > timeout_s_)
    {
      RCLCPP_WARN(getLogger(), "no answer within %.1f s", timeout_s_);
      withdrawPrompt();
      return BtStatus::FAILURE;
    }
  }
  return BtStatus::RUNNING;
}

void UserPromptBehaviorBase::onHalted()
{
  withdrawPrompt();
}

void UserPromptBehaviorBase::withdrawPrompt()
{
  // An empty prompt_id means "the previous question is withdrawn", so a cancelled Objective does
  // not leave a dialog on screen waiting for an answer that will not be acted on.
  moveit2_extended_msgs::msg::UserPrompt cleared;
  prompt_pub_->publish(cleared);
  response_sub_.reset();
  prompt_id_.clear();
}

BtStatus UserPromptBehaviorBase::processChoice(const std::string& choice)
{
  const auto accepted = getInputOr<std::vector<std::string>>("success_choices", std::vector<std::string>{ "Approve" });
  const bool ok = std::find(accepted.begin(), accepted.end(), choice) != accepted.end();
  RCLCPP_INFO(getLogger(), "operator chose '%s'", choice.c_str());
  return ok ? BtStatus::SUCCESS : BtStatus::FAILURE;
}

// ---------------------------------------------------------------------------------------------
// GetTextFromUser
// ---------------------------------------------------------------------------------------------

GetTextFromUser::GetTextFromUser(const std::string& name, const NodeConfig& config,
                                 BehaviorContextPtr shared_resources)
  : UserPromptBehaviorBase(name, config, std::move(shared_resources))
{
}

BT::PortsList GetTextFromUser::providedPorts()
{
  return providedBasicPorts({});
}

BtExpected<moveit2_extended_msgs::msg::UserPrompt> GetTextFromUser::createPrompt()
{
  moveit2_extended_msgs::msg::UserPrompt prompt;
  // Expanded rather than used raw: this string is the only thing the operator sees, and BT leaves
  // an embedded {reference} untouched. "Bend utilisation {worst_bend}" is not a question anyone
  // can answer.
  prompt.message =
      expandBlackboardReferences(config().blackboard, getInputOr<std::string>("prompt", std::string("Continue?")));
  prompt.choices = getInputOr<std::vector<std::string>>("choices", std::vector<std::string>{ "Approve", "Reject" });
  prompt.has_trajectory = false;
  return prompt;
}

BtStatus GetTextFromUser::processChoice(const std::string& choice)
{
  return UserPromptBehaviorBase::processChoice(choice);
}

// ---------------------------------------------------------------------------------------------
// WaitForUserTrajectoryApproval
// ---------------------------------------------------------------------------------------------

WaitForUserTrajectoryApproval::WaitForUserTrajectoryApproval(const std::string& name, const NodeConfig& config,
                                                             BehaviorContextPtr shared_resources)
  : UserPromptBehaviorBase(name, config, std::move(shared_resources))
{
  preview_pub_ = getNode()->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/objective/preview_trajectory",
                                                                                   latchedQos());
}

BT::PortsList WaitForUserTrajectoryApproval::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory", "what the operator is being asked to approve"),
  });
}

BtExpected<moveit2_extended_msgs::msg::UserPrompt> WaitForUserTrajectoryApproval::createPrompt()
{
  const auto trajectory = getInput<moveit_msgs::msg::RobotTrajectory>("trajectory");
  if (!trajectory)
  {
    return nonstd::make_unexpected("trajectory: " + trajectory.error());
  }

  moveit2_extended_msgs::msg::UserPrompt prompt;
  prompt.message = expandBlackboardReferences(config().blackboard,
                                              getInputOr<std::string>("prompt", std::string("Run this trajectory?")));
  prompt.choices = getInputOr<std::vector<std::string>>("choices", std::vector<std::string>{ "Approve", "Reject" });
  prompt.has_trajectory = true;
  return prompt;
}

void WaitForUserTrajectoryApproval::publishContext()
{
  const auto trajectory = getInput<moveit_msgs::msg::RobotTrajectory>("trajectory");
  const auto current = getSharedResources()->currentState();
  if (!trajectory || !current)
  {
    return;
  }

  // Published so RViz's Trajectory display, or the Studio's 3D view, can show what is being
  // approved. Approving a motion you cannot see is not approval.
  moveit_msgs::msg::DisplayTrajectory display;
  display.trajectory = { trajectory.value() };
  moveit::core::robotStateToRobotStateMsg(*current, display.trajectory_start);
  preview_pub_->publish(display);
}

// ---------------------------------------------------------------------------------------------
// IsUserAvailable
// ---------------------------------------------------------------------------------------------

IsUserAvailable::IsUserAvailable(const std::string& name, const NodeConfig& config,
                                 BehaviorContextPtr shared_resources)
  : ConditionBehaviorBase(name, config, std::move(shared_resources))
{
}

BT::PortsList IsUserAvailable::providedPorts()
{
  return {
    BT::InputPort<std::string>("ui_node_name", "studio_bridge",
                               "the node whose presence means somebody is watching"),
  };
}

BtStatus IsUserAvailable::tick()
{
  const auto wanted = getInputOr<std::string>("ui_node_name", std::string("studio_bridge"));
  // Presence of the bridge node, not of a subscriber on the prompt topic: a latched publisher
  // reports subscribers that have gone away, so counting those would say "somebody is watching"
  // long after the browser was closed.
  for (const auto& node : getNode()->get_node_names())
  {
    // get_node_names returns fully qualified names, so match the leaf.
    const size_t slash = node.find_last_of('/');
    const std::string leaf = slash == std::string::npos ? node : node.substr(slash + 1);
    if (leaf == wanted)
    {
      return BtStatus::SUCCESS;
    }
  }
  return BtStatus::FAILURE;
}

}  // namespace moveit2_extended::behaviors
