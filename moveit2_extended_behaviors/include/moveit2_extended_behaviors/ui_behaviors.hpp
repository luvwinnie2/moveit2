// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

// Behaviors that stop and ask the operator something.
//
// This is how the Studio is driven, and it is the same mechanism MoveIt Pro uses: the tree blocks
// on a Behavior, that Behavior publishes a prompt, and the UI answers it. The UI therefore needs
// no knowledge of what the tree is doing -- it renders whatever prompt arrives and sends back a
// choice.
//
// The prompt is latched, so a browser opened after the prompt appeared still sees it. Halting
// publishes an empty prompt_id to withdraw it, so a cancelled Objective does not leave a dead
// dialog on screen waiting for an answer nobody is going to act on.

#include <moveit2_extended_behaviors/port_conversions.hpp>

#include <moveit2_extended_core/shared_resources_node.hpp>

#include <moveit2_extended_msgs/msg/user_prompt.hpp>
#include <moveit2_extended_msgs/msg/user_response.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <mutex>
#include <optional>

namespace moveit2_extended::behaviors
{

/** Common machinery: publish a prompt, wait for a matching response, time out.
 *
 *  Derived classes decide what the prompt says and what to do with the answer. */
class UserPromptBehaviorBase : public SharedResourcesNode<BT::StatefulActionNode>
{
public:
  UserPromptBehaviorBase(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);

  static BT::PortsList providedBasicPorts(BT::PortsList addition);
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

protected:
  /** What to ask. Called once, at onStart. */
  virtual BtExpected<moveit2_extended_msgs::msg::UserPrompt> createPrompt() = 0;
  /** What the answer means. The default succeeds when the choice is in `success_choices`. */
  virtual BtStatus processChoice(const std::string& choice);
  /** Called once at onStart, after the prompt is published: a hook for publishing whatever the
   *  operator needs to look at in order to answer. */
  virtual void publishContext()
  {
  }

private:
  BtStatus onStart() override;
  BtStatus onRunning() override;
  void onHalted() override;
  void withdrawPrompt();

  rclcpp::Publisher<moveit2_extended_msgs::msg::UserPrompt>::SharedPtr prompt_pub_;
  rclcpp::Subscription<moveit2_extended_msgs::msg::UserResponse>::SharedPtr response_sub_;
  BehaviorContext::CallbackIsland island_;

  std::mutex mutex_;
  std::optional<std::string> choice_;
  std::string prompt_id_;
  std::chrono::steady_clock::time_point started_at_;
  double timeout_s_ = 0.0;
};

/** Ask a yes/no (or n-way) question and branch on the answer. */
class GetTextFromUser : public UserPromptBehaviorBase
{
public:
  GetTextFromUser(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

protected:
  BtExpected<moveit2_extended_msgs::msg::UserPrompt> createPrompt() override;
  BtStatus processChoice(const std::string& choice) override;
};

/** Show a planned trajectory and wait for the operator to approve it.
 *
 *  Deliberately takes a moveit_msgs/RobotTrajectory rather than MoveIt Pro's MTC Solution: this
 *  stack plans through move_group, and requiring MTC here would drag in a dependency for no gain.
 *  The port name and role match Pro's, so an Objective reads the same. */
class WaitForUserTrajectoryApproval : public UserPromptBehaviorBase
{
public:
  WaitForUserTrajectoryApproval(const std::string& name, const NodeConfig& config,
                                BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

protected:
  BtExpected<moveit2_extended_msgs::msg::UserPrompt> createPrompt() override;
  void publishContext() override;

private:
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr preview_pub_;
};

/** Is anybody watching?
 *
 *  Lets an Objective take the automatic branch when it is running unattended and the interactive
 *  one when somebody is at the screen, instead of blocking for ever on a prompt nobody will see.
 *  Pro decides this by looking for its UI's node; we do the same with the Studio bridge. */
class IsUserAvailable : public ConditionBehaviorBase
{
public:
  IsUserAvailable(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  static BT::PortsList providedPorts();

private:
  BtStatus tick() override;
};

}  // namespace moveit2_extended::behaviors
