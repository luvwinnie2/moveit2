// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_core/shared_resources_node.hpp>

#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace moveit2_extended
{

/** A Behavior that sends one goal to a ROS 2 action server and waits for the result.
 *
 *  The goal is sent asynchronously and polled from onRunning(), so the tick never blocks and
 *  onHalted() can genuinely cancel. All three rclcpp_action callbacks are delivered on this
 *  Behavior's own callback island, which is pumped only from the tick thread -- so in practice
 *  they run on the tick thread, and the mutex below is belt and braces.
 *
 *  Derived classes implement createGoal() and processResult(); everything else is handled here. */
template <typename ActionT>
class ActionClientBehaviorBase : public SharedResourcesNode<BT::StatefulActionNode>
{
public:
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;

  ActionClientBehaviorBase(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources,
                           std::string default_action_name)
    : SharedResourcesNode<BT::StatefulActionNode>(name, config, std::move(shared_resources))
    , default_action_name_(std::move(default_action_name))
  {
    island_ = getSharedResources()->makeCallbackIsland();
  }

  /** Every derived providedPorts() must funnel through this so the common ports exist everywhere. */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("action_name", "", "override the action server name"),
      BT::InputPort<double>("server_timeout", 3.0, "seconds to wait for the action server"),
      BT::InputPort<double>("result_timeout", 0.0, "seconds before giving up on the result; 0 = wait"),
    };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

protected:
  /** Build the goal from ports and configuration. Return an error to fail without contacting the
   *  server at all. Runs on the tick thread. */
  virtual BtExpected<Goal> createGoal() = 0;

  /** Map the server's result onto a status and write output ports. Runs on the tick thread. */
  virtual BtStatus processResult(const WrappedResult& result) = 0;

  /** Optional. Runs on the tick thread; only the newest feedback is kept. */
  virtual void processFeedback(const std::shared_ptr<const Feedback>& /*feedback*/)
  {
  }

  const std::string& actionName() const
  {
    return action_name_;
  }

private:
  BtStatus onStart() override
  {
    action_name_ = getInputOr<std::string>("action_name", "");
    if (action_name_.empty())
    {
      action_name_ = params().template get<std::string>("action_name", default_action_name_);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_handle_.reset();
      result_.reset();
      latest_feedback_.reset();
      goal_rejected_ = false;
    }

    client_ = rclcpp_action::create_client<ActionT>(getNode(), action_name_, island_.group);

    const auto server_timeout = std::chrono::duration<double>(getInputOr<double>("server_timeout", 3.0));
    if (!client_->wait_for_action_server(std::chrono::duration_cast<std::chrono::nanoseconds>(server_timeout)))
    {
      RCLCPP_ERROR(getLogger(), "no action server on '%s'", action_name_.c_str());
      return BtStatus::FAILURE;
    }

    auto goal = createGoal();
    if (!goal)
    {
      RCLCPP_ERROR(getLogger(), "cannot build the goal: %s", goal.error().c_str());
      return BtStatus::FAILURE;
    }

    typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
    options.goal_response_callback = [this](typename GoalHandle::SharedPtr handle) {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_handle_ = handle;
      goal_rejected_ = !handle;
    };
    options.feedback_callback = [this](typename GoalHandle::SharedPtr,
                                       const std::shared_ptr<const Feedback> feedback) {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_feedback_ = feedback;
    };
    options.result_callback = [this](const WrappedResult& result) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        result_ = result;
      }
      // Cut the tree's sleep short so the result is acted on immediately rather than at the end of
      // the tick period.
      emitStateChanged();
    };

    client_->async_send_goal(goal.value(), options);
    started_at_ = std::chrono::steady_clock::now();
    return BtStatus::RUNNING;
  }

  BtStatus onRunning() override
  {
    island_.pump();

    std::shared_ptr<const Feedback> feedback;
    std::optional<WrappedResult> result;
    bool rejected = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      feedback = latest_feedback_;
      latest_feedback_.reset();
      result = result_;
      rejected = goal_rejected_;
    }

    if (rejected)
    {
      RCLCPP_ERROR(getLogger(), "goal rejected by '%s'", action_name_.c_str());
      return BtStatus::FAILURE;
    }
    if (feedback)
    {
      processFeedback(feedback);
    }
    if (result)
    {
      return processResult(*result);
    }

    const double result_timeout = getInputOr<double>("result_timeout", 0.0);
    if (result_timeout > 0.0)
    {
      const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at_).count();
      if (elapsed > result_timeout)
      {
        RCLCPP_ERROR(getLogger(), "no result from '%s' within %.1f s", action_name_.c_str(), result_timeout);
        cancelGoal();
        return BtStatus::FAILURE;
      }
    }
    return BtStatus::RUNNING;
  }

  void onHalted() override
  {
    cancelGoal();
  }

  void cancelGoal()
  {
    typename GoalHandle::SharedPtr handle;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      handle = goal_handle_;
      goal_handle_.reset();
    }
    if (!handle || !client_)
    {
      return;
    }
    client_->async_cancel_goal(handle);
    // Give the cancel a moment to leave the process. Bounded, because a halt must not turn into a
    // blocking wait on a server that may itself be wedged.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    while (std::chrono::steady_clock::now() < deadline)
    {
      island_.pump();
      std::lock_guard<std::mutex> lock(mutex_);
      if (result_)
      {
        break;
      }
    }
  }

  std::string default_action_name_;
  std::string action_name_;
  typename rclcpp_action::Client<ActionT>::SharedPtr client_;
  BehaviorContext::CallbackIsland island_;

  std::mutex mutex_;
  typename GoalHandle::SharedPtr goal_handle_;
  std::optional<WrappedResult> result_;
  std::shared_ptr<const Feedback> latest_feedback_;
  bool goal_rejected_ = false;

  std::chrono::steady_clock::time_point started_at_;
};

}  // namespace moveit2_extended
