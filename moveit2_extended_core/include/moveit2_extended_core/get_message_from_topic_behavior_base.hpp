// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_core/shared_resources_node.hpp>

#include <chrono>
#include <mutex>
#include <optional>
#include <string>

namespace moveit2_extended
{

/** Waits for one message on a topic and puts it on an output port.
 *
 *  The subscription is created per activation and destroyed as soon as the message arrives. That
 *  is deliberate: a tree that runs this inside a Repeat must not accumulate subscriptions, and a
 *  latched message from a previous activation must not satisfy a later one -- "wait for the next
 *  reading" is what the caller asked for. */
template <typename MessageT>
class GetMessageFromTopicBehaviorBase : public SharedResourcesNode<BT::StatefulActionNode>
{
public:
  GetMessageFromTopicBehaviorBase(const std::string& name, const NodeConfig& config,
                                  BehaviorContextPtr shared_resources, std::string default_topic)
    : SharedResourcesNode<BT::StatefulActionNode>(name, config, std::move(shared_resources))
    , default_topic_(std::move(default_topic))
  {
    island_ = getSharedResources()->makeCallbackIsland();
  }

  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("topic_name", "", "override the topic"),
      BT::InputPort<double>("timeout", 5.0, "seconds to wait; 0 = wait for ever"),
      BT::InputPort<std::string>("qos", "default", "default | sensor_data | latched"),
      BT::OutputPort<MessageT>("message", "the message that was received"),
    };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

protected:
  /** Reject a message and keep waiting -- e.g. wrong frame_id, or a stale stamp. */
  virtual bool accept(const MessageT& /*message*/)
  {
    return true;
  }

  /** Write output ports. Tick thread. The default puts the message on "message". */
  virtual BtStatus onMessage(const MessageT& message)
  {
    setOutput("message", message);
    return BtStatus::SUCCESS;
  }

  const std::string& topicName() const
  {
    return topic_name_;
  }

private:
  static rclcpp::QoS resolveQos(const std::string& name)
  {
    if (name == "sensor_data")
    {
      return rclcpp::SensorDataQoS();
    }
    if (name == "latched" || name == "transient_local")
    {
      return rclcpp::QoS(1).transient_local().reliable();
    }
    return rclcpp::QoS(10);
  }

  BtStatus onStart() override
  {
    topic_name_ = getInputOr<std::string>("topic_name", "");
    if (topic_name_.empty())
    {
      topic_name_ = params().template get<std::string>("topic_name", default_topic_);
    }
    timeout_s_ = getInputOr<double>("timeout", 5.0);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      message_.reset();
    }

    rclcpp::SubscriptionOptions options;
    options.callback_group = island_.group;
    sub_ = getNode()->template create_subscription<MessageT>(
        topic_name_, resolveQos(getInputOr<std::string>("qos", std::string("default"))),
        [this](const std::shared_ptr<MessageT> msg) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            if (message_)
            {
              return;  // already have one; keep the first
            }
            message_ = *msg;
          }
          emitStateChanged();
        },
        options);

    started_at_ = std::chrono::steady_clock::now();
    return BtStatus::RUNNING;
  }

  BtStatus onRunning() override
  {
    island_.pump();

    std::optional<MessageT> message;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      message = message_;
    }

    if (message)
    {
      if (!accept(*message))
      {
        std::lock_guard<std::mutex> lock(mutex_);
        message_.reset();  // not the one we want; keep listening
      }
      else
      {
        sub_.reset();
        return onMessage(*message);
      }
    }

    if (timeout_s_ > 0.0)
    {
      const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at_).count();
      if (elapsed > timeout_s_)
      {
        RCLCPP_ERROR(getLogger(), "no acceptable message on '%s' within %.1f s", topic_name_.c_str(), timeout_s_);
        sub_.reset();
        return BtStatus::FAILURE;
      }
    }
    return BtStatus::RUNNING;
  }

  void onHalted() override
  {
    sub_.reset();
  }

  std::string default_topic_;
  std::string topic_name_;
  typename rclcpp::Subscription<MessageT>::SharedPtr sub_;
  BehaviorContext::CallbackIsland island_;

  std::mutex mutex_;
  std::optional<MessageT> message_;
  std::chrono::steady_clock::time_point started_at_;
  double timeout_s_ = 5.0;
};

}  // namespace moveit2_extended
