// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_core/shared_resources_node.hpp>

#include <string>

namespace moveit2_extended
{

/** Publishes one message and returns immediately.
 *
 *  The only base class here that is a SyncActionNode: publishing does not block, so returning
 *  RUNNING would be a lie.
 *
 *  Unlike the subscriber case the publisher is created on first tick and then kept. Churning a
 *  publisher costs discovery traffic every time, and on a latched topic a fresh publisher drops
 *  the connection its previous message was holding open. */
template <typename MessageT>
class SendMessageToTopicBehaviorBase : public SharedResourcesNode<BT::SyncActionNode>
{
public:
  SendMessageToTopicBehaviorBase(const std::string& name, const NodeConfig& config,
                                 BehaviorContextPtr shared_resources, std::string default_topic)
    : SharedResourcesNode<BT::SyncActionNode>(name, config, std::move(shared_resources))
    , default_topic_(std::move(default_topic))
  {
  }

  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("topic_name", "", "override the topic"),
      BT::InputPort<std::string>("qos", "default", "default | sensor_data | latched"),
      BT::InputPort<MessageT>("message", "what to publish"),
    };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

protected:
  /** Build the message. The default reads the "message" input port. */
  virtual BtExpected<MessageT> createMessage()
  {
    const auto value = getInput<MessageT>("message");
    if (!value)
    {
      return nonstd::make_unexpected(value.error());
    }
    return value.value();
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

  BtStatus tick() override
  {
    if (!pub_)
    {
      topic_name_ = getInputOr<std::string>("topic_name", "");
      if (topic_name_.empty())
      {
        topic_name_ = params().template get<std::string>("topic_name", default_topic_);
      }
      pub_ = getNode()->template create_publisher<MessageT>(
          topic_name_, resolveQos(getInputOr<std::string>("qos", std::string("default"))));
    }

    auto message = createMessage();
    if (!message)
    {
      RCLCPP_ERROR(getLogger(), "cannot build the message: %s", message.error().c_str());
      return BtStatus::FAILURE;
    }
    pub_->publish(message.value());
    return BtStatus::SUCCESS;
  }

  std::string default_topic_;
  std::string topic_name_;
  typename rclcpp::Publisher<MessageT>::SharedPtr pub_;
};

}  // namespace moveit2_extended
