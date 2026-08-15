// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_core/shared_resources_node.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace moveit2_extended
{

/** A Behavior that sends one service request and waits for the response.
 *
 *  IMPORTANT, and worth knowing before you use this for anything with side effects: ROS 2 services
 *  cannot be cancelled. onHalted() abandons the future and removes the pending request, but the
 *  server still does the work. A Behavior whose service changes the world must therefore be paired
 *  with an idempotent undo Behavior and wrapped in a Fallback -- halting it is not the same as
 *  undoing it. */
template <typename ServiceT>
class ServiceClientBehaviorBase : public SharedResourcesNode<BT::StatefulActionNode>
{
public:
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;

  ServiceClientBehaviorBase(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources,
                            std::string default_service_name)
    : SharedResourcesNode<BT::StatefulActionNode>(name, config, std::move(shared_resources))
    , default_service_name_(std::move(default_service_name))
  {
    island_ = getSharedResources()->makeCallbackIsland();
  }

  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("service_name", "", "override the service name"),
      BT::InputPort<double>("service_timeout", 3.0, "seconds to wait for the service to appear"),
      BT::InputPort<double>("response_timeout", 5.0, "seconds to wait for the response; 0 = wait"),
    };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

protected:
  /** Build the request. Return an error to fail without calling the service. Tick thread. */
  virtual BtExpected<std::shared_ptr<Request>> createRequest() = 0;

  /** Map the response onto a status and write output ports. Tick thread. */
  virtual BtStatus processResponse(const std::shared_ptr<Response>& response) = 0;

  const std::string& serviceName() const
  {
    return service_name_;
  }

private:
  BtStatus onStart() override
  {
    service_name_ = getInputOr<std::string>("service_name", "");
    if (service_name_.empty())
    {
      service_name_ = params().template get<std::string>("service_name", default_service_name_);
    }

    client_ = getNode()->template create_client<ServiceT>(service_name_, rmw_qos_profile_services_default,
                                                         island_.group);

    const auto service_timeout = std::chrono::duration<double>(getInputOr<double>("service_timeout", 3.0));
    if (!client_->wait_for_service(std::chrono::duration_cast<std::chrono::nanoseconds>(service_timeout)))
    {
      RCLCPP_ERROR(getLogger(), "no service on '%s'", service_name_.c_str());
      return BtStatus::FAILURE;
    }

    auto request = createRequest();
    if (!request)
    {
      RCLCPP_ERROR(getLogger(), "cannot build the request: %s", request.error().c_str());
      return BtStatus::FAILURE;
    }

    future_ = client_->async_send_request(request.value()).future.share();
    started_at_ = std::chrono::steady_clock::now();
    return BtStatus::RUNNING;
  }

  BtStatus onRunning() override
  {
    island_.pump();

    if (!future_.valid())
    {
      RCLCPP_ERROR(getLogger(), "onRunning() with no request in flight");
      return BtStatus::FAILURE;
    }
    if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      auto response = future_.get();
      return processResponse(response);
    }

    const double response_timeout = getInputOr<double>("response_timeout", 5.0);
    if (response_timeout > 0.0)
    {
      const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at_).count();
      if (elapsed > response_timeout)
      {
        RCLCPP_ERROR(getLogger(), "no response from '%s' within %.1f s", service_name_.c_str(), response_timeout);
        abandonRequest();
        return BtStatus::FAILURE;
      }
    }
    return BtStatus::RUNNING;
  }

  void onHalted() override
  {
    abandonRequest();
  }

  void abandonRequest()
  {
    if (client_ && future_.valid())
    {
      // The server keeps going; this only stops us waiting. See the class comment.
      client_->remove_pending_request(future_);
      RCLCPP_WARN(getLogger(),
                  "abandoned the request to '%s'. ROS services cannot be cancelled, so the server "
                  "will still carry it out.",
                  service_name_.c_str());
    }
    future_ = {};
  }

  std::string default_service_name_;
  std::string service_name_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
  BehaviorContext::CallbackIsland island_;
  typename rclcpp::Client<ServiceT>::SharedFuture future_;
  std::chrono::steady_clock::time_point started_at_;
};

}  // namespace moveit2_extended
