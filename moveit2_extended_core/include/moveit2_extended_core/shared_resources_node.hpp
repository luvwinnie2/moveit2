// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_core/behavior_context.hpp>
#include <moveit2_extended_core/bt_compat.hpp>

#include <optional>
#include <string>
#include <type_traits>
#include <utility>

namespace moveit2_extended
{

/** Mixin that hands the BehaviorContext and this Behavior's configuration parameters to any
 *  BehaviorTree.CPP node base. Every Behavior derives from SharedResourcesNode<Base>, never from
 *  Base directly.
 *
 *  This exists because BehaviorTree.CPP v3.8's NodeBuilder signature is fixed at
 *  (const std::string&, const NodeConfiguration&). A third constructor argument is reachable only
 *  through a capturing lambda registered with registerBuilder<T>() -- which is exactly what
 *  registerBehavior<T>() in behavior_loader_base.hpp does. That, and not any deficiency of
 *  pluginlib, is why this architecture needs a loader object per package at all. MoveIt Pro
 *  arrives at the same design and its own documentation points at BehaviorTree.CPP tutorial 8,
 *  "Method 1: register a custom builder". */
template <typename BaseT>
class SharedResourcesNode : public BaseT
{
  static_assert(std::is_base_of_v<BT::TreeNode, BaseT>,
                "SharedResourcesNode wraps a BehaviorTree.CPP node base class");

public:
  SharedResourcesNode(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources)
    : BaseT(name, config), shared_resources_(std::move(shared_resources))
  {
    if (!shared_resources_)
    {
      throw BT::RuntimeError("Behavior '" + name + "' was constructed with a null BehaviorContext");
    }
  }

protected:
  const BehaviorContextPtr& getSharedResources() const
  {
    return shared_resources_;
  }
  const rclcpp::Node::SharedPtr& getNode() const
  {
    return shared_resources_->node();
  }
  rclcpp::Logger getLogger() const
  {
    return shared_resources_->logger().get_child(this->name());
  }

  /** Configuration parameters for this Behavior *type*.
   *
   *  Resolved lazily rather than in the constructor because registrationName() is not populated
   *  until after the base class has been constructed. */
  const BehaviorParameters& params() const
  {
    if (!params_cache_)
    {
      params_cache_ = shared_resources_->parametersFor(this->registrationName());
    }
    return *params_cache_;
  }

  /** getInput that fails loudly.
   *
   *  Substituting a default for a missing target pose is worse than a red tree: the robot moves
   *  somewhere nobody asked for. Anything without a defensible default goes through this. */
  template <typename T>
  T getRequiredInput(const std::string& port) const
  {
    const auto value = this->template getInput<T>(port);
    if (!value)
    {
      throw BT::RuntimeError("[" + this->name() + "] required input port '" + port + "': " + value.error());
    }
    return value.value();
  }

  template <typename T>
  T getInputOr(const std::string& port, const T& fallback) const
  {
    const auto value = this->template getInput<T>(port);
    return value ? value.value() : fallback;
  }

private:
  BehaviorContextPtr shared_resources_;
  mutable std::optional<BehaviorParameters> params_cache_;
};

/** A condition is deliberately given no callback island and no asynchronous machinery: it must be
 *  cheap and synchronous, because control nodes tick conditions far more often than actions. */
using ConditionBehaviorBase = SharedResourcesNode<BT::ConditionNode>;
using SyncBehaviorBase = SharedResourcesNode<BT::SyncActionNode>;
using DecoratorBehaviorBase = SharedResourcesNode<BT::DecoratorNode>;
using ControlBehaviorBase = SharedResourcesNode<BT::ControlNode>;

}  // namespace moveit2_extended
