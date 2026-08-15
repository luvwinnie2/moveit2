// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_core/behavior_context.hpp>
#include <moveit2_extended_core/bt_compat.hpp>
#include <moveit2_extended_core/shared_resources_node.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace moveit2_extended
{

/** pluginlib base class. One loader per package that provides Behaviors.
 *
 *  WHY THIS EXISTS, rather than BehaviorTree.CPP's own registerFromPlugin()/BT_REGISTER_NODES:
 *  that route hands a plugin nothing but a BehaviorTreeFactory, and BT::NodeBuilder's signature is
 *  fixed at (name, NodeConfiguration). There is no seam through which a BehaviorContext could
 *  reach a node's constructor. A loader object receives the context as an argument and closes over
 *  it in each builder lambda.
 *
 *  The Objective server constructs one instance of every configured loader and calls
 *  registerBehaviors() on the single factory it owns. */
class BehaviorLoaderBase
{
public:
  virtual ~BehaviorLoaderBase() = default;

  /** Register every BT node type this package provides, via registerBehavior<T>() below.
   *
   *  Registering an ID that another loader already took throws. The server catches that per
   *  loader, names both packages, and carries on with the rest -- one package's collision must not
   *  cost you every other package's Behaviors. */
  virtual void registerBehaviors(BtFactory& factory, const BehaviorContextPtr& shared_resources) = 0;

  /** Optional documentation, merged into the factory manifests so ListBehaviors and the editor's
   *  node palette can show it. */
  virtual std::unordered_map<std::string, std::string> behaviorDescriptions() const
  {
    return {};
  }

  /** Set by the server after construction. Diagnostics only -- it is what lets an ID collision be
   *  reported as "package A vs package B" instead of a bare duplicate-name error. */
  void setSource(std::string package, std::string plugin_class)
  {
    source_package_ = std::move(package);
    source_class_ = std::move(plugin_class);
  }
  const std::string& sourcePackage() const
  {
    return source_package_;
  }
  const std::string& sourceClass() const
  {
    return source_class_;
  }

protected:
  BehaviorLoaderBase() = default;

private:
  std::string source_package_;
  std::string source_class_;
};

/** The only sanctioned way to register a Behavior.
 *
 *  BehaviorTree.CPP v3.8's registerNodeType<T>() static_asserts that T is constructible from
 *  (name) or (name, NodeConfiguration) -- so a Behavior that takes a BehaviorContext cannot use
 *  it, and must go through registerBuilder<T>(ID, builder) instead.
 *
 *  registerBuilder has no such assertions, and that is the trap this wrapper closes: it still
 *  builds the manifest with CreateManifest<T>(ID), which reads T::providedPorts() through SFINAE
 *  and silently yields an EMPTY port list if the method is missing. The Behavior then registers
 *  fine and every getInput() fails at runtime with "port not provided". The second static_assert
 *  turns that into a compile error. */
template <typename BehaviorT>
void registerBehavior(BtFactory& factory, const std::string& registration_name,
                      const BehaviorContextPtr& shared_resources)
{
  static_assert(std::is_constructible_v<BehaviorT, const std::string&, const NodeConfig&, BehaviorContextPtr>,
                "A Behavior must be constructible from (name, NodeConfig, BehaviorContextPtr). "
                "Derive from SharedResourcesNode<Base> and forward all three arguments.");
  static_assert(BT::has_static_method_providedPorts<BehaviorT>::value,
                "A Behavior must declare 'static BT::PortsList providedPorts()'. Without it, "
                "registerBuilder silently registers an empty port list and every getInput() fails "
                "at runtime.");

  BT::NodeBuilder builder = [shared_resources](const std::string& name, const NodeConfig& config) {
    return std::make_unique<BehaviorT>(name, config, shared_resources);
  };
  factory.registerBuilder<BehaviorT>(registration_name, builder);
}

}  // namespace moveit2_extended
