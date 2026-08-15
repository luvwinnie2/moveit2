// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_carrier/carrier_behaviors.hpp>

#include <moveit2_extended_core/behavior_loader_base.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace moveit2_extended::carrier
{

/** The cable-carrier Behaviors.
 *
 *  A separate package from the general Behaviors on purpose: this one links moveit_cable_carrier,
 *  Eigen and yaml-cpp, and a deployment without a dresspack should not pay for any of that. Not
 *  installing this package simply means these Behaviors do not appear -- no configuration change
 *  anywhere else. */
class CarrierBehaviorsLoader : public BehaviorLoaderBase
{
public:
  void registerBehaviors(BtFactory& factory, const BehaviorContextPtr& shared_resources) override
  {
    registerBehavior<ValidateCarrierAlongTrajectory>(factory, "ValidateCarrierAlongTrajectory", shared_resources);
    registerBehavior<IsCarrierFeasible>(factory, "IsCarrierFeasible", shared_resources);
    registerBehavior<GetCarrierDiagnostics>(factory, "GetCarrierDiagnostics", shared_resources);
    registerBehavior<SwitchCarrierType>(factory, "SwitchCarrierType", shared_resources);
  }

  std::unordered_map<std::string, std::string> behaviorDescriptions() const override
  {
    return {
      { "ValidateCarrierAlongTrajectory",
        "Judge a planned trajectory against the cable carrier: bend, twist, cable radius, tension "
        "and per-segment fatigue. Fails the tree when the carrier could not survive it." },
      { "IsCarrierFeasible", "Cheap condition: can the carrier take its shape at the current pose?" },
      { "GetCarrierDiagnostics", "The carrier's numbers at the current pose, onto the blackboard." },
      { "SwitchCarrierType", "Load a different carrier configuration at run time (this process only)." },
    };
  }
};

}  // namespace moveit2_extended::carrier

PLUGINLIB_EXPORT_CLASS(moveit2_extended::carrier::CarrierBehaviorsLoader, moveit2_extended::BehaviorLoaderBase)
