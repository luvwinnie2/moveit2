// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_core/behavior_loader_base.hpp>
#include <moveit2_extended_core/behaviors/check_blackboard_value.hpp>
#include <moveit2_extended_core/behaviors/log_message.hpp>
#include <moveit2_extended_core/behaviors/wait_for_duration.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace moveit2_extended::behaviors
{

/** The Behaviors that come with the core, so a server with no other package installed can still
 *  load, build and run a tree. That is what makes the smoke test able to prove the pluginlib +
 *  registerBuilder seam works without dragging in MoveIt. */
class CoreBehaviorsLoader : public BehaviorLoaderBase
{
public:
  void registerBehaviors(BtFactory& factory, const BehaviorContextPtr& shared_resources) override
  {
    registerBehavior<LogMessage>(factory, "LogMessage", shared_resources);
    registerBehavior<WaitForDuration>(factory, "WaitForDuration", shared_resources);
    registerBehavior<CheckBlackboardValue>(factory, "CheckBlackboardValue", shared_resources);
  }

  std::unordered_map<std::string, std::string> behaviorDescriptions() const override
  {
    return {
      { "LogMessage", "Write a line to the ROS log. Always succeeds." },
      { "WaitForDuration", "Return RUNNING for a fixed number of seconds, then SUCCESS." },
      { "CheckBlackboardValue", "Compare a blackboard entry against a literal, as strings." },
    };
  }
};

}  // namespace moveit2_extended::behaviors

PLUGINLIB_EXPORT_CLASS(moveit2_extended::behaviors::CoreBehaviorsLoader, moveit2_extended::BehaviorLoaderBase)
