// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_behaviors/motion_behaviors.hpp>
#include <moveit2_extended_behaviors/scene_behaviors.hpp>
#include <moveit2_extended_behaviors/ui_behaviors.hpp>
#include <moveit2_extended_behaviors/utility_behaviors.hpp>

#include <moveit2_extended_core/behavior_loader_base.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace moveit2_extended::behaviors
{

/** Registers the general-purpose Behaviors.
 *
 *  Names follow MoveIt Pro's where a Behavior does the same job, so an Objective written against
 *  Pro reads the same here. Where the two genuinely differ -- Pro's approval Behaviors take an MTC
 *  Solution, ours takes a RobotTrajectory -- the name is kept and the difference documented on the
 *  class, rather than inventing a new name for a familiar idea. */
class GeneralBehaviorsLoader : public BehaviorLoaderBase
{
public:
  void registerBehaviors(BtFactory& factory, const BehaviorContextPtr& shared_resources) override
  {
    // motion
    registerBehavior<MoveToJointState>(factory, "MoveToJointState", shared_resources);
    registerBehavior<MoveToPose>(factory, "MoveToPose", shared_resources);
    registerBehavior<PlanCartesianPath>(factory, "PlanCartesianPath", shared_resources);
    registerBehavior<RetimeTrajectory>(factory, "RetimeTrajectory", shared_resources);
    registerBehavior<ExecuteTrajectory>(factory, "ExecuteTrajectory", shared_resources);
    registerBehavior<ComputeInverseKinematics>(factory, "ComputeInverseKinematics", shared_resources);
    registerBehavior<StopMotion>(factory, "StopMotion", shared_resources);

    // planning scene
    registerBehavior<GetCurrentPlanningScene>(factory, "GetCurrentPlanningScene", shared_resources);
    registerBehavior<AddVirtualObjectToPlanningScene>(factory, "AddVirtualObjectToPlanningScene", shared_resources);
    registerBehavior<ModifyObjectInPlanningScene>(factory, "ModifyObjectInPlanningScene", shared_resources);
    registerBehavior<ClearSceneObjects>(factory, "ClearSceneObjects", shared_resources);

    // utility
    registerBehavior<CreatePoseStamped>(factory, "CreatePoseStamped", shared_resources);
    registerBehavior<TransformPose>(factory, "TransformPose", shared_resources);
    registerBehavior<GetLatestTransform>(factory, "GetLatestTransform", shared_resources);
    registerBehavior<GetJointState>(factory, "GetJointState", shared_resources);
    registerBehavior<LoadObjectiveParameters>(factory, "LoadObjectiveParameters", shared_resources);
    registerBehavior<GetElementOfVector>(factory, "GetElementOfVector", shared_resources);
    registerBehavior<IsPoseNearIdentity>(factory, "IsPoseNearIdentity", shared_resources);

    // operator interaction
    registerBehavior<GetTextFromUser>(factory, "GetTextFromUser", shared_resources);
    registerBehavior<WaitForUserTrajectoryApproval>(factory, "WaitForUserTrajectoryApproval", shared_resources);
    registerBehavior<IsUserAvailable>(factory, "IsUserAvailable", shared_resources);
  }

  std::unordered_map<std::string, std::string> behaviorDescriptions() const override
  {
    return {
      { "MoveToJointState", "Plan (and optionally execute) to a joint target or an SRDF named state." },
      { "MoveToPose", "Plan (and optionally execute) to a Cartesian pose, correcting target_link vs ik_link." },
      { "PlanCartesianPath", "Straight-line path through waypoints. Returns an UNTIMED trajectory." },
      { "RetimeTrajectory", "Time-parameterise a trajectory so a controller will accept it." },
      { "ExecuteTrajectory", "Run a planned trajectory. Refuses one that was never timed." },
      { "ComputeInverseKinematics", "Solve IK for a pose, with the target_link/ik_link correction applied." },
      { "StopMotion", "Ask move_group to stop executing. NOT an emergency stop." },
      { "GetCurrentPlanningScene", "Fetch the planning scene as it is now." },
      { "AddVirtualObjectToPlanningScene", "Add a primitive collision object, in the world or attached." },
      { "ModifyObjectInPlanningScene", "Move, remove, attach or detach an object already in the scene." },
      { "ClearSceneObjects", "Remove named collision objects." },
      { "CreatePoseStamped", "Build a PoseStamped from numbers." },
      { "TransformPose", "Offset a pose, in its own frame or its parent's." },
      { "GetLatestTransform", "Look a frame up in TF." },
      { "GetJointState", "The robot's current joint values." },
      { "LoadObjectiveParameters", "Read a YAML file onto the blackboard." },
      { "GetElementOfVector", "One element of a list." },
      { "IsPoseNearIdentity", "Succeed when two frames coincide within tolerance." },
      { "GetTextFromUser", "Ask the operator a question and branch on the answer." },
      { "WaitForUserTrajectoryApproval", "Show a trajectory and wait for approval before executing." },
      { "IsUserAvailable", "Succeed when a Studio UI is connected." },
    };
  }
};

}  // namespace moveit2_extended::behaviors

PLUGINLIB_EXPORT_CLASS(moveit2_extended::behaviors::GeneralBehaviorsLoader, moveit2_extended::BehaviorLoaderBase)
