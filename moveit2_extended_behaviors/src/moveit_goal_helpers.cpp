// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_behaviors/moveit_goal_helpers.hpp>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace moveit2_extended::behaviors
{
namespace
{
/** True when `descendant` can be reached from `ancestor` by following parent links through FIXED
 *  joints only. */
bool isFixedChainFrom(const moveit::core::RobotModelConstPtr& model, const std::string& ancestor,
                      const std::string& descendant)
{
  const moveit::core::LinkModel* link = model->getLinkModel(descendant);
  while (link)
  {
    if (link->getName() == ancestor)
    {
      return true;
    }
    const moveit::core::JointModel* joint = link->getParentJointModel();
    if (!joint || joint->getType() != moveit::core::JointModel::FIXED)
    {
      return false;
    }
    link = link->getParentLinkModel();
  }
  return false;
}

/** Rigid in either direction: the tool may hang off the flange, or (less usually) the other way. */
bool isRigidlyConnected(const moveit::core::RobotModelConstPtr& model, const std::string& a, const std::string& b)
{
  return isFixedChainFrom(model, a, b) || isFixedChainFrom(model, b, a);
}
}  // namespace

BtExpected<geometry_msgs::msg::PoseStamped> retargetPose(const moveit::core::RobotModelConstPtr& model,
                                                         const geometry_msgs::msg::PoseStamped& target_pose,
                                                         const std::string& target_link,
                                                         const std::string& ik_link)
{
  if (!model)
  {
    return nonstd::make_unexpected("no robot model available");
  }
  if (target_link.empty() || ik_link.empty())
  {
    return nonstd::make_unexpected("both target_link and ik_link must be named");
  }
  if (target_link == ik_link)
  {
    return target_pose;  // nothing to do
  }
  if (!model->hasLinkModel(target_link))
  {
    return nonstd::make_unexpected("target_link '" + target_link + "' is not in the robot model");
  }
  if (!model->hasLinkModel(ik_link))
  {
    return nonstd::make_unexpected("ik_link '" + ik_link + "' is not in the robot model");
  }

  // Rigidity is checked against the model structure rather than by perturbing joints: walk the
  // parent chain and require every joint between the two links to be FIXED. If a movable joint
  // sits between them, a constant offset is meaningless -- and silently returning a
  // plausible-looking wrong pose is exactly the failure this function exists to prevent.
  if (!isRigidlyConnected(model, ik_link, target_link))
  {
    return nonstd::make_unexpected("'" + ik_link + "' and '" + target_link +
                                   "' are not rigidly connected, so a fixed offset cannot be used to "
                                   "retarget between them");
  }

  // With rigidity established, any state gives the same relative transform, so the default one
  // will do.
  moveit::core::RobotState probe(model);
  probe.setToDefaultValues();
  probe.update();

  const Eigen::Isometry3d ik_to_target =
      probe.getGlobalLinkTransform(ik_link).inverse() * probe.getGlobalLinkTransform(target_link);

  Eigen::Isometry3d target;
  tf2::fromMsg(target_pose.pose, target);

  geometry_msgs::msg::PoseStamped out;
  out.header = target_pose.header;
  out.pose = tf2::toMsg(target * ik_to_target.inverse());
  return out;
}

BtExpected<moveit::core::RobotState> resolveTargetState(const moveit::core::RobotModelConstPtr& model,
                                                        const moveit::core::RobotState& reference,
                                                        const std::string& group, const std::string& named_state,
                                                        const sensor_msgs::msg::JointState& joint_state)
{
  if (!model)
  {
    return nonstd::make_unexpected("no robot model available");
  }
  const moveit::core::JointModelGroup* jmg = model->getJointModelGroup(group);
  if (!jmg)
  {
    return nonstd::make_unexpected("no planning group named '" + group + "'");
  }

  // Seeded from the reference so joints the caller did not mention keep their current values.
  moveit::core::RobotState state(reference);

  if (!named_state.empty())
  {
    if (!state.setToDefaultValues(jmg, named_state))
    {
      return nonstd::make_unexpected("the SRDF has no group_state '" + named_state + "' for group '" + group + "'");
    }
    state.update();
    return state;
  }

  if (joint_state.position.empty())
  {
    return nonstd::make_unexpected("neither named_state nor joint_state was given");
  }

  if (!joint_state.name.empty())
  {
    if (joint_state.name.size() != joint_state.position.size())
    {
      return nonstd::make_unexpected("joint_state has " + std::to_string(joint_state.name.size()) + " name(s) but " +
                                     std::to_string(joint_state.position.size()) + " position(s)");
    }
    for (size_t i = 0; i < joint_state.name.size(); ++i)
    {
      if (!model->hasJointModel(joint_state.name[i]))
      {
        return nonstd::make_unexpected("no joint named '" + joint_state.name[i] + "'");
      }
      state.setJointPositions(joint_state.name[i], &joint_state.position[i]);
    }
  }
  else
  {
    // Unnamed: positional against the group's active joints. Accepted because it is convenient,
    // but it silently means something else the day the group is reordered -- so it is checked
    // strictly and the named form is what the documentation recommends.
    const auto& names = jmg->getActiveJointModelNames();
    if (joint_state.position.size() != names.size())
    {
      return nonstd::make_unexpected("an unnamed joint_state must have exactly one value per active joint of '" +
                                     group + "' (" + std::to_string(names.size()) + "), got " +
                                     std::to_string(joint_state.position.size()));
    }
    for (size_t i = 0; i < names.size(); ++i)
    {
      state.setJointPositions(names[i], &joint_state.position[i]);
    }
  }

  state.update();
  return state;
}

BtExpected<moveit_msgs::msg::Constraints> makeJointGoal(const moveit::core::RobotState& goal_state,
                                                        const std::string& group, double tolerance)
{
  const moveit::core::JointModelGroup* jmg = goal_state.getJointModelGroup(group);
  if (!jmg)
  {
    return nonstd::make_unexpected("no planning group named '" + group + "'");
  }
  return kinematic_constraints::constructGoalConstraints(goal_state, jmg, tolerance);
}

moveit_msgs::msg::Constraints makePoseGoal(const geometry_msgs::msg::PoseStamped& pose, const std::string& ik_link,
                                           double position_tolerance, double orientation_tolerance)
{
  return kinematic_constraints::constructGoalConstraints(ik_link, pose, position_tolerance, orientation_tolerance);
}

BtExpected<moveit_msgs::msg::RobotState> finalStateOf(const moveit::core::RobotModelConstPtr& model,
                                                      const moveit::core::RobotState& reference,
                                                      const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  if (!model)
  {
    return nonstd::make_unexpected("no robot model available");
  }
  if (trajectory.joint_trajectory.points.empty() && trajectory.multi_dof_joint_trajectory.points.empty())
  {
    return nonstd::make_unexpected("the trajectory has no waypoints");
  }

  robot_trajectory::RobotTrajectory parsed(model, "");
  try
  {
    moveit_msgs::msg::RobotState reference_msg;
    moveit::core::robotStateToRobotStateMsg(reference, reference_msg);
    parsed.setRobotTrajectoryMsg(reference, reference_msg, trajectory);
  }
  catch (const std::exception& exc)
  {
    return nonstd::make_unexpected(std::string("cannot read the trajectory: ") + exc.what());
  }
  if (parsed.empty())
  {
    return nonstd::make_unexpected("the trajectory has no waypoints");
  }

  moveit_msgs::msg::RobotState out;
  moveit::core::robotStateToRobotStateMsg(parsed.getLastWayPoint(), out);
  return out;
}

}  // namespace moveit2_extended::behaviors
