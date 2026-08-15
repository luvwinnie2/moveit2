// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// The boundary between "there is nothing to run" and "nothing was planned".
//
// Found by running the same Objective twice: the second time the arm was already standing on the
// taught waypoint, the planner returned a one-point trajectory, and ExecuteTrajectory refused it as
// "not time-parameterised". An Objective that fails only on its second run, with a message about
// timing, is about as misleading as a failure gets.
//
// The fix must NOT swallow the other case. An empty trajectory means the planner produced nothing,
// and quietly succeeding on that would let an Objective report success having never moved.
//
// No move_group and no action server are involved: a one-point trajectory is answered before
// ExecuteTrajectory looks for a server, which is exactly what makes it testable here.

#include <moveit2_extended_behaviors/motion_behaviors.hpp>

#include <moveit2_extended_core/behavior_loader_base.hpp>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

using moveit2_extended::BehaviorContext;
using moveit2_extended::BehaviorContextConfig;
using moveit2_extended::BehaviorContextPtr;
using moveit2_extended::BehaviorParameterMap;
using moveit2_extended::BtFactory;
using moveit2_extended::BtStatus;
using moveit2_extended::registerBehavior;
using moveit2_extended::tickOnce;

namespace
{
BehaviorContextPtr makeContext()
{
  auto node = std::make_shared<rclcpp::Node>("test_execute_trajectory_noop");
  BehaviorContextConfig config;
  config.start_planning_scene_monitor = false;
  return std::make_shared<BehaviorContext>(node, config, BehaviorParameterMap::fromText(""));
}

moveit_msgs::msg::RobotTrajectory trajectoryWithPoints(size_t count)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = { "J1", "J2", "J3", "J4", "J5", "J6" };
  for (size_t i = 0; i < count; ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(6, 0.0);
    // Left at t=0 deliberately: that is what the "not time-parameterised" check keys on, so a
    // one-point trajectory has to be answered before that check to prove anything.
    trajectory.joint_trajectory.points.push_back(point);
  }
  return trajectory;
}

/** Build a tree with one ExecuteTrajectory and the given trajectory on its port. */
BtStatus tickWith(const BehaviorContextPtr& context, const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  BtFactory factory;
  registerBehavior<moveit2_extended::behaviors::ExecuteTrajectory>(factory, "ExecuteTrajectory", context);

  auto blackboard = BT::Blackboard::create();
  blackboard->set("traj", trajectory);
  // server_timeout is pushed to nearly nothing so the empty case fails fast instead of spending
  // the default three seconds looking for a move_group that is not running.
  auto tree = factory.createTreeFromText(
      R"(<root main_tree_to_execute="T"><BehaviorTree ID="T">
           <ExecuteTrajectory trajectory="{traj}" server_timeout="0.2"/>
         </BehaviorTree></root>)",
      blackboard);
  return tickOnce(tree);
}
}  // namespace

TEST(ExecuteTrajectoryNoOp, AOnePointTrajectoryIsNothingToDoAndSucceeds)
{
  auto context = makeContext();
  EXPECT_EQ(tickWith(context, trajectoryWithPoints(1)), BtStatus::SUCCESS);
}

TEST(ExecuteTrajectoryNoOp, AnEmptyTrajectoryStillFails)
{
  // The regression guard for the fix itself: `<= 1` would have made this succeed.
  auto context = makeContext();
  EXPECT_EQ(tickWith(context, trajectoryWithPoints(0)), BtStatus::FAILURE);
}

TEST(ExecuteTrajectoryNoOp, ATwoPointUntimedTrajectoryStillFails)
{
  // Two points both at t=0 is a genuinely untimed trajectory and must keep being refused --
  // running it would go as fast as the hardware allows.
  auto context = makeContext();
  EXPECT_EQ(tickWith(context, trajectoryWithPoints(2)), BtStatus::FAILURE);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
