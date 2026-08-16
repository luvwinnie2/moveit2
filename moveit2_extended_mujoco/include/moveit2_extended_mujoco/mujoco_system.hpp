// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <moveit2_extended_msgs/srv/set_simulation_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// MuJoCo's own headers. Only the C API is used, so the Python wheel's bundled library is enough.
#include <mujoco/mujoco.h>

namespace moveit2_extended::mujoco_sim
{

/** MuJoCo behind ros2_control, so the simulator is a robot rather than a special case.
 *
 *  This is the shape MoveIt Pro uses -- its service lives at `/mujoco_system/set_joint_states`,
 *  which is ros2_control naming, and its documentation says the state-setting Behavior explicitly
 *  does NOT manage controllers. Copying that shape buys the thing that matters: joint_trajectory
 *  _controller drives the simulated arm, MoveIt executes FollowJointTrajectory into it with no
 *  simulator-specific code, and the same Objective XML runs here and on hardware. The alternative
 *  -- a bespoke node that replays trajectories -- is what this replaces, and it could never test
 *  the controller layer because it *was* the controller layer.
 *
 *  Interfaces: position and velocity out, position in. The MJCF carries one position actuator per
 *  revolute joint (urdf_to_mujoco.py adds them; a URDF-derived model has none, and without them
 *  the arm is a ragdoll).
 *
 *  THREADING. mjData is not thread safe and two things touch it: the controller manager's
 *  read/write, and the service callbacks that teleport the simulation. One mutex covers both. The
 *  services run on their own node and executor thread so a slow service call cannot stall the
 *  control loop.
 */
class MujocoSystem : public hardware_interface::SystemInterface
{
public:
  MujocoSystem() = default;
  ~MujocoSystem() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  /** Apply a named keyframe from the model. Caller holds the mutex. */
  bool applyKeyframe(const std::string& name, std::string& message);

  void onSetState(const std::shared_ptr<moveit2_extended_msgs::srv::SetSimulationState::Request> request,
                  std::shared_ptr<moveit2_extended_msgs::srv::SetSimulationState::Response> response);
  void onReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  std::mutex mutex_;

  /** Per exported joint, the index into MuJoCo's qpos/qvel and its actuator, or -1. */
  struct JointMap
  {
    std::string name;
    int qpos_index = -1;
    int qvel_index = -1;
    int actuator = -1;
  };
  std::vector<JointMap> joints_;

  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> command_;

  std::string mjcf_path_;
  std::string initial_keyframe_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<moveit2_extended_msgs::srv::SetSimulationState>::SharedPtr set_state_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
};

}  // namespace moveit2_extended::mujoco_sim
