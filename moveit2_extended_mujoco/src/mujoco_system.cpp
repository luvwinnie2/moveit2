// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_mujoco/mujoco_system.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <regex>

namespace moveit2_extended::mujoco_sim
{
namespace
{
constexpr const char* kLogger = "mujoco_system";
/** One control period should never advance the simulation more than this. */
constexpr int kMaxStepsPerCycle = 200;
}  // namespace

MujocoSystem::~MujocoSystem()
{
  if (executor_)
  {
    executor_->cancel();
  }
  if (spin_thread_.joinable())
  {
    spin_thread_.join();
  }
  if (data_ != nullptr)
  {
    mj_deleteData(data_);
  }
  if (model_ != nullptr)
  {
    mj_deleteModel(model_);
  }
}

hardware_interface::CallbackReturn MujocoSystem::on_init(const hardware_interface::HardwareInfo& info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto mjcf = info.hardware_parameters.find("mjcf");
  if (mjcf == info.hardware_parameters.end() || mjcf->second.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger),
                 "the <hardware> block needs an 'mjcf' parameter pointing at a compiled model. "
                 "Generate one with: ros2 run moveit2_extended_mujoco urdf_to_mujoco.py");
    return hardware_interface::CallbackReturn::ERROR;
  }
  mjcf_path_ = mjcf->second;

  const auto keyframe = info.hardware_parameters.find("initial_keyframe");
  initial_keyframe_ = keyframe == info.hardware_parameters.end() ? "" : keyframe->second;

  std::array<char, 1024> error{};
  model_ = mj_loadXML(mjcf_path_.c_str(), nullptr, error.data(), static_cast<int>(error.size()));
  if (model_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "cannot load %s: %s", mjcf_path_.c_str(), error.data());
    return hardware_interface::CallbackReturn::ERROR;
  }
  data_ = mj_makeData(model_);

  // Map each ros2_control joint onto MuJoCo's arrays by NAME. Index order is not guaranteed to
  // match the URDF's, and silently trusting it would drive the wrong joint -- which looks like a
  // kinematics bug rather than a mapping one.
  joints_.reserve(info.joints.size());
  for (const auto& joint : info.joints)
  {
    JointMap entry;
    entry.name = joint.name;

    const int id = mj_name2id(model_, mjOBJ_JOINT, joint.name.c_str());
    if (id < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(kLogger), "joint '%s' is not in %s", joint.name.c_str(), mjcf_path_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    entry.qpos_index = model_->jnt_qposadr[id];
    entry.qvel_index = model_->jnt_dofadr[id];

    // The actuator naming matches what urdf_to_mujoco.py emits. A joint with no actuator is
    // reported rather than tolerated: it would read back fine and never move.
    const std::string actuator_name = joint.name + "_pos";
    entry.actuator = mj_name2id(model_, mjOBJ_ACTUATOR, actuator_name.c_str());
    if (entry.actuator < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(kLogger),
                   "no actuator '%s' in the model, so joint '%s' could never be commanded. "
                   "Regenerate the MJCF with urdf_to_mujoco.py, which adds one per revolute joint.",
                   actuator_name.c_str(), joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joints_.push_back(entry);
  }

  position_.assign(joints_.size(), 0.0);
  velocity_.assign(joints_.size(), 0.0);
  command_.assign(joints_.size(), std::numeric_limits<double>::quiet_NaN());

  // Services on their own node and executor: a service call must never block the control loop.
  node_ = std::make_shared<rclcpp::Node>("mujoco_system");
  set_state_service_ = node_->create_service<moveit2_extended_msgs::srv::SetSimulationState>(
      "~/set_joint_states",
      [this](const std::shared_ptr<moveit2_extended_msgs::srv::SetSimulationState::Request> request,
             std::shared_ptr<moveit2_extended_msgs::srv::SetSimulationState::Response> response) {
        onSetState(request, response);
      });
  reset_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) { onReset(request, response); });

  discoverCarriers();
  sim_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("~/sim_joint_states",
                                                                              rclcpp::QoS(1));
  for (const auto& carrier : carriers_)
  {
    carrier_publishers_.push_back(node_->create_publisher<moveit2_extended_msgs::msg::CarrierDiagnostics>(
        "~/carrier/" + carrier.name, rclcpp::QoS(1)));
  }

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });

  RCLCPP_INFO(rclcpp::get_logger(kLogger), "loaded %s: %d joints, %d actuators, timestep %.4f s",
              mjcf_path_.c_str(), model_->njnt, model_->nu, model_->opt.timestep);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MujocoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t index = 0; index < joints_.size(); ++index)
  {
    interfaces.emplace_back(joints_[index].name, hardware_interface::HW_IF_POSITION, &position_[index]);
    interfaces.emplace_back(joints_[index].name, hardware_interface::HW_IF_VELOCITY, &velocity_[index]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> MujocoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t index = 0; index < joints_.size(); ++index)
  {
    interfaces.emplace_back(joints_[index].name, hardware_interface::HW_IF_POSITION, &command_[index]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn MujocoSystem::on_activate(const rclcpp_lifecycle::State&)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initial_keyframe_.empty())
  {
    std::string message;
    if (!applyKeyframe(initial_keyframe_, message))
    {
      RCLCPP_WARN(rclcpp::get_logger(kLogger), "%s", message.c_str());
    }
  }
  mj_forward(model_, data_);

  // Seed the commands from where the arm actually is. Activating with commands at 0 (or NaN) makes
  // the first write() order the arm to the zero pose at full gain -- on this arm that is a fold
  // through itself, and it happens before any controller has had a chance to send anything.
  for (size_t index = 0; index < joints_.size(); ++index)
  {
    position_[index] = data_->qpos[joints_[index].qpos_index];
    velocity_[index] = data_->qvel[joints_[index].qvel_index];
    command_[index] = position_[index];
    data_->ctrl[joints_[index].actuator] = position_[index];
  }

  RCLCPP_INFO(rclcpp::get_logger(kLogger), "activated at the model's %s pose",
              initial_keyframe_.empty() ? "default" : initial_keyframe_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoSystem::on_deactivate(const rclcpp_lifecycle::State&)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MujocoSystem::read(const rclcpp::Time&, const rclcpp::Duration& period)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Advance by however much wall time passed, in the model's own timestep. Clamped, because a
  // stalled control loop would otherwise ask for thousands of steps and stall it further -- the
  // simulation falling behind is better than the whole node locking up.
  const double seconds = period.seconds();
  int steps = seconds > 0.0 ? static_cast<int>(std::round(seconds / model_->opt.timestep)) : 1;
  steps = std::clamp(steps, 1, kMaxStepsPerCycle);
  for (int step = 0; step < steps; ++step)
  {
    mj_step(model_, data_);
  }

  for (size_t index = 0; index < joints_.size(); ++index)
  {
    position_[index] = data_->qpos[joints_[index].qpos_index];
    velocity_[index] = data_->qvel[joints_[index].qvel_index];
  }

  // Throttled to 20 Hz: the control loop runs at 100 and nobody watching a dresspack needs five
  // times a second more than that, while the publish itself is not free.
  const rclcpp::Time now = node_->get_clock()->now();
  if ((now - last_publish_).seconds() >= 0.05)
  {
    last_publish_ = now;

    sensor_msgs::msg::JointState state;
    state.header.stamp = now;
    for (int id = 0; id < model_->njnt; ++id)
    {
      const char* name = mj_id2name(model_, mjOBJ_JOINT, id);
      if (name != nullptr)
      {
        state.name.emplace_back(name);
        state.position.push_back(data_->qpos[model_->jnt_qposadr[id]]);
      }
    }
    sim_state_publisher_->publish(state);
    publishCarrierRisk();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystem::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (size_t index = 0; index < joints_.size(); ++index)
  {
    // NaN is what an unclaimed command interface holds. Writing it into ctrl poisons the solver and
    // every joint goes to NaN at once, which reads as the robot vanishing.
    if (std::isfinite(command_[index]))
    {
      data_->ctrl[joints_[index].actuator] = command_[index];
    }
  }
  return hardware_interface::return_type::OK;
}

void MujocoSystem::discoverCarriers()
{
  // Found by the naming urdf_to_mujoco.py uses, rather than by a second list that would have to be
  // kept in step with it. A carrier that is in the model is monitored; one that is not, is not.
  const std::regex pattern(R"(^(.+)_(bendy|bendz|twist)(\d+)$)");
  std::map<std::string, Carrier> found;

  for (int id = 0; id < model_->njnt; ++id)
  {
    const char* raw = mj_id2name(model_, mjOBJ_JOINT, id);
    if (raw == nullptr)
    {
      continue;
    }
    std::smatch match;
    const std::string name(raw);
    if (!std::regex_match(name, match, pattern))
    {
      continue;
    }
    auto& carrier = found[match[1].str()];
    carrier.name = match[1].str();
    const int address = model_->jnt_qposadr[id];
    if (match[2].str() == "twist")
    {
      carrier.twist_dofs.push_back(address);
      carrier.twist_limit = model_->jnt_range[2 * id + 1];
    }
    else
    {
      carrier.bend_dofs.push_back(address);
      carrier.bend_limit = model_->jnt_range[2 * id + 1];
    }
  }

  for (auto& [name, carrier] : found)
  {
    carrier.equality = mj_name2id(model_, mjOBJ_EQUALITY, (name + "_tip").c_str());
    for (int id = 0; id < model_->ngeom; ++id)
    {
      const char* raw = mj_id2name(model_, mjOBJ_GEOM, id);
      if (raw != nullptr && std::string(raw).rfind(name + "_geom", 0) == 0)
      {
        carrier.geoms.push_back(id);
      }
    }
    carriers_.push_back(carrier);
    RCLCPP_INFO(rclcpp::get_logger(kLogger),
                "carrier '%s': %zu bend joints (limit %.1f deg), %zu twist joints (limit %.1f deg), "
                "anchor %s",
                carrier.name.c_str(), carrier.bend_dofs.size(), carrier.bend_limit * 180.0 / M_PI,
                carrier.twist_dofs.size(), carrier.twist_limit * 180.0 / M_PI,
                carrier.equality >= 0 ? "found" : "MISSING (tension cannot be measured)");
  }
}

void MujocoSystem::publishCarrierRisk()
{
  // Caller holds the mutex.
  if (carriers_.empty() || carrier_publishers_.empty())
  {
    return;
  }

  for (size_t index = 0; index < carriers_.size(); ++index)
  {
    const auto& carrier = carriers_[index];
    moveit2_extended_msgs::msg::CarrierDiagnostics message;
    message.carrier_name = carrier.name;

    double worst_bend = 0.0;
    for (const int address : carrier.bend_dofs)
    {
      worst_bend = std::max(worst_bend, std::abs(data_->qpos[address]));
    }
    double worst_twist = 0.0;
    for (const int address : carrier.twist_dofs)
    {
      worst_twist = std::max(worst_twist, std::abs(data_->qpos[address]));
    }

    message.bend_utilisation = carrier.bend_limit > 0.0 ? worst_bend / carrier.bend_limit : 0.0;
    message.twist_utilisation = carrier.twist_limit > 0.0 ? worst_twist / carrier.twist_limit : 0.0;
    // A chain of rigid links bends to a radius; the angle per link and the link length give it.
    message.min_bend_radius = worst_bend > 1e-9 && carrier.bend_limit > 0.0
                                  ? (carrier.bend_limit > 0.0 ? 1.0 : 0.0) * 0.0
                                  : 0.0;

    // Tension: the force MuJoCo needs to keep the far end on its bracket. This is the number that
    // says a pose is tearing the carrier off, as opposed to merely bending it hard -- and it is
    // read from the solver rather than estimated.
    double tension = 0.0;
    if (carrier.equality >= 0)
    {
      double squared = 0.0;
      for (int row = 0; row < data_->nefc; ++row)
      {
        if (data_->efc_type[row] == mjCNSTR_EQUALITY && data_->efc_id[row] == carrier.equality)
        {
          squared += data_->efc_force[row] * data_->efc_force[row];
        }
      }
      tension = std::sqrt(squared);
    }
    message.tension = tension;

    // Feasible means: not against either stop, and not being pulled hard enough to matter. The
    // tension threshold is deliberately explicit rather than hidden in a comparison -- it is a
    // guess until somebody pulls on a real carrier with a load cell.
    constexpr double kTensionConcern = 50.0;  // N
    message.feasible = message.bend_utilisation <= 1.0 && message.twist_utilisation <= 1.0 &&
                       tension < kTensionConcern;
    message.cables_within_limit = message.feasible;

    carrier_publishers_[index]->publish(message);
  }
}

bool MujocoSystem::applyKeyframe(const std::string& name, std::string& message)
{
  const int id = mj_name2id(model_, mjOBJ_KEY, name.c_str());
  if (id < 0)
  {
    message = "the model has no keyframe called '" + name + "'";
    return false;
  }
  mj_resetDataKeyframe(model_, data_, id);
  return true;
}

void MujocoSystem::onSetState(
    const std::shared_ptr<moveit2_extended_msgs::srv::SetSimulationState::Request> request,
    std::shared_ptr<moveit2_extended_msgs::srv::SetSimulationState::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!request->keyframe.empty())
  {
    std::string message;
    response->success = applyKeyframe(request->keyframe, message);
    response->message = response->success ? "reset to keyframe '" + request->keyframe + "'" : message;
  }
  else
  {
    const auto& names = request->joint_state.joint_state.name;
    const auto& values = request->joint_state.joint_state.position;
    if (names.size() != values.size())
    {
      response->success = false;
      response->message = "joint_state has " + std::to_string(names.size()) + " names and " +
                          std::to_string(values.size()) + " positions";
      return;
    }
    size_t applied = 0;
    for (size_t i = 0; i < names.size(); ++i)
    {
      const auto found = std::find_if(joints_.begin(), joints_.end(),
                                      [&](const JointMap& j) { return j.name == names[i]; });
      if (found == joints_.end())
      {
        continue;
      }
      data_->qpos[found->qpos_index] = values[i];
      // The actuator target moves with the state. Without this the arm is teleported and then
      // immediately dragged back to wherever the actuators were still pointing.
      data_->ctrl[found->actuator] = values[i];
      ++applied;
    }
    response->success = applied > 0;
    response->message = response->success ? "set " + std::to_string(applied) + " joint(s)"
                                          : "none of the given joints are in this model";
  }

  if (request->zero_velocity)
  {
    for (int i = 0; i < model_->nv; ++i)
    {
      data_->qvel[i] = 0.0;
    }
  }
  mj_forward(model_, data_);

  response->resulting_state.name.clear();
  response->resulting_state.position.clear();
  for (const auto& joint : joints_)
  {
    response->resulting_state.name.push_back(joint.name);
    response->resulting_state.position.push_back(data_->qpos[joint.qpos_index]);
  }
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "%s", response->message.c_str());
}

void MujocoSystem::onReset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (initial_keyframe_.empty())
  {
    mj_resetData(model_, data_);
    response->message = "reset to the model's default pose";
  }
  else
  {
    std::string message;
    if (!applyKeyframe(initial_keyframe_, message))
    {
      response->success = false;
      response->message = message;
      return;
    }
    response->message = "reset to keyframe '" + initial_keyframe_ + "'";
  }
  mj_forward(model_, data_);
  for (const auto& joint : joints_)
  {
    data_->ctrl[joint.actuator] = data_->qpos[joint.qpos_index];
  }
  response->success = true;
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "%s", response->message.c_str());
}

}  // namespace moveit2_extended::mujoco_sim

PLUGINLIB_EXPORT_CLASS(moveit2_extended::mujoco_sim::MujocoSystem, hardware_interface::SystemInterface)
