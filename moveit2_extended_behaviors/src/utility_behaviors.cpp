// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_behaviors/utility_behaviors.hpp>

#include <moveit2_extended_core/path_utils.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>

namespace moveit2_extended::behaviors
{

// ---------------------------------------------------------------------------------------------
// CreatePoseStamped
// ---------------------------------------------------------------------------------------------

CreatePoseStamped::CreatePoseStamped(const std::string& name, const NodeConfig& config,
                                     BehaviorContextPtr shared_resources)
  : SyncBehaviorBase(name, config, std::move(shared_resources))
{
}

BT::PortsList CreatePoseStamped::providedPorts()
{
  return {
    BT::InputPort<std::string>("frame_id", "the frame the pose is expressed in; required"),
    BT::InputPort<double>("x", 0.0, "metres"),
    BT::InputPort<double>("y", 0.0, "metres"),
    BT::InputPort<double>("z", 0.0, "metres"),
    BT::InputPort<double>("roll", 0.0, "radians"),
    BT::InputPort<double>("pitch", 0.0, "radians"),
    BT::InputPort<double>("yaw", 0.0, "radians"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", ""),
  };
}

BtStatus CreatePoseStamped::tick()
{
  const auto frame = getInputOr<std::string>("frame_id", std::string(""));
  if (frame.empty())
  {
    // A pose with no frame is not a pose. Defaulting it is how a target ends up somewhere nobody
    // asked for, with nothing in the log.
    RCLCPP_ERROR(getLogger(), "frame_id must not be empty");
    return BtStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp = getSharedResources()->clock()->now();
  pose.pose.position.x = getInputOr<double>("x", 0.0);
  pose.pose.position.y = getInputOr<double>("y", 0.0);
  pose.pose.position.z = getInputOr<double>("z", 0.0);

  tf2::Quaternion q;
  q.setRPY(getInputOr<double>("roll", 0.0), getInputOr<double>("pitch", 0.0), getInputOr<double>("yaw", 0.0));
  q.normalize();
  pose.pose.orientation = tf2::toMsg(q);

  setOutput("pose", pose);
  return BtStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------
// TransformPose
// ---------------------------------------------------------------------------------------------

TransformPose::TransformPose(const std::string& name, const NodeConfig& config,
                             BehaviorContextPtr shared_resources)
  : SyncBehaviorBase(name, config, std::move(shared_resources))
{
}

BT::PortsList TransformPose::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "the pose to move"),
    BT::InputPort<double>("dx", 0.0, ""),
    BT::InputPort<double>("dy", 0.0, ""),
    BT::InputPort<double>("dz", 0.0, ""),
    BT::InputPort<double>("droll", 0.0, ""),
    BT::InputPort<double>("dpitch", 0.0, ""),
    BT::InputPort<double>("dyaw", 0.0, ""),
    BT::InputPort<bool>("local", true,
                        "true applies the offset in the pose's own frame (retreat along the tool axis); "
                        "false applies it in the parent frame (lift straight up)"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("transformed_pose", ""),
  };
}

BtStatus TransformPose::tick()
{
  const auto input = getInput<geometry_msgs::msg::PoseStamped>("pose");
  if (!input)
  {
    RCLCPP_ERROR(getLogger(), "pose: %s", input.error().c_str());
    return BtStatus::FAILURE;
  }

  Eigen::Isometry3d base;
  tf2::fromMsg(input->pose, base);

  tf2::Quaternion delta_rotation;
  delta_rotation.setRPY(getInputOr<double>("droll", 0.0), getInputOr<double>("dpitch", 0.0),
                        getInputOr<double>("dyaw", 0.0));
  delta_rotation.normalize();

  Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
  delta.translation() = Eigen::Vector3d(getInputOr<double>("dx", 0.0), getInputOr<double>("dy", 0.0),
                                        getInputOr<double>("dz", 0.0));
  delta.linear() = Eigen::Quaterniond(delta_rotation.w(), delta_rotation.x(), delta_rotation.y(),
                                      delta_rotation.z())
                       .toRotationMatrix();

  // Post-multiply for a local offset, pre-multiply for a parent-frame one. The difference is the
  // difference between backing out along the tool and lifting vertically.
  const Eigen::Isometry3d result = getInputOr<bool>("local", true) ? (base * delta) : (delta * base);

  geometry_msgs::msg::PoseStamped out;
  out.header = input->header;
  out.pose = tf2::toMsg(result);
  setOutput("transformed_pose", out);
  return BtStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------
// GetLatestTransform
// ---------------------------------------------------------------------------------------------

GetLatestTransform::GetLatestTransform(const std::string& name, const NodeConfig& config,
                                       BehaviorContextPtr shared_resources)
  : SyncBehaviorBase(name, config, std::move(shared_resources))
{
}

BT::PortsList GetLatestTransform::providedPorts()
{
  return {
    BT::InputPort<std::string>("target_frame", "the frame to express the result in"),
    BT::InputPort<std::string>("source_frame", "the frame whose pose is wanted"),
    BT::InputPort<double>("timeout", 0.5, "seconds to wait for the transform"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "source_frame's origin, in target_frame"),
  };
}

BtStatus GetLatestTransform::tick()
{
  const auto target = getInputOr<std::string>("target_frame", std::string(""));
  const auto source = getInputOr<std::string>("source_frame", std::string(""));
  if (target.empty() || source.empty())
  {
    RCLCPP_ERROR(getLogger(), "both target_frame and source_frame are required");
    return BtStatus::FAILURE;
  }

  try
  {
    const auto transform = getSharedResources()->tf()->lookupTransform(
        target, source, tf2::TimePointZero,
        tf2::durationFromSec(getInputOr<double>("timeout", 0.5)));

    geometry_msgs::msg::PoseStamped pose;
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    setOutput("pose", pose);
    return BtStatus::SUCCESS;
  }
  catch (const tf2::TransformException& exc)
  {
    RCLCPP_ERROR(getLogger(), "no transform %s <- %s: %s", target.c_str(), source.c_str(), exc.what());
    return BtStatus::FAILURE;
  }
}

// ---------------------------------------------------------------------------------------------
// GetJointState
// ---------------------------------------------------------------------------------------------

GetJointState::GetJointState(const std::string& name, const NodeConfig& config,
                             BehaviorContextPtr shared_resources)
  : SyncBehaviorBase(name, config, std::move(shared_resources))
{
}

BT::PortsList GetJointState::providedPorts()
{
  return {
    BT::InputPort<std::string>("planning_group", "arm", "restrict to this group's active joints"),
    BT::OutputPort<sensor_msgs::msg::JointState>("joint_state", "the current joint values"),
  };
}

BtStatus GetJointState::tick()
{
  const auto model = getSharedResources()->robotModel();
  const auto current = getSharedResources()->currentState();
  if (!model || !current)
  {
    RCLCPP_ERROR(getLogger(), "MoveIt is not available; is move_group running?");
    return BtStatus::FAILURE;
  }

  const auto group = getInputOr<std::string>("planning_group", std::string("arm"));
  const moveit::core::JointModelGroup* jmg = model->getJointModelGroup(group);
  if (!jmg)
  {
    RCLCPP_ERROR(getLogger(), "no planning group named '%s'", group.c_str());
    return BtStatus::FAILURE;
  }

  sensor_msgs::msg::JointState state;
  state.header.stamp = getSharedResources()->clock()->now();
  for (const auto& joint : jmg->getActiveJointModelNames())
  {
    state.name.push_back(joint);
    state.position.push_back(current->getVariablePosition(joint));
  }
  setOutput("joint_state", state);
  return BtStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------
// LoadObjectiveParameters
// ---------------------------------------------------------------------------------------------

LoadObjectiveParameters::LoadObjectiveParameters(const std::string& name, const NodeConfig& config,
                                                 BehaviorContextPtr shared_resources)
  : SyncBehaviorBase(name, config, std::move(shared_resources))
{
}

BT::PortsList LoadObjectiveParameters::providedPorts()
{
  return {
    BT::InputPort<std::string>("file_path", "package:// URI or absolute path"),
    BT::InputPort<std::string>("key_prefix", "", "prepended to every key written"),
    BT::InputPort<bool>("overwrite", false,
                        "false leaves keys that already exist alone, so a value passed in the goal wins "
                        "over the file"),
    BT::OutputPort<std::vector<std::string>>("loaded_keys", "what was written"),
  };
}

BtStatus LoadObjectiveParameters::tick()
{
  const auto path = getInputOr<std::string>("file_path", std::string(""));
  if (path.empty())
  {
    RCLCPP_ERROR(getLogger(), "file_path is required");
    return BtStatus::FAILURE;
  }

  std::string error;
  const std::string text = readFile(path, &error);
  if (text.empty())
  {
    RCLCPP_ERROR(getLogger(), "cannot read %s: %s", path.c_str(), error.c_str());
    return BtStatus::FAILURE;
  }

  YAML::Node root;
  try
  {
    root = YAML::Load(text);
  }
  catch (const YAML::Exception& exc)
  {
    RCLCPP_ERROR(getLogger(), "cannot parse %s: %s", path.c_str(), exc.what());
    return BtStatus::FAILURE;
  }
  if (!root || !root.IsMap())
  {
    RCLCPP_ERROR(getLogger(), "%s must contain a map of key -> value", path.c_str());
    return BtStatus::FAILURE;
  }

  const auto prefix = getInputOr<std::string>("key_prefix", std::string(""));
  const bool overwrite = getInputOr<bool>("overwrite", false);
  auto blackboard = config().blackboard;

  std::vector<std::string> written;
  for (const auto& entry : root)
  {
    const std::string key = prefix + entry.first.as<std::string>();
    if (!overwrite && blackboard->getAny(key) != nullptr)
    {
      // A value supplied in the goal beats the file, so an operator can override one number
      // without editing anything on disk.
      RCLCPP_DEBUG(getLogger(), "'%s' is already set; leaving it alone", key.c_str());
      continue;
    }
    // Written as a string: BehaviorTree.CPP converts on read to whatever the port declares, so
    // this works for port types this Behavior knows nothing about.
    std::string value;
    if (entry.second.IsScalar())
    {
      value = entry.second.as<std::string>();
    }
    else if (entry.second.IsSequence())
    {
      for (size_t i = 0; i < entry.second.size(); ++i)
      {
        value += (i ? ";" : "") + entry.second[i].as<std::string>();
      }
    }
    else
    {
      RCLCPP_WARN(getLogger(), "skipping '%s': only scalars and flat sequences can go on the blackboard",
                  key.c_str());
      continue;
    }
    blackboard->set(key, value);
    written.push_back(key);
  }

  setOutput("loaded_keys", written);
  return BtStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------
// GetElementOfVector
// ---------------------------------------------------------------------------------------------

GetElementOfVector::GetElementOfVector(const std::string& name, const NodeConfig& config,
                                       BehaviorContextPtr shared_resources)
  : SyncBehaviorBase(name, config, std::move(shared_resources))
{
}

BT::PortsList GetElementOfVector::providedPorts()
{
  return {
    BT::InputPort<std::vector<std::string>>("vector", "semicolon-separated list"),
    BT::InputPort<int>("index", 0, "0-based; negative counts from the end"),
    BT::OutputPort<std::string>("element", ""),
    BT::OutputPort<int>("size", "how many elements the list has"),
  };
}

BtStatus GetElementOfVector::tick()
{
  const auto vector = getInputOr<std::vector<std::string>>("vector", std::vector<std::string>{});
  setOutput("size", static_cast<int>(vector.size()));

  int index = getInputOr<int>("index", 0);
  if (index < 0)
  {
    index += static_cast<int>(vector.size());
  }
  if (index < 0 || index >= static_cast<int>(vector.size()))
  {
    RCLCPP_ERROR(getLogger(), "index %d is outside a list of %zu", getInputOr<int>("index", 0), vector.size());
    return BtStatus::FAILURE;
  }
  setOutput("element", vector[static_cast<size_t>(index)]);
  return BtStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------
// IsPoseNearIdentity
// ---------------------------------------------------------------------------------------------

IsPoseNearIdentity::IsPoseNearIdentity(const std::string& name, const NodeConfig& config,
                                       BehaviorContextPtr shared_resources)
  : ConditionBehaviorBase(name, config, std::move(shared_resources))
{
}

BT::PortsList IsPoseNearIdentity::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "usually a relative transform"),
    BT::InputPort<double>("position_tolerance", 0.005, "metres"),
    BT::InputPort<double>("orientation_tolerance", 0.05, "radians"),
  };
}

BtStatus IsPoseNearIdentity::tick()
{
  const auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose");
  if (!pose)
  {
    RCLCPP_ERROR(getLogger(), "pose: %s", pose.error().c_str());
    return BtStatus::FAILURE;
  }

  const auto& p = pose->pose.position;
  const double distance = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);

  tf2::Quaternion q(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z,
                    pose->pose.orientation.w);
  if (q.length2() < 1e-9)
  {
    RCLCPP_ERROR(getLogger(), "the pose has a zero-length quaternion");
    return BtStatus::FAILURE;
  }
  q.normalize();
  // Angle of the rotation, taking the shorter of the two equivalent representations.
  const double angle = 2.0 * std::acos(std::min(1.0, std::abs(q.w())));

  const bool near = distance <= getInputOr<double>("position_tolerance", 0.005) &&
                    angle <= getInputOr<double>("orientation_tolerance", 0.05);
  return near ? BtStatus::SUCCESS : BtStatus::FAILURE;
}

}  // namespace moveit2_extended::behaviors
