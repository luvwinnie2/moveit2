// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// Subscribes to /joint_states, solves the carrier shape for the live robot configuration and
// publishes it as a MarkerArray so the deformable geometry MoveIt is planning against is
// actually visible in RViz.
//
//   ros2 run moveit_cable_carrier carrier_visualizer --ros-args
//        -p carrier_config:=<path to carrier.yaml>
//
// robot_description / robot_description_semantic are read as parameters, matching the usual
// move_group setup.

#include <moveit_cable_carrier/carrier_attacher.hpp>
#include <moveit_cable_carrier/carrier_registry.hpp>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

class CarrierVisualizer : public rclcpp::Node
{
public:
  CarrierVisualizer() : rclcpp::Node("carrier_visualizer")
  {
    const std::string urdf = declare_parameter<std::string>("robot_description", "");
    const std::string srdf = declare_parameter<std::string>("robot_description_semantic", "");
    const std::string config = declare_parameter<std::string>("carrier_config", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "");
    rate_hz_ = declare_parameter<double>("rate", 20.0);

    if (urdf.empty() || config.empty())
    {
      RCLCPP_FATAL(get_logger(), "robot_description and carrier_config parameters are required");
      throw std::runtime_error("missing parameters");
    }

    auto urdf_model = urdf::parseURDF(urdf);
    if (!urdf_model)
    {
      throw std::runtime_error("cannot parse robot_description");
    }
    auto srdf_model = std::make_shared<srdf::Model>();
    if (srdf.empty())
    {
      srdf_model->initString(*urdf_model, "<robot name='r'/>");
    }
    else
    {
      srdf_model->initString(*urdf_model, srdf);
    }
    model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
    state_ = std::make_shared<moveit::core::RobotState>(model_);
    state_->setToDefaultValues();

    std::string err;
    if (!moveit_cable_carrier::CarrierRegistry::instance().loadFromYaml(config, &err))
    {
      throw std::runtime_error("cannot load carrier config: " + err);
    }
    for (const auto& p : moveit_cable_carrier::CarrierRegistry::instance().carriers())
    {
      auto a = std::make_shared<moveit_cable_carrier::CarrierAttacher>(p, model_);
      if (a->valid())
      {
        attachers_.push_back(std::move(a));
      }
      else
      {
        RCLCPP_WARN(get_logger(), "carrier '%s' references unknown links; skipped", p.name.c_str());
      }
    }
    if (frame_id_.empty())
    {
      frame_id_ = model_->getModelFrame();
    }

    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("cable_carrier_markers", 1);
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { onJointState(*msg); });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_)),
                               [this] { publish(); });

    RCLCPP_INFO(get_logger(), "carrier_visualizer up: %zu carrier(s), frame '%s'", attachers_.size(),
                frame_id_.c_str());
  }

private:
  void onJointState(const sensor_msgs::msg::JointState& msg)
  {
    for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i)
    {
      if (model_->hasJointModel(msg.name[i]))
      {
        state_->setJointPositions(msg.name[i], &msg.position[i]);
      }
    }
    state_->update();
    have_state_ = true;
  }

  void publish()
  {
    if (!have_state_ || attachers_.empty())
    {
      return;
    }
    visualization_msgs::msg::MarkerArray array;
    int id = 0;
    for (const auto& attacher : attachers_)
    {
      const auto shape = attacher->computeShape(*state_);
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame_id_;
      m.header.stamp = now();
      m.ns = attacher->params().name;
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      const double d = 2.0 * shape.radius;
      m.scale.x = m.scale.y = m.scale.z = d;
      // Green when the carrier can physically take this shape, red when it cannot -- an
      // infeasible shape is reported to MoveIt as a collision.
      m.color.r = shape.feasible ? 0.1f : 0.9f;
      m.color.g = shape.feasible ? 0.8f : 0.1f;
      m.color.b = 0.2f;
      m.color.a = 0.55f;
      for (const auto& p : shape.nodes)
      {
        geometry_msgs::msg::Point pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        m.points.push_back(pt);
      }
      array.markers.push_back(std::move(m));
    }
    pub_->publish(array);
  }

  moveit::core::RobotModelPtr model_;
  std::shared_ptr<moveit::core::RobotState> state_;
  std::vector<moveit_cable_carrier::CarrierAttacherPtr> attachers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;
  double rate_hz_ = 20.0;
  bool have_state_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<CarrierVisualizer>());
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("carrier_visualizer"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
