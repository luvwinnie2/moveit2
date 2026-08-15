// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// Publishes the solved carrier shape as markers, so the deformable geometry MoveIt is actually
// planning against is visible in RViz rather than inferred.
//
// Every physical parameter is a live ROS 2 parameter. That is what lets the RViz panel
// (rviz/carrier_panel.cpp) retune the carrier and see the answer immediately, instead of editing
// YAML and restarting the stack for each trial.
//
// Three states are tracked:
//   /joint_states          the robot as it is now
//   /motion_plan_request   the goal RViz asked for, so the carrier jumps to the pose you dragged
//                          to as soon as you hit Plan
//   /display_planned_path  the whole planned motion, which is animated and, more usefully,
//                          scanned for its worst case
//
// RViz does *not* publish the goal state while you drag the interactive marker -- the query state
// lives inside the MotionPlanning display and never reaches the graph -- so live following during
// a drag is not possible without patching that plugin. Planning is the trigger instead, which is
// also the moment the answer actually matters.
//
//   ros2 run moveit_cable_carrier carrier_visualizer --ros-args
//        -p robot_description:="$(cat robot.urdf)" -p carrier_config:=<carrier.yaml>

#include <moveit_cable_carrier/carrier_attacher.hpp>
#include <moveit_cable_carrier/carrier_registry.hpp>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <srdfdom/model.h>
#include <std_msgs/msg/string.hpp>
#include <urdf_parser/urdf_parser.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace
{
constexpr double kDeg = M_PI / 180.0;
}

class CarrierVisualizer : public rclcpp::Node
{
public:
  CarrierVisualizer() : rclcpp::Node("carrier_visualizer")
  {
    const std::string urdf = declare_parameter<std::string>("robot_description", "");
    const std::string srdf = declare_parameter<std::string>("robot_description_semantic", "");
    const std::string config = declare_parameter<std::string>("carrier_config", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "");
    const double rate = declare_parameter<double>("rate", 20.0);

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
    srdf_model->initString(*urdf_model, srdf.empty() ? "<robot name='r'/>" : srdf);
    model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
    state_ = std::make_shared<moveit::core::RobotState>(model_);
    state_->setToDefaultValues();

    std::string err;
    if (!moveit_cable_carrier::CarrierRegistry::instance().loadFromYaml(config, &err))
    {
      throw std::runtime_error("cannot load carrier config: " + err);
    }
    base_ = moveit_cable_carrier::CarrierRegistry::instance().carriers();
    if (base_.empty())
    {
      throw std::runtime_error("carrier config declares no carriers");
    }
    if (frame_id_.empty())
    {
      frame_id_ = model_->getModelFrame();
    }

    // Tunables. Defaults come from the YAML entry that is selected, so the panel starts showing
    // the configured carrier rather than arbitrary numbers.
    const auto& first = base_.front();
    declare_parameter<int>("carrier_index", 0);
    declare_parameter<double>("length", first.length);
    declare_parameter<double>("bend_radius", first.bend_radius);
    declare_parameter<double>("safety_margin", first.safety_margin);
    declare_parameter<double>("twist_limit_deg", first.twist_limit_per_link / kDeg);
    declare_parameter<double>("gravity_sag", first.gravity_sag);
    declare_parameter<double>("energy_relaxation", first.energy_relaxation);
    declare_parameter<int>("num_segments", first.num_segments);
    declare_parameter<double>("cable_outer_diameter",
                              first.inner_cables.empty() ? 0.0 : first.inner_cables.front().outer_diameter);
    declare_parameter<double>("cable_min_bend_factor",
                              first.inner_cables.empty() ? 0.0 : first.inner_cables.front().min_bend_factor);
    declare_parameter<bool>("show_all", true);

    rebuild();
    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& p) { return onParameters(p); });

    markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("cable_carrier_markers", 1);
    status_ = create_publisher<std_msgs::msg::String>("cable_carrier_status", rclcpp::QoS(1).transient_local());
    plan_req_ = create_subscription<moveit_msgs::msg::MotionPlanRequest>(
        "motion_plan_request", 2,
        [this](const moveit_msgs::msg::MotionPlanRequest::SharedPtr msg) { onPlanRequest(*msg); });
    plan_path_ = create_subscription<moveit_msgs::msg::DisplayTrajectory>(
        "display_planned_path", 2,
        [this](const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg) { onPlannedPath(*msg); });
    joints_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { onJointState(*msg); });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, rate)),
                               [this] { publish(); });

    RCLCPP_INFO(get_logger(), "carrier_visualizer: %zu carrier(s), frame '%s'", base_.size(), frame_id_.c_str());
  }

private:
  /** Apply the tunable parameters on top of the YAML entry and rebuild the solvers. */
  void rebuild()
  {
    const int index = std::clamp(get_parameter("carrier_index").as_int(), 0L,
                                 static_cast<long>(base_.size()) - 1L);
    tuned_ = base_;
    auto& c = tuned_[static_cast<size_t>(index)];
    c.length = get_parameter("length").as_double();
    c.bend_radius = get_parameter("bend_radius").as_double();
    c.safety_margin = get_parameter("safety_margin").as_double();
    c.twist_limit_per_link = get_parameter("twist_limit_deg").as_double() * kDeg;
    c.gravity_sag = get_parameter("gravity_sag").as_double();
    c.energy_relaxation = get_parameter("energy_relaxation").as_double();
    c.num_segments = static_cast<int>(get_parameter("num_segments").as_int());

    const double od = get_parameter("cable_outer_diameter").as_double();
    const double factor = get_parameter("cable_min_bend_factor").as_double();
    if (od > 0.0 && factor > 0.0)
    {
      if (c.inner_cables.empty())
      {
        c.inner_cables.emplace_back();
      }
      c.inner_cables.front().outer_diameter = od;
      c.inner_cables.front().min_bend_factor = factor;
    }

    active_index_ = static_cast<size_t>(index);
    attachers_.clear();
    for (const auto& p : tuned_)
    {
      auto a = std::make_shared<moveit_cable_carrier::CarrierAttacher>(p, model_);
      if (a->valid())
      {
        attachers_.push_back(std::move(a));
      }
      else
      {
        RCLCPP_WARN(get_logger(), "carrier '%s' references unknown links; skipped", p.name.c_str());
        attachers_.push_back(nullptr);
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult onParameters(const std::vector<rclcpp::Parameter>& params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto& p : params)
    {
      // Reject values that cannot describe a carrier before they reach the solver, so a slip in
      // the panel produces a message rather than a silently absurd shape.
      if (p.get_name() == "length" && p.as_double() <= 0.0)
      {
        result.successful = false;
        result.reason = "length must be positive";
      }
      else if (p.get_name() == "bend_radius" && p.as_double() <= 0.0)
      {
        result.successful = false;
        result.reason = "bend_radius must be positive";
      }
      else if (p.get_name() == "num_segments" && p.as_int() < 4)
      {
        result.successful = false;
        result.reason = "num_segments must be at least 4";
      }
    }
    if (result.successful)
    {
      pending_rebuild_ = true;
    }
    return result;
  }

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

  /** RViz publishes the request when you press Plan. Joint goals carry the exact configuration the
   *  interactive marker was dragged to, which is what lets the carrier show that pose. */
  void onPlanRequest(const moveit_msgs::msg::MotionPlanRequest& req)
  {
    if (req.goal_constraints.empty())
    {
      return;
    }
    auto goal = std::make_shared<moveit::core::RobotState>(*state_);
    bool any = false;
    for (const auto& jc : req.goal_constraints.front().joint_constraints)
    {
      if (model_->hasJointModel(jc.joint_name))
      {
        goal->setJointPositions(jc.joint_name, &jc.position);
        any = true;
      }
    }
    if (!any)
    {
      // A pose goal carries no joint values; solving IK here would just duplicate what RViz
      // already did, and could pick a different branch, so the goal preview is simply skipped.
      return;
    }
    goal->update();
    goal_state_ = goal;
  }

  /** Scan the whole planned motion, not just its endpoints: a carrier is usually fine at both ends
   *  and over-bent somewhere in the middle. */
  void onPlannedPath(const moveit_msgs::msg::DisplayTrajectory& msg)
  {
    plan_points_.clear();
    plan_joints_.clear();
    plan_cursor_ = 0;
    plan_summary_.clear();
    if (msg.trajectory.empty())
    {
      return;
    }
    const auto& jt = msg.trajectory.front().joint_trajectory;
    plan_joints_ = jt.joint_names;
    for (const auto& pt : jt.points)
    {
      plan_points_.push_back(pt.positions);
    }
    if (plan_points_.empty() || attachers_.empty() || !attachers_[active_index_])
    {
      return;
    }

    moveit::core::RobotState probe(*state_);
    double tightest = 1e30;
    double worst_ratio = 1e30;
    double required = 0.0;
    long infeasible = 0, cable_bad = 0;
    for (const auto& q : plan_points_)
    {
      for (size_t j = 0; j < plan_joints_.size() && j < q.size(); ++j)
      {
        probe.setJointPositions(plan_joints_[j], &q[j]);
      }
      probe.update();
      const auto shape = attachers_[active_index_]->computeShape(probe);
      if (!shape.feasible)
      {
        ++infeasible;
        continue;
      }
      tightest = std::min(tightest, shape.min_bend_radius());
      if (shape.required_cable_bend_ratio > 0.0)
      {
        required = shape.required_cable_bend_ratio;
        worst_ratio = std::min(worst_ratio, shape.min_cable_bend_ratio);
        if (!shape.cablesWithinLimit())
        {
          ++cable_bad;
        }
      }
    }
    std::ostringstream os;
    os << "plan_points=" << plan_points_.size() << ";plan_infeasible=" << infeasible
       << ";plan_min_radius_mm=" << (tightest < 1e29 ? 1000.0 * tightest : 0.0)
       << ";plan_cable_ratio=" << (worst_ratio < 1e29 ? worst_ratio : 0.0)
       << ";plan_cable_required=" << required << ";plan_cable_violations=" << cable_bad;
    plan_summary_ = os.str();
    RCLCPP_INFO(get_logger(),
                "plan scanned: %zu points, %ld infeasible, tightest %.1f mm, worst cable %.1fx OD "
                "(needs %.0fx) on %ld points",
                plan_points_.size(), infeasible, tightest < 1e29 ? 1000.0 * tightest : 0.0,
                worst_ratio < 1e29 ? worst_ratio : 0.0, required, cable_bad);
  }

  void publish()
  {
    if (pending_rebuild_)
    {
      // Rebuilt here rather than in the parameter callback: the callback runs before the values
      // are committed, so reading them there would apply the previous set.
      pending_rebuild_ = false;
      rebuild();
    }
    if (!have_state_ || attachers_.empty())
    {
      return;
    }

    const bool show_all = get_parameter("show_all").as_bool();
    visualization_msgs::msg::MarkerArray array;
    std::ostringstream status;
    int id = 0;

    for (size_t i = 0; i < attachers_.size(); ++i)
    {
      const bool active = (i == active_index_);
      if (!attachers_[i] || (!show_all && !active))
      {
        continue;
      }
      const auto shape = attachers_[i]->computeShape(*state_);
      const auto& p = attachers_[i]->params();

      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame_id_;
      m.header.stamp = now();
      m.ns = p.name;
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 2.0 * shape.radius;
      // Green when the carrier can take this shape, amber when it can but a cable inside is
      // over-bent, red when the shape is impossible. The amber case is the one that is easy to
      // miss: the carrier looks fine and the cable inside is the part being destroyed.
      const bool cable_ok = shape.cablesWithinLimit();
      m.color.r = !shape.feasible ? 0.9f : (cable_ok ? 0.1f : 0.95f);
      m.color.g = !shape.feasible ? 0.1f : (cable_ok ? 0.8f : 0.65f);
      m.color.b = 0.2f;
      m.color.a = active ? 0.6f : 0.3f;
      for (const auto& node : shape.nodes)
      {
        geometry_msgs::msg::Point pt;
        pt.x = node.x();
        pt.y = node.y();
        pt.z = node.z();
        m.points.push_back(pt);
      }
      array.markers.push_back(std::move(m));

      if (active)
      {
        status << "name=" << p.name << ";feasible=" << (shape.feasible ? 1 : 0)
               << ";radius_mm=" << 1000.0 * shape.min_bend_radius()
               << ";bend_util=" << shape.bend_utilisation << ";twist_util=" << shape.twist_utilisation
               << ";cable=" << (shape.worst_cable.empty() ? "-" : shape.worst_cable)
               << ";cable_ratio=" << shape.min_cable_bend_ratio
               << ";cable_required=" << shape.required_cable_bend_ratio
               << ";cable_ok=" << (cable_ok ? 1 : 0) << ";strain=" << shape.max_cable_strain
               << ";iterations=" << shape.iterations << ";length=" << p.length
               << ";bend_radius=" << p.bend_radius << ";margin=" << p.safety_margin;
      }
    }

    // The pose the interactive marker was dragged to, shown once Plan is pressed.
    if (goal_state_ && attachers_[active_index_])
    {
      appendCarrier(array, *goal_state_, active_index_, "_goal", 0.35f, id);
    }
    // Step through the planned motion so the carrier sweeps the path rather than showing one pose.
    if (!plan_points_.empty() && attachers_[active_index_])
    {
      moveit::core::RobotState probe(*state_);
      const auto& q = plan_points_[plan_cursor_ % plan_points_.size()];
      for (size_t j = 0; j < plan_joints_.size() && j < q.size(); ++j)
      {
        probe.setJointPositions(plan_joints_[j], &q[j]);
      }
      probe.update();
      appendCarrier(array, probe, active_index_, "_plan", 0.5f, id);
      ++plan_cursor_;
    }

    markers_->publish(array);
    std_msgs::msg::String s;
    s.data = status.str() + (plan_summary_.empty() ? "" : ";" + plan_summary_);
    status_->publish(s);
  }

  /** Draw one carrier for an arbitrary state under its own namespace. */
  void appendCarrier(visualization_msgs::msg::MarkerArray& array, const moveit::core::RobotState& st,
                     size_t index, const std::string& suffix, float alpha, int& id)
  {
    const auto shape = attachers_[index]->computeShape(st);
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = now();
    m.ns = attachers_[index]->params().name + suffix;
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 2.0 * shape.radius;
    const bool cable_ok = shape.cablesWithinLimit();
    m.color.r = !shape.feasible ? 0.9f : (cable_ok ? 0.2f : 0.95f);
    m.color.g = !shape.feasible ? 0.1f : (cable_ok ? 0.5f : 0.65f);
    m.color.b = !shape.feasible ? 0.2f : (cable_ok ? 0.95f : 0.2f);
    m.color.a = alpha;
    for (const auto& node : shape.nodes)
    {
      geometry_msgs::msg::Point pt;
      pt.x = node.x();
      pt.y = node.y();
      pt.z = node.z();
      m.points.push_back(pt);
    }
    array.markers.push_back(std::move(m));
  }

  moveit::core::RobotModelPtr model_;
  std::shared_ptr<moveit::core::RobotState> state_;
  std::vector<moveit_cable_carrier::CarrierParams> base_, tuned_;
  std::vector<moveit_cable_carrier::CarrierAttacherPtr> attachers_;
  size_t active_index_ = 0;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Subscription<moveit_msgs::msg::MotionPlanRequest>::SharedPtr plan_req_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr plan_path_;
  std::shared_ptr<moveit::core::RobotState> goal_state_;
  std::vector<std::vector<double>> plan_points_;
  std::vector<std::string> plan_joints_;
  size_t plan_cursor_ = 0;
  std::string plan_summary_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  std::string frame_id_;
  bool have_state_ = false;
  bool pending_rebuild_ = false;
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
