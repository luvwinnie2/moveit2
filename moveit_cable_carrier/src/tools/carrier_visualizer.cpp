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
// Live following of MoveIt's end-effector marker works by subscribing to that marker's *feedback*
// topic and solving IK here. RViz never publishes the query goal state itself -- it lives inside
// the MotionPlanning display -- but the pose being dragged does go out as InteractiveMarkerFeedback,
// which is enough to reconstruct the configuration. The IK is seeded from the previous preview so
// the arm keeps the same branch while dragging instead of flipping elbow-up/elbow-down, which is
// what RViz does internally too.
//
//   ros2 run moveit_cable_carrier carrier_visualizer --ros-args
//        -p robot_description:="$(cat robot.urdf)" -p carrier_config:=<carrier.yaml>

#include <moveit_cable_carrier/carrier_attacher.hpp>
#include <moveit_cable_carrier/carrier_registry.hpp>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
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
/** Largest joint move any single Jacobian iteration may make, radians. Small enough that a
 *  singular pseudo-inverse cannot throw the arm across its workspace, large enough that a normal
 *  drag converges within the iteration budget. */
constexpr double kMaxJointStepPerIteration = 0.02;
}

class CarrierVisualizer : public rclcpp::Node
{
public:
  CarrierVisualizer() : rclcpp::Node("carrier_visualizer")
  {
    declare_parameter<std::string>("robot_description", "");
    declare_parameter<std::string>("robot_description_semantic", "");
    declare_parameter<std::string>("carrier_config", "");
    declare_parameter<std::string>("planning_group", "arm");
    declare_parameter<std::string>("marker_feedback_topic",
                                   "/rviz_moveit_motion_planning_display/"
                                   "robot_interaction_interactive_marker_topic/feedback");
    declare_parameter<double>("ik_timeout", 0.05);
    // The link MoveIt puts the end-effector marker on. The SRDF declares the end-effector with
    // parent_link="flange", and that is where RViz places the handle, so the marker pose and the
    // IK target are the same frame.
    declare_parameter<std::string>("marker_link", "flange");
    // 0 disables the nearness test, which is the default, because this arm's kinematics plugin is
    // analytic and returns a single solution: the callback can only accept or reject it, never make
    // the solver look for a closer one. Rejecting therefore means the arm simply stops following --
    // measured, 40 of 40 updates failed at 0.6 rad for a 96 mm move that solves perfectly with the
    // test off. Set it above 0 only with a solver that enumerates solutions.
    declare_parameter<double>("max_ik_jump", 0.0);
    declare_parameter<double>("ik_tolerance", 0.002);
    declare_parameter<int>("ik_max_iterations", 40);
    declare_parameter<double>("ik_step_gain", 0.6);
    // Off by default, and it should stay off for normal use.
    //
    // The end-effector marker is a *query*: it asks what the arm and the carrier would look like at
    // a pose, so you can judge it before committing. Nothing may actually move until the trajectory
    // is planned and executed. Driving the real robot straight from the marker skips that entirely.
    //
    // The preview arm is published as a DisplayRobotState instead, so the carrier preview has a
    // visible arm to belong to -- without one it looks like the marker is dragging the carrier
    // around on its own.
    declare_parameter<bool>("drive_robot", false);
    declare_parameter<std::string>("start_pose", "ready");
    frame_id_ = declare_parameter<std::string>("frame_id", "");
    declare_parameter<double>("rate", 20.0);
  }

  bool jmgHasState(const std::string& name) const
  {
    const auto* g = model_->hasJointModelGroup(get_parameter("planning_group").as_string())
                        ? model_->getJointModelGroup(get_parameter("planning_group").as_string())
                        : nullptr;
    if (!g)
    {
      return false;
    }
    const auto& states = g->getDefaultStateNames();
    return std::find(states.begin(), states.end(), name) != states.end();
  }

  /** Deferred because RobotModelLoader needs a shared_ptr to this node, which does not exist yet
   *  inside the constructor. */
  void init()
  {
    const std::string urdf = get_parameter("robot_description").as_string();
    const std::string config = get_parameter("carrier_config").as_string();
    const double rate = get_parameter("rate").as_double();
    group_ = get_parameter("planning_group").as_string();

    if (urdf.empty() || config.empty())
    {
      RCLCPP_FATAL(get_logger(), "robot_description and carrier_config parameters are required");
      throw std::runtime_error("missing parameters");
    }

    // RobotModelLoader rather than a hand-built RobotModel: it also wires up the kinematics
    // solvers declared in robot_description_kinematics, which is what setFromIK needs to follow
    // the end-effector marker.
    robot_model_loader::RobotModelLoader::Options opt;
    opt.robot_description_ = "robot_description";
    opt.load_kinematics_solvers_ = true;
    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), opt);
    model_ = loader_->getModel();
    if (!model_)
    {
      throw std::runtime_error("RobotModelLoader could not build a robot model");
    }
    state_ = std::make_shared<moveit::core::RobotState>(model_);
    state_->setToDefaultValues();
    // The all-zeros pose is a singularity on this arm (the SRDF says so, which is why it defines a
    // separate 'home'), and starting a Jacobian follower at a singularity is asking for trouble.
    const std::string start_pose = get_parameter("start_pose").as_string();
    if (!start_pose.empty() && jmgHasState(start_pose))
    {
      state_->setToDefaultValues(model_->getJointModelGroup(get_parameter("planning_group").as_string()),
                                 start_pose);
    }
    state_->update();

    marker_link_ = get_parameter("marker_link").as_string();
    ik_client_ = create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");
    jmg_ = model_->hasJointModelGroup(group_) ? model_->getJointModelGroup(group_) : nullptr;
    if (!jmg_)
    {
      RCLCPP_WARN(get_logger(), "planning group '%s' not found; marker following disabled",
                  group_.c_str());
    }
    else if (!jmg_->getSolverInstance())
    {
      RCLCPP_WARN(get_logger(),
                  "group '%s' has no IK solver (is robot_description_kinematics passed to this "
                  "node?); marker following disabled",
                  group_.c_str());
    }

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
    // Colour of a preview (ghost) carrier when it is within every limit. Live-settable, so the
    // hue can be picked against whatever the scene looks like:
    //   ros2 param set /carrier_visualizer preview_color "[0.9, 0.4, 1.0]"
    declare_parameter<std::vector<double>>("preview_color", { 0.2, 0.9, 0.95 });

    rebuild();
    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& p) { return onParameters(p); });

    markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("cable_carrier_markers", 1);
    driven_joints_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    preview_robot_ = create_publisher<moveit_msgs::msg::DisplayRobotState>("carrier_preview_state",
                                                                          rclcpp::QoS(1).transient_local());
    status_ = create_publisher<std_msgs::msg::String>("cable_carrier_status", rclcpp::QoS(1).transient_local());
    marker_fb_ = create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>(
        get_parameter("marker_feedback_topic").as_string(), rclcpp::QoS(10),
        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr msg) {
          onMarkerFeedback(*msg);
        });
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
    // Deliberately does not touch preview_state_. In drive mode this message is our own echo and
    // already agrees with it; when the robot is driven by something else, the preview is what the
    // marker is asking for and must not be overwritten by the current pose.
  }

  /** Follow MoveIt's end-effector marker while it is being dragged.
   *
   *  The feedback carries the pose of the dragged control, so IK gives back the configuration RViz
   *  is showing as its goal state. Seeded from the last preview so the solution keeps the same
   *  branch across a drag rather than snapping between elbow-up and elbow-down. */
  void onMarkerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback& fb)
  {
    if (!jmg_ || !jmg_->getSolverInstance())
    {
      return;
    }
    if (fb.event_type != visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE &&
        fb.event_type != visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP)
    {
      return;
    }


    // The feedback pose is in the frame the marker was published in. Anything other than the model
    // frame would need a TF lookup; report it rather than silently placing the carrier wrongly.
    if (!fb.header.frame_id.empty() && fb.header.frame_id != model_->getModelFrame())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "marker feedback is in frame '%s' but the model frame is '%s'; "
                           "not following", fb.header.frame_id.c_str(),
                           model_->getModelFrame().c_str());
      return;
    }

    // Solve with MoveIt's own IK service rather than in-process.
    //
    // Calling setFromIK here looked equivalent and was not: measured against /compute_ik for the
    // same marker pose and the same seed, this node was picking solutions 2.5-2.9 rad from the
    // current pose while MoveIt picked ones 0.5-0.8 rad away. The previewed carrier was therefore
    // hanging on an arm configuration that is never drawn, which is exactly what "the preview
    // floats in mid-air" looks like. Going through the service makes the preview agree with the
    // ghost the user is actually dragging, by construction.
    if (!ik_client_->service_is_ready())
    {
      return;
    }

    auto req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    req->ik_request.group_name = group_;
    req->ik_request.ik_link_name = marker_link_;
    req->ik_request.timeout = rclcpp::Duration::from_seconds(get_parameter("ik_timeout").as_double());
    req->ik_request.pose_stamped.header.frame_id = model_->getModelFrame();
    req->ik_request.pose_stamped.pose = fb.pose;
    moveit::core::robotStateToRobotStateMsg(*state_, req->ik_request.robot_state);

    // Fire and forget: the reply lands on the executor thread and only stores a state, so a slow
    // solve cannot stall the marker callback and back the subscription queue up.
    ik_client_->async_send_request(
        req, [this](rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future) {
          const auto res = future.get();
          if (!res || res->error_code.val != res->error_code.SUCCESS)
          {
            // Unreachable at this pose. Keep the last good preview rather than blanking it;
            // dragging past the envelope is normal.
            return;
          }
          auto solved = std::make_shared<moveit::core::RobotState>(*state_);
          const auto& js = res->solution.joint_state;
          for (size_t i = 0; i < js.name.size() && i < js.position.size(); ++i)
          {
            if (model_->hasJointModel(js.name[i]))
            {
              solved->setJointPositions(js.name[i], &js.position[i]);
            }
          }
          solved->update();
          preview_state_ = solved;
          publishPreviewArm();
          driveRobot();
        });
    return;

  }

  /** Publish the queried configuration as a ghost robot.
   *
   *  This is what makes the preview readable: the carrier drawn for that configuration now sits on
   *  an arm you can see, instead of floating in space where it looks like the marker is dragging
   *  the carrier itself. The real robot is untouched -- it moves when the plan is executed. */
  void publishPreviewArm()
  {
    if (!preview_state_)
    {
      return;
    }
    moveit_msgs::msg::DisplayRobotState msg;
    moveit::core::robotStateToRobotStateMsg(*preview_state_, msg.state);
    preview_robot_->publish(msg);
  }

  /** Push the followed configuration out as the robot's joint state, so the arm itself moves and
   *  the carrier follows it. Only one publisher may own /joint_states, so the launch drops
   *  joint_state_publisher when this is on. */
  void driveRobot()
  {
    if (!preview_state_ || !get_parameter("drive_robot").as_bool())
    {
      return;
    }
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    for (const auto* jm : jmg_->getActiveJointModels())
    {
      msg.name.push_back(jm->getName());
      msg.position.push_back(preview_state_->getVariablePosition(jm->getFirstVariableIndex()));
    }
    driven_joints_->publish(msg);
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
    // With drive_robot on this node owns /joint_states, so it has to keep publishing whether or not
    // the marker is being touched. Publishing only on feedback leaves robot_state_publisher with
    // nothing between drags, TF goes stale, and both RViz and any TF lookup stop working -- which
    // looks exactly like "it does not move at some positions".
    if (get_parameter("drive_robot").as_bool() && jmg_)
    {
      const auto& source = preview_state_ ? *preview_state_ : *state_;
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = now();
      for (const auto* jm : jmg_->getActiveJointModels())
      {
        msg.name.push_back(jm->getName());
        msg.position.push_back(source.getVariablePosition(jm->getFirstVariableIndex()));
      }
      driven_joints_->publish(msg);
    }

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

    // The queried configuration's carrier. Drawn alongside the current one so the two can be
    // compared: this is what the run would look like if the arm went where the marker is.
    if (preview_state_ && attachers_[active_index_] && !get_parameter("drive_robot").as_bool())
    {
      appendCarrier(array, *preview_state_, active_index_, "_marker", 0.55f, id);
    }
    // The pose the interactive marker was dragged to, shown once Plan is pressed.
    if (goal_state_ && attachers_[active_index_])
    {
      appendCarrier(array, *goal_state_, active_index_, "_goal", 0.35f, id);
    }
    // Sweep the planned motion once, then clear it. Looping it for ever -- which is what the
    // modulo used to do -- leaves the display animating long after the motion is finished and
    // reads as the robot moving on its own, especially since some intermediate poses draw the
    // carrier red.
    if (!plan_points_.empty() && attachers_[active_index_])
    {
      if (plan_cursor_ < plan_points_.size())
      {
        moveit::core::RobotState probe(*state_);
        const auto& q = plan_points_[plan_cursor_];
        for (size_t j = 0; j < plan_joints_.size() && j < q.size(); ++j)
        {
          probe.setJointPositions(plan_joints_[j], &q[j]);
        }
        probe.update();
        appendCarrier(array, probe, active_index_, "_plan", 0.5f, id);
        ++plan_cursor_;
      }
      else
      {
        // One pass done: drop every ghost so what is left on screen is the robot as it now is.
        deleteCarrier(array, active_index_, "_plan");
        deleteCarrier(array, active_index_, "_goal");
        deleteCarrier(array, active_index_, "_marker");
        plan_points_.clear();
        goal_state_.reset();
        preview_state_.reset();
      }
    }

    markers_->publish(array);
    std_msgs::msg::String s;
    s.data = status.str() + (plan_summary_.empty() ? "" : ";" + plan_summary_);
    status_->publish(s);
  }

  /** Stable marker id per carrier and role, so a ghost can be deleted again by name.
   *  Ids assigned in draw order cannot be deleted reliably because the order changes with which
   *  ghosts happen to exist that cycle. */
  static int markerId(size_t index, const std::string& suffix)
  {
    const int role = suffix.empty() ? 0 : suffix == "_marker" ? 1 : suffix == "_goal" ? 2 : 3;
    return static_cast<int>(index) * 10 + role;
  }

  /** Remove a ghost from the display. RViz keeps the last message for a namespace, so a ghost that
   *  simply stops being published stays on screen for ever. */
  void deleteCarrier(visualization_msgs::msg::MarkerArray& array, size_t index,
                     const std::string& suffix)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = now();
    m.ns = attachers_[index]->params().name + suffix;
    m.id = markerId(index, suffix);
    m.action = visualization_msgs::msg::Marker::DELETE;
    array.markers.push_back(std::move(m));
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
    m.id = markerId(index, suffix);
    (void)id;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 2.0 * shape.radius;
    // Preview carriers use a different hue family from the real one at *every* status, not just
    // when everything is fine. Sharing the amber "cable over-bent" colour made the two
    // indistinguishable exactly when it mattered -- and with a 10 mm cable inside, over-bent is the
    // common case, so both were amber most of the time.
    //
    //   real     : green  / amber   / red
    //   preview  : cyan   / magenta / dark red
    const bool cable_ok = shape.cablesWithinLimit();
    const auto rgb = get_parameter("preview_color").as_double_array();
    if (!shape.feasible)
    {
      m.color.r = 0.65f;
      m.color.g = 0.0f;
      m.color.b = 0.15f;
    }
    else if (!cable_ok)
    {
      m.color.r = 0.95f;
      m.color.g = 0.25f;
      m.color.b = 0.9f;
    }
    else
    {
      m.color.r = rgb.size() > 0 ? static_cast<float>(rgb[0]) : 0.2f;
      m.color.g = rgb.size() > 1 ? static_cast<float>(rgb[1]) : 0.9f;
      m.color.b = rgb.size() > 2 ? static_cast<float>(rgb[2]) : 0.95f;
    }
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
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr driven_joints_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr preview_robot_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Subscription<moveit_msgs::msg::MotionPlanRequest>::SharedPtr plan_req_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr plan_path_;
  rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr marker_fb_;
  robot_model_loader::RobotModelLoaderPtr loader_;
  const moveit::core::JointModelGroup* jmg_ = nullptr;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  std::string marker_link_;
  std::string group_;
  std::shared_ptr<moveit::core::RobotState> preview_state_;
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
    auto node = std::make_shared<CarrierVisualizer>();
    node->init();
    rclcpp::spin(node);
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
