// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// Fits and removes end-effectors at run time.
//
// A tool is applied as an AttachedCollisionObject on the mount link, so the planner accounts for
// it immediately and no URDF, SRDF or node restart is involved. Switching a tool also switches the
// dresspack configuration that goes with it -- a heavier, longer tool works the carrier harder,
// and the two were never independent.

#include <moveit2_extended_tools/tool_registry.hpp>

#include <moveit2_extended_msgs/srv/get_active_tool.hpp>
#include <moveit2_extended_msgs/srv/list_tools.hpp>
#include <moveit2_extended_msgs/srv/switch_tool.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace moveit2_extended::tools
{
namespace srvs = moveit2_extended_msgs::srv;

class ToolManager
{
public:
  explicit ToolManager(const rclcpp::NodeOptions& options)
    : node_(std::make_shared<rclcpp::Node>("tool_manager", options))
  {
    node_->declare_parameter<std::string>("tool_registry", "");
    node_->declare_parameter<std::string>("initial_tool", "");
    node_->declare_parameter<std::string>("apply_planning_scene_service", "/apply_planning_scene");
    node_->declare_parameter<bool>("switch_carrier_with_tool", true);

    const std::string registry_file = node_->get_parameter("tool_registry").as_string();
    if (registry_file.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "no tool_registry configured; this node can do nothing");
    }
    else
    {
      std::string error;
      if (!registry_.loadFromFile(registry_file, &error))
      {
        RCLCPP_ERROR(node_->get_logger(), "cannot read %s: %s", registry_file.c_str(), error.c_str());
      }
      else
      {
        RCLCPP_INFO(node_->get_logger(), "%zu tool(s) available, mounting on '%s'", registry_.size(),
                    registry_.mount_link.c_str());
      }
    }

    // The planning-scene client gets its OWN callback group, and this is not optional.
    //
    // fitTool() waits on the response from inside a service callback. With the client in the
    // default (mutually exclusive) group, the executor cannot deliver that response while the
    // service callback is still running, so the wait times out every time and the tool never
    // changes -- which is exactly what happened the first time this ran.
    scene_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    scene_client_ = node_->create_client<moveit_msgs::srv::ApplyPlanningScene>(
        node_->get_parameter("apply_planning_scene_service").as_string(), rmw_qos_profile_services_default,
        scene_callback_group_);

    using namespace std::placeholders;
    switch_srv_ = node_->create_service<srvs::SwitchTool>("~/switch", std::bind(&ToolManager::onSwitch, this, _1, _2));
    list_srv_ = node_->create_service<srvs::ListTools>("~/list", std::bind(&ToolManager::onList, this, _1, _2));
    active_srv_ =
        node_->create_service<srvs::GetActiveTool>("~/active", std::bind(&ToolManager::onActive, this, _1, _2));

    active_pub_ = node_->create_publisher<moveit2_extended_msgs::msg::ToolInfo>(
        "~/active_tool", rclcpp::QoS(1).transient_local().reliable());

    const std::string initial = node_->get_parameter("initial_tool").as_string();
    if (!initial.empty())
    {
      // Deferred: /apply_planning_scene will not be there yet if move_group is starting alongside
      // us, and failing at construction would take the node down for a race.
      startup_timer_ = node_->create_wall_timer(std::chrono::seconds(2), [this, initial]() {
        startup_timer_->cancel();
        std::string message;
        if (!fitTool(initial, message))
        {
          RCLCPP_ERROR(node_->get_logger(), "cannot fit the initial tool '%s': %s", initial.c_str(),
                       message.c_str());
        }
      });
    }
    publishActive();
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const
  {
    return node_->get_node_base_interface();
  }

private:
  /** Detach whatever is fitted, then attach `name` (empty just detaches). */
  bool fitTool(const std::string& name, std::string& message)
  {
    if (registry_.mount_link.empty())
    {
      message = "no tool registry loaded";
      return false;
    }
    if (!scene_client_->wait_for_service(std::chrono::seconds(5)))
    {
      message = "no /apply_planning_scene; is move_group running?";
      return false;
    }

    const Tool* tool = nullptr;
    if (!name.empty())
    {
      tool = registry_.find(name);
      if (!tool)
      {
        message = "no tool named '" + name + "'";
        return false;
      }
    }

    auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    request->scene.is_diff = true;
    request->scene.robot_state.is_diff = true;

    // Remove the previous tool first, in the same diff. Attaching a second tool without detaching
    // the first would leave the planner avoiding a gripper that is no longer on the robot.
    if (!active_.empty())
    {
      moveit_msgs::msg::AttachedCollisionObject remove;
      remove.link_name = registry_.mount_link;
      remove.object.id = objectId(active_);
      remove.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      request->scene.robot_state.attached_collision_objects.push_back(remove);
    }

    // ...and then actually delete it, which is a SEPARATE operation.
    //
    // AttachedCollisionObject::REMOVE does not mean "delete": MoveIt DETACHES the object and puts
    // it straight back into the world at the pose it had -- which is on the flange. Detaching
    // alone therefore leaves a gripper-shaped obstacle exactly where the next gripper is about to
    // go, and every subsequent plan fails on a collision with a tool that is not on the robot.
    // Found by tool_carrier_benchmark.py: the cutter planned 7/7 and every gripper 0/7, with
    // move_group reporting contacts against `tool__robotiq_2f140` of type 'Object' -- a world
    // object, not an attached one.
    //
    // Ordering within the one diff is safe: PlanningScene::setPlanningSceneDiffMsg applies
    // robot_state (the detach, planning_scene.cpp:1340) before world.collision_objects (the
    // delete, :1358), so by the time this is processed the object is in the world to be deleted.
    //
    // ONLY the tool that was just detached. A tempting "sweep every id the registry knows" is
    // wrong: processCollisionObjectMsg returns false for an id that is not in the world, and
    // setPlanningSceneDiffMsg ANDs those results together, so one absent id makes
    // ApplyPlanningScene reject the entire change -- including the attach. Tried it; every switch
    // then failed with "the planning scene rejected the change".
    //
    // And not at all when the tool being fitted is the one already fitted. Both the detach and the
    // attach live in robot_state, so they are applied in order and the ADD takes the object back
    // out of the world before this is reached -- leaving nothing to delete, which is once again a
    // false result that rejects the whole diff. Re-fitting the current tool is a legitimate thing
    // to ask for (it repairs a scene someone edited by hand), so it has to work.
    const bool refitting_same_tool = tool && tool->info.name == active_;
    if (!active_.empty() && !refitting_same_tool)
    {
      moveit_msgs::msg::CollisionObject gone;
      gone.id = objectId(active_);
      gone.header.frame_id = registry_.mount_link;
      gone.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      request->scene.world.collision_objects.push_back(gone);
    }

    if (tool)
    {
      moveit_msgs::msg::AttachedCollisionObject attached;
      attached.link_name = registry_.mount_link;
      attached.touch_links = tool->info.allowed_collision_links;
      attached.object.id = objectId(tool->info.name);
      attached.object.header.frame_id = registry_.mount_link;
      attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
      for (const auto& shape : tool->shapes)
      {
        attached.object.primitives.push_back(shape.primitive);
        attached.object.primitive_poses.push_back(shape.pose);
      }
      request->scene.robot_state.attached_collision_objects.push_back(attached);
    }

    auto pending = scene_client_->async_send_request(request);
    if (pending.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
      scene_client_->remove_pending_request(pending.request_id);
      message = "the planning scene did not answer";
      return false;
    }
    if (!pending.future.get()->success)
    {
      message = "the planning scene rejected the change";
      return false;
    }

    active_ = tool ? tool->info.name : "";
    publishActive();

    if (tool && node_->get_parameter("switch_carrier_with_tool").as_bool() && !tool->info.carrier_config.empty())
    {
      // Said rather than done here: this node has no business reaching into another process's
      // collision detector, and the Objective that switches tools should switch the carrier
      // explicitly so the change is visible in the tree.
      RCLCPP_INFO(node_->get_logger(),
                  "tool '%s' expects carrier configuration '%s' -- use SwitchCarrierType to apply it",
                  tool->info.name.c_str(), tool->info.carrier_config.c_str());
    }

    message = tool ? ("fitted '" + tool->info.name + "'") : "detached";
    RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
    return true;
  }

  static std::string objectId(const std::string& tool_name)
  {
    // Namespaced so a tool cannot collide with an object the Objective put in the scene itself.
    return "tool__" + tool_name;
  }

  void publishActive()
  {
    moveit2_extended_msgs::msg::ToolInfo message;
    if (const Tool* tool = active_.empty() ? nullptr : registry_.find(active_))
    {
      message = tool->info;
      message.attached = true;
    }
    active_pub_->publish(message);
  }

  void onSwitch(const std::shared_ptr<srvs::SwitchTool::Request> request,
                std::shared_ptr<srvs::SwitchTool::Response> response)
  {
    const std::string wanted = request->detach_only ? "" : request->name;
    std::string message;
    response->success = fitTool(wanted, message);
    response->message = message;
    if (response->success && !active_.empty())
    {
      if (const Tool* tool = registry_.find(active_))
      {
        response->active_tool = tool->info;
        response->active_tool.attached = true;
      }
    }
  }

  void onList(const std::shared_ptr<srvs::ListTools::Request>, std::shared_ptr<srvs::ListTools::Response> response)
  {
    response->tools = registry_.list();
    for (auto& tool : response->tools)
    {
      tool.attached = (tool.name == active_);
    }
    response->success = true;
  }

  void onActive(const std::shared_ptr<srvs::GetActiveTool::Request>,
                std::shared_ptr<srvs::GetActiveTool::Response> response)
  {
    response->success = true;
    response->has_tool = !active_.empty();
    if (const Tool* tool = active_.empty() ? nullptr : registry_.find(active_))
    {
      response->tool = tool->info;
      response->tool.attached = true;
    }
  }

  rclcpp::Node::SharedPtr node_;
  ToolRegistry registry_;
  std::string active_;

  rclcpp::CallbackGroup::SharedPtr scene_callback_group_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr scene_client_;
  rclcpp::Service<srvs::SwitchTool>::SharedPtr switch_srv_;
  rclcpp::Service<srvs::ListTools>::SharedPtr list_srv_;
  rclcpp::Service<srvs::GetActiveTool>::SharedPtr active_srv_;
  rclcpp::Publisher<moveit2_extended_msgs::msg::ToolInfo>::SharedPtr active_pub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

}  // namespace moveit2_extended::tools

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto manager = std::make_shared<moveit2_extended::tools::ToolManager>(rclcpp::NodeOptions{});
  executor.add_node(manager->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
