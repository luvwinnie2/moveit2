// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_behaviors/scene_behaviors.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <algorithm>

namespace moveit2_extended::behaviors
{
namespace
{
/** Build the SolidPrimitive for a named shape, checking the dimension count. Getting this wrong
 *  silently produces a zero-sized object that collides with nothing. */
BtExpected<shape_msgs::msg::SolidPrimitive> makePrimitive(const std::string& shape,
                                                          const std::vector<double>& dimensions)
{
  shape_msgs::msg::SolidPrimitive primitive;
  size_t expected = 0;
  if (shape == "box")
  {
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    expected = 3;  // x, y, z
  }
  else if (shape == "sphere")
  {
    primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    expected = 1;  // radius
  }
  else if (shape == "cylinder")
  {
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    expected = 2;  // height, radius
  }
  else if (shape == "cone")
  {
    primitive.type = shape_msgs::msg::SolidPrimitive::CONE;
    expected = 2;  // height, radius
  }
  else
  {
    return nonstd::make_unexpected("unknown shape '" + shape + "'; use box, sphere, cylinder or cone");
  }

  if (dimensions.size() != expected)
  {
    return nonstd::make_unexpected("a " + shape + " needs " + std::to_string(expected) + " dimension(s), got " +
                                   std::to_string(dimensions.size()));
  }
  for (double dimension : dimensions)
  {
    if (!(dimension > 0.0))
    {
      return nonstd::make_unexpected("every dimension must be greater than zero");
    }
  }
  // SolidPrimitive::dimensions is a BoundedVector, not a std::vector, so it cannot be assigned
  // from one directly.
  primitive.dimensions.resize(dimensions.size());
  std::copy(dimensions.begin(), dimensions.end(), primitive.dimensions.begin());
  return primitive;
}
}  // namespace

// ---------------------------------------------------------------------------------------------
// GetCurrentPlanningScene
// ---------------------------------------------------------------------------------------------

GetCurrentPlanningScene::GetCurrentPlanningScene(const std::string& name, const NodeConfig& config,
                                                 BehaviorContextPtr shared_resources)
  : ServiceClientBehaviorBase<moveit_msgs::srv::GetPlanningScene>(name, config, std::move(shared_resources),
                                                                   "/get_planning_scene")
{
}

BT::PortsList GetCurrentPlanningScene::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<int>("components", 1023, "PlanningSceneComponents bitmask; the default asks for everything"),
      BT::OutputPort<moveit_msgs::msg::PlanningScene>("planning_scene", "the scene as it is now"),
      BT::OutputPort<std::vector<std::string>>("object_ids", "collision object names in the scene"),
  });
}

BtExpected<std::shared_ptr<GetCurrentPlanningScene::Request>> GetCurrentPlanningScene::createRequest()
{
  auto request = std::make_shared<Request>();
  request->components.components = static_cast<uint32_t>(getInputOr<int>("components", 1023));
  return request;
}

BtStatus GetCurrentPlanningScene::processResponse(const std::shared_ptr<Response>& response)
{
  setOutput("planning_scene", response->scene);

  std::vector<std::string> ids;
  for (const auto& object : response->scene.world.collision_objects)
  {
    ids.push_back(object.id);
  }
  std::sort(ids.begin(), ids.end());
  setOutput("object_ids", ids);
  return BtStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------
// AddVirtualObjectToPlanningScene
// ---------------------------------------------------------------------------------------------

AddVirtualObjectToPlanningScene::AddVirtualObjectToPlanningScene(const std::string& name, const NodeConfig& config,
                                                                 BehaviorContextPtr shared_resources)
  : ServiceClientBehaviorBase<moveit_msgs::srv::ApplyPlanningScene>(name, config, std::move(shared_resources),
                                                                     "/apply_planning_scene")
{
}

BT::PortsList AddVirtualObjectToPlanningScene::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<std::string>("object_id", "a name to refer to it by later"),
      BT::InputPort<std::string>("shape", "box", "box | sphere | cylinder | cone | mesh"),
      BT::InputPort<std::vector<double>>("dimensions", "box x;y;z, sphere r, cylinder h;r, cone h;r"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "where to put it"),
      BT::InputPort<std::string>("mesh_resource", "", "package:// URI, for shape=mesh"),
      BT::InputPort<double>("mesh_scale", 1.0, ""),
      BT::InputPort<std::string>("attach_to_link", "",
                                 "attach to this link instead of leaving it in the world"),
      BT::InputPort<std::vector<std::string>>("touch_links", "links this object may touch when attached"),
      BT::OutputPort<bool>("success", ""),
  });
}

BtExpected<std::shared_ptr<AddVirtualObjectToPlanningScene::Request>>
AddVirtualObjectToPlanningScene::createRequest()
{
  const auto object_id = getInputOr<std::string>("object_id", std::string(""));
  if (object_id.empty())
  {
    return nonstd::make_unexpected("object_id must not be empty");
  }

  const auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose");
  if (!pose)
  {
    return nonstd::make_unexpected("pose: " + pose.error());
  }

  moveit_msgs::msg::CollisionObject object;
  object.id = object_id;
  object.header = pose->header;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  const auto shape = getInputOr<std::string>("shape", std::string("box"));
  if (shape == "mesh")
  {
    // Not implemented rather than half-implemented. ApplyPlanningScene carries mesh VERTICES, not
    // a URI, so this would have to load and triangulate the resource here -- and a mesh silently
    // added as an empty shape collides with nothing, which is the worst possible failure for a
    // collision object. Say so instead.
    return nonstd::make_unexpected("shape=mesh is not implemented: ApplyPlanningScene needs the mesh vertices, "
                                   "not a URI. Use a primitive, or publish the mesh through the planning "
                                   "scene monitor.");
  }

  const auto dimensions = getInputOr<std::vector<double>>("dimensions", std::vector<double>{});
  const auto primitive = makePrimitive(shape, dimensions);
  if (!primitive)
  {
    return nonstd::make_unexpected(primitive.error());
  }
  object.primitives = { *primitive };
  object.primitive_poses = { pose->pose };

  auto request = std::make_shared<Request>();
  request->scene.is_diff = true;
  request->scene.robot_state.is_diff = true;

  const auto attach_to = getInputOr<std::string>("attach_to_link", std::string(""));
  if (attach_to.empty())
  {
    request->scene.world.collision_objects = { object };
  }
  else
  {
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = attach_to;
    attached.object = object;
    attached.object.header.frame_id = pose->header.frame_id;
    attached.touch_links = getInputOr<std::vector<std::string>>("touch_links", std::vector<std::string>{});
    request->scene.robot_state.attached_collision_objects = { attached };
  }
  return request;
}

BtStatus AddVirtualObjectToPlanningScene::processResponse(const std::shared_ptr<Response>& response)
{
  setOutput("success", response->success);
  if (!response->success)
  {
    RCLCPP_ERROR(getLogger(), "the planning scene rejected the object");
    return BtStatus::FAILURE;
  }
  return BtStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------
// ModifyObjectInPlanningScene
// ---------------------------------------------------------------------------------------------

ModifyObjectInPlanningScene::ModifyObjectInPlanningScene(const std::string& name, const NodeConfig& config,
                                                         BehaviorContextPtr shared_resources)
  : ServiceClientBehaviorBase<moveit_msgs::srv::ApplyPlanningScene>(name, config, std::move(shared_resources),
                                                                     "/apply_planning_scene")
{
}

BT::PortsList ModifyObjectInPlanningScene::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<std::string>("object_id", "which object"),
      BT::InputPort<std::string>("operation", "move", "move | remove | attach | detach"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "the new pose, for operation=move"),
      BT::InputPort<std::string>("attach_to_link", "", "the link, for operation=attach"),
      BT::InputPort<std::vector<std::string>>("touch_links", "links it may touch while attached"),
      BT::OutputPort<bool>("success", ""),
  });
}

BtExpected<std::shared_ptr<ModifyObjectInPlanningScene::Request>> ModifyObjectInPlanningScene::createRequest()
{
  const auto object_id = getInputOr<std::string>("object_id", std::string(""));
  if (object_id.empty())
  {
    return nonstd::make_unexpected("object_id must not be empty");
  }
  const auto operation = getInputOr<std::string>("operation", std::string("move"));

  auto request = std::make_shared<Request>();
  request->scene.is_diff = true;
  request->scene.robot_state.is_diff = true;

  if (operation == "remove")
  {
    moveit_msgs::msg::CollisionObject object;
    object.id = object_id;
    object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    request->scene.world.collision_objects = { object };
    return request;
  }

  if (operation == "move")
  {
    const auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose");
    if (!pose)
    {
      return nonstd::make_unexpected("operation=move needs a pose: " + pose.error());
    }
    moveit_msgs::msg::CollisionObject object;
    object.id = object_id;
    object.header = pose->header;
    object.operation = moveit_msgs::msg::CollisionObject::MOVE;
    object.primitive_poses = { pose->pose };
    request->scene.world.collision_objects = { object };
    return request;
  }

  if (operation == "attach")
  {
    const auto link = getInputOr<std::string>("attach_to_link", std::string(""));
    if (link.empty())
    {
      return nonstd::make_unexpected("operation=attach needs attach_to_link");
    }
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = link;
    attached.object.id = object_id;
    attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    attached.touch_links = getInputOr<std::vector<std::string>>("touch_links", std::vector<std::string>{});
    request->scene.robot_state.attached_collision_objects = { attached };
    return request;
  }

  if (operation == "detach")
  {
    // MoveIt's idiom: REMOVE on the attached object puts it back into the world rather than
    // deleting it. Deleting is what ClearSceneObjects is for.
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.object.id = object_id;
    attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    request->scene.robot_state.attached_collision_objects = { attached };
    return request;
  }

  return nonstd::make_unexpected("unknown operation '" + operation + "'; use move, remove, attach or detach");
}

BtStatus ModifyObjectInPlanningScene::processResponse(const std::shared_ptr<Response>& response)
{
  setOutput("success", response->success);
  if (!response->success)
  {
    RCLCPP_ERROR(getLogger(), "the planning scene rejected the change");
    return BtStatus::FAILURE;
  }
  return BtStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------
// ClearSceneObjects
// ---------------------------------------------------------------------------------------------

ClearSceneObjects::ClearSceneObjects(const std::string& name, const NodeConfig& config,
                                     BehaviorContextPtr shared_resources)
  : ServiceClientBehaviorBase<moveit_msgs::srv::ApplyPlanningScene>(name, config, std::move(shared_resources),
                                                                     "/apply_planning_scene")
{
}

BT::PortsList ClearSceneObjects::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<std::vector<std::string>>("object_ids", "which to remove; required"),
      BT::OutputPort<bool>("success", ""),
  });
}

BtExpected<std::shared_ptr<ClearSceneObjects::Request>> ClearSceneObjects::createRequest()
{
  const auto ids = getInputOr<std::vector<std::string>>("object_ids", std::vector<std::string>{});
  if (ids.empty())
  {
    // Deliberately not "empty means everything". Sending a non-diff scene to wipe the world also
    // wipes the Allowed Collision Matrix, and a tree that clears more than it meant to is a very
    // quiet way to make the robot collide with itself.
    return nonstd::make_unexpected("object_ids must name what to remove; there is no 'remove everything' mode, "
                                   "because that would also discard the allowed-collision matrix");
  }

  auto request = std::make_shared<Request>();
  request->scene.is_diff = true;
  request->scene.robot_state.is_diff = true;
  for (const auto& id : ids)
  {
    moveit_msgs::msg::CollisionObject object;
    object.id = id;
    object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    request->scene.world.collision_objects.push_back(object);
  }
  return request;
}

BtStatus ClearSceneObjects::processResponse(const std::shared_ptr<Response>& response)
{
  setOutput("success", response->success);
  return response->success ? BtStatus::SUCCESS : BtStatus::FAILURE;
}

}  // namespace moveit2_extended::behaviors
