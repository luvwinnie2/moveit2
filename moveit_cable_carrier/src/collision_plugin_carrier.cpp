// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit_cable_carrier/collision_env_carrier.hpp>

#include <moveit/collision_detection/collision_plugin.h>
#include <pluginlib/class_list_macros.hpp>

namespace moveit_cable_carrier
{

/** Entry point used by MoveIt's collision plugin loader. Selected by setting
 *    collision_detector: CABLE_CARRIER
 *  in the move_group configuration (or by calling
 *  PlanningScene::allocateCollisionDetector(CollisionDetectorAllocatorCarrier::create()) directly). */
class CarrierCollisionPlugin : public collision_detection::CollisionPlugin
{
public:
  bool initialize(const planning_scene::PlanningScenePtr& scene) const override
  {
    scene->allocateCollisionDetector(CollisionDetectorAllocatorCarrier::create());
    return true;
  }
};

}  // namespace moveit_cable_carrier

PLUGINLIB_EXPORT_CLASS(moveit_cable_carrier::CarrierCollisionPlugin, collision_detection::CollisionPlugin)
