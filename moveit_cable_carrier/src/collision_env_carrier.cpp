// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit_cable_carrier/collision_env_carrier.hpp>
#include <moveit_cable_carrier/carrier_registry.hpp>

#include <rclcpp/logging.hpp>

namespace moveit_cable_carrier
{
namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("moveit_cable_carrier");

/** Per-thread attacher cache. Each thread owns its own attachers so the rod solver's warm start
 *  is never shared, which keeps collision checking thread safe without locking. */
struct AttacherCache
{
  const void* owner = nullptr;
  std::vector<CarrierAttacherPtr> attachers;
};
}  // namespace

const std::string CollisionDetectorAllocatorCarrier::NAME("CABLE_CARRIER");

CollisionEnvCarrier::CollisionEnvCarrier(const moveit::core::RobotModelConstPtr& model, double padding, double scale)
  : collision_detection::CollisionEnvFCL(model, padding, scale)
  , carrier_params_(CarrierRegistry::instance().carriers())
{
  RCLCPP_INFO(kLogger, "CollisionEnvCarrier: %zu cable carrier(s) configured", carrier_params_.size());
}

CollisionEnvCarrier::CollisionEnvCarrier(const moveit::core::RobotModelConstPtr& model,
                                         const collision_detection::WorldPtr& world, double padding, double scale)
  : collision_detection::CollisionEnvFCL(model, world, padding, scale)
  , carrier_params_(CarrierRegistry::instance().carriers())
{
  RCLCPP_INFO(kLogger, "CollisionEnvCarrier: %zu cable carrier(s) configured", carrier_params_.size());
}

CollisionEnvCarrier::CollisionEnvCarrier(const CollisionEnvCarrier& other,
                                         const collision_detection::WorldPtr& world)
  : collision_detection::CollisionEnvFCL(other, world), carrier_params_(other.carrier_params_)
{
  // Copy-constructed for a diff planning scene: keep the *same* carrier set as the parent rather
  // than re-reading the registry, so a mid-plan reconfiguration cannot change geometry underneath
  // a running planner.
}

const std::vector<CarrierAttacherPtr>& CollisionEnvCarrier::attachers() const
{
  static thread_local AttacherCache cache;
  if (cache.owner != static_cast<const void*>(this))
  {
    cache.attachers.clear();
    cache.attachers.reserve(carrier_params_.size());
    for (const auto& p : carrier_params_)
    {
      auto attacher = std::make_shared<CarrierAttacher>(p, getRobotModel());
      if (!attacher->valid())
      {
        RCLCPP_WARN(kLogger, "carrier '%s' references unknown links ('%s' -> '%s'); it will be ignored",
                    p.name.c_str(), p.base_link.c_str(), p.tip_link.c_str());
        continue;
      }
      cache.attachers.push_back(std::move(attacher));
    }
    cache.owner = static_cast<const void*>(this);
  }
  return cache.attachers;
}

moveit::core::RobotState CollisionEnvCarrier::augment(const moveit::core::RobotState& state, bool& infeasible) const
{
  infeasible = false;
  moveit::core::RobotState augmented(state);
  augmented.updateLinkTransforms();
  for (const auto& attacher : attachers())
  {
    const CarrierShape shape = attacher->computeShape(augmented);
    if (!shape.feasible)
    {
      // An infeasible shape means the brackets are further apart than the carrier is long, or the
      // required bend is tighter than R_min. The hardware cannot do this, so the configuration is
      // rejected rather than approximated.
      infeasible = true;
      continue;
    }
    attacher->attachShape(augmented, shape);
  }
  return augmented;
}

void CollisionEnvCarrier::reportInfeasible(const collision_detection::CollisionRequest& req,
                                           collision_detection::CollisionResult& res) const
{
  res.collision = true;
  res.distance = 0.0;
  if (req.contacts && res.contact_count < req.max_contacts)
  {
    collision_detection::Contact contact;
    contact.pos = Eigen::Vector3d::Zero();
    contact.normal = Eigen::Vector3d::UnitZ();
    contact.depth = 0.0;
    contact.body_name_1 = "cable_carrier";
    contact.body_type_1 = collision_detection::BodyTypes::ROBOT_ATTACHED;
    contact.body_name_2 = "cable_carrier_limit";
    contact.body_type_2 = collision_detection::BodyTypes::ROBOT_ATTACHED;
    res.contacts[std::make_pair(contact.body_name_1, contact.body_name_2)].push_back(contact);
    ++res.contact_count;
  }
}

void CollisionEnvCarrier::checkSelfCollision(const collision_detection::CollisionRequest& req,
                                             collision_detection::CollisionResult& res,
                                             const moveit::core::RobotState& state) const
{
  bool infeasible = false;
  const moveit::core::RobotState augmented = augment(state, infeasible);
  if (infeasible)
  {
    reportInfeasible(req, res);
    return;
  }
  collision_detection::CollisionEnvFCL::checkSelfCollision(req, res, augmented);
}

void CollisionEnvCarrier::checkSelfCollision(const collision_detection::CollisionRequest& req,
                                             collision_detection::CollisionResult& res,
                                             const moveit::core::RobotState& state,
                                             const collision_detection::AllowedCollisionMatrix& acm) const
{
  bool infeasible = false;
  const moveit::core::RobotState augmented = augment(state, infeasible);
  if (infeasible)
  {
    reportInfeasible(req, res);
    return;
  }
  collision_detection::CollisionEnvFCL::checkSelfCollision(req, res, augmented, acm);
}

void CollisionEnvCarrier::checkRobotCollision(const collision_detection::CollisionRequest& req,
                                              collision_detection::CollisionResult& res,
                                              const moveit::core::RobotState& state) const
{
  bool infeasible = false;
  const moveit::core::RobotState augmented = augment(state, infeasible);
  if (infeasible)
  {
    reportInfeasible(req, res);
    return;
  }
  collision_detection::CollisionEnvFCL::checkRobotCollision(req, res, augmented);
}

void CollisionEnvCarrier::checkRobotCollision(const collision_detection::CollisionRequest& req,
                                              collision_detection::CollisionResult& res,
                                              const moveit::core::RobotState& state,
                                              const collision_detection::AllowedCollisionMatrix& acm) const
{
  bool infeasible = false;
  const moveit::core::RobotState augmented = augment(state, infeasible);
  if (infeasible)
  {
    reportInfeasible(req, res);
    return;
  }
  collision_detection::CollisionEnvFCL::checkRobotCollision(req, res, augmented, acm);
}

void CollisionEnvCarrier::checkRobotCollision(const collision_detection::CollisionRequest& req,
                                              collision_detection::CollisionResult& res,
                                              const moveit::core::RobotState& state1,
                                              const moveit::core::RobotState& state2,
                                              const collision_detection::AllowedCollisionMatrix& acm) const
{
  bool infeasible1 = false, infeasible2 = false;
  const moveit::core::RobotState a1 = augment(state1, infeasible1);
  const moveit::core::RobotState a2 = augment(state2, infeasible2);
  if (infeasible1 || infeasible2)
  {
    reportInfeasible(req, res);
    return;
  }
  collision_detection::CollisionEnvFCL::checkRobotCollision(req, res, a1, a2, acm);
}

void CollisionEnvCarrier::checkRobotCollision(const collision_detection::CollisionRequest& req,
                                              collision_detection::CollisionResult& res,
                                              const moveit::core::RobotState& state1,
                                              const moveit::core::RobotState& state2) const
{
  bool infeasible1 = false, infeasible2 = false;
  const moveit::core::RobotState a1 = augment(state1, infeasible1);
  const moveit::core::RobotState a2 = augment(state2, infeasible2);
  if (infeasible1 || infeasible2)
  {
    reportInfeasible(req, res);
    return;
  }
  collision_detection::CollisionEnvFCL::checkRobotCollision(req, res, a1, a2);
}

void CollisionEnvCarrier::distanceSelf(const collision_detection::DistanceRequest& req,
                                       collision_detection::DistanceResult& res,
                                       const moveit::core::RobotState& state) const
{
  bool infeasible = false;
  const moveit::core::RobotState augmented = augment(state, infeasible);
  if (infeasible)
  {
    res.collision = true;
    res.minimum_distance.distance = 0.0;
    return;
  }
  collision_detection::CollisionEnvFCL::distanceSelf(req, res, augmented);
}

void CollisionEnvCarrier::distanceRobot(const collision_detection::DistanceRequest& req,
                                        collision_detection::DistanceResult& res,
                                        const moveit::core::RobotState& state) const
{
  bool infeasible = false;
  const moveit::core::RobotState augmented = augment(state, infeasible);
  if (infeasible)
  {
    res.collision = true;
    res.minimum_distance.distance = 0.0;
    return;
  }
  collision_detection::CollisionEnvFCL::distanceRobot(req, res, augmented);
}

}  // namespace moveit_cable_carrier
