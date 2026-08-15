// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit_cable_carrier/carrier_attacher.hpp>

#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit/collision_detection_fcl/collision_env_fcl.h>

#include <string>
#include <vector>

namespace moveit_cable_carrier
{

/** FCL-backed collision environment that additionally carries a set of deformable cable
 *  carriers whose geometry is recomputed for every queried robot state.
 *
 *  Rationale for wrapping rather than replacing FCL: everything MoveIt already does well --
 *  broadphase, the ACM, link padding/scaling, attached objects, distance and contact reporting --
 *  stays intact. The only thing added is that, before delegating, the queried state is copied and
 *  the carrier's solved shape is attached to it as an ordinary AttachedBody.
 *
 *  Why a collision detector and not a StateFeasibilityFn: MoveIt's PlanningScene does not
 *  propagate the feasibility predicate to diff scenes (isStateFeasible does not consult the
 *  parent, and the diff constructor does not copy it), and move_group plans on a diff of the
 *  monitored scene. A predicate installed on the monitored scene would therefore be silently
 *  ignored. allocateCollisionDetector *does* carry the detector across diffs. */
class CollisionEnvCarrier : public collision_detection::CollisionEnvFCL
{
public:
  CollisionEnvCarrier() = delete;

  explicit CollisionEnvCarrier(const moveit::core::RobotModelConstPtr& model, double padding = 0.0,
                               double scale = 1.0);
  CollisionEnvCarrier(const moveit::core::RobotModelConstPtr& model, const collision_detection::WorldPtr& world,
                      double padding = 0.0, double scale = 1.0);
  CollisionEnvCarrier(const CollisionEnvCarrier& other, const collision_detection::WorldPtr& world);

  ~CollisionEnvCarrier() override = default;

  void checkSelfCollision(const collision_detection::CollisionRequest& req,
                          collision_detection::CollisionResult& res,
                          const moveit::core::RobotState& state) const override;

  void checkSelfCollision(const collision_detection::CollisionRequest& req,
                          collision_detection::CollisionResult& res, const moveit::core::RobotState& state,
                          const collision_detection::AllowedCollisionMatrix& acm) const override;

  void checkRobotCollision(const collision_detection::CollisionRequest& req,
                           collision_detection::CollisionResult& res,
                           const moveit::core::RobotState& state) const override;

  void checkRobotCollision(const collision_detection::CollisionRequest& req,
                           collision_detection::CollisionResult& res, const moveit::core::RobotState& state,
                           const collision_detection::AllowedCollisionMatrix& acm) const override;

  void checkRobotCollision(const collision_detection::CollisionRequest& req,
                           collision_detection::CollisionResult& res, const moveit::core::RobotState& state1,
                           const moveit::core::RobotState& state2,
                           const collision_detection::AllowedCollisionMatrix& acm) const override;

  void checkRobotCollision(const collision_detection::CollisionRequest& req,
                           collision_detection::CollisionResult& res, const moveit::core::RobotState& state1,
                           const moveit::core::RobotState& state2) const override;

  void distanceSelf(const collision_detection::DistanceRequest& req, collision_detection::DistanceResult& res,
                    const moveit::core::RobotState& state) const override;

  void distanceRobot(const collision_detection::DistanceRequest& req, collision_detection::DistanceResult& res,
                     const moveit::core::RobotState& state) const override;

  /** Carriers active in this environment (snapshot taken from CarrierRegistry at construction). */
  const std::vector<CarrierParams>& carriers() const { return carrier_params_; }

private:
  /** Copy `state`, solve every carrier for it and attach the results.
   *  Sets `infeasible` when any carrier cannot physically span its two brackets. */
  moveit::core::RobotState augment(const moveit::core::RobotState& state, bool& infeasible) const;

  /** Per-thread attachers, so the solver warm start is not shared across threads. */
  const std::vector<CarrierAttacherPtr>& attachers() const;

  /** Fill `res` with a synthetic collision describing an unreachable carrier configuration. */
  void reportInfeasible(const collision_detection::CollisionRequest& req,
                        collision_detection::CollisionResult& res) const;

  std::vector<CarrierParams> carrier_params_;
};

/** Allocator so the environment can be selected through MoveIt's normal machinery. */
class CollisionDetectorAllocatorCarrier
  : public collision_detection::CollisionDetectorAllocatorTemplate<CollisionEnvCarrier,
                                                                   CollisionDetectorAllocatorCarrier>
{
public:
  static const std::string NAME;
};

}  // namespace moveit_cable_carrier
