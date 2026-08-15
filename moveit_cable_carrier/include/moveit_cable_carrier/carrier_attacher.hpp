// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit_cable_carrier/carrier_params.hpp>
#include <moveit_cable_carrier/rod_solver.hpp>

#include <geometric_shapes/shapes.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace moveit_cable_carrier
{

/** Turns a solved centreline into a capsule chain built from primitives that MoveIt/FCL
 *  understand (geometric_shapes has no capsule, so a cylinder per edge plus a sphere per
 *  interior node is used -- geometrically identical to a capsule chain).
 *
 *  `poses` are in the same frame as `shape.nodes`. */
void buildCapsuleChain(const CarrierShape& shape, std::vector<shapes::ShapeConstPtr>& shapes_out,
                       EigenSTL::vector_Isometry3d& poses_out);

/** Binds one CarrierParams to a RobotModel and evaluates the carrier for a given RobotState.
 *
 *  This is the piece that gives MoveIt something it does not otherwise have: collision geometry
 *  whose *shape*, not just pose, is a function of the joint configuration. The carrier is handed
 *  to MoveIt as an ordinary AttachedBody, so the ACM, link padding, distance queries and contact
 *  reporting all keep working unchanged. Its touch_links carry the links the carrier is allowed
 *  to rest against, so no ACM editing is required either. */
class CarrierAttacher
{
public:
  CarrierAttacher(const CarrierParams& params, const moveit::core::RobotModelConstPtr& model);

  /** True if both mount links exist in the robot model. When false the attacher is inert. */
  bool valid() const { return valid_; }
  const CarrierParams& params() const { return params_; }
  const std::string& id() const { return id_; }

  /** Solve the carrier shape for `state`, in the planning (model) frame.
   *  `state` must already have up-to-date link transforms. */
  CarrierShape computeShape(const moveit::core::RobotState& state) const;

  /** Capsule proxies for the robot's links at `state`, in the model frame.
   *
   *  Built from the link collision geometry, which on this arm is already cylinders and a box, so
   *  the proxy is close to exact rather than a crude bound. Meshes fall back to their bounding
   *  sphere, which is conservative -- the carrier keeps further away than it strictly must. */
  std::vector<Obstacle> buildObstacles(const moveit::core::RobotState& state) const;

  /** Solve and attach to `state`. Returns false when the shape is infeasible -- callers should
   *  treat that as "this configuration is not usable", because an infeasible shape means the
   *  carrier would have to stretch or over-bend, which the hardware cannot do. */
  bool attach(moveit::core::RobotState& state) const;

  /** Attach a shape that was already solved (used when a caller reuses one solve for several
   *  queries). */
  bool attachShape(moveit::core::RobotState& state, const CarrierShape& shape) const;

private:
  CarrierParams params_;
  moveit::core::RobotModelConstPtr model_;
  /** Mutable for the same reason as last_shape_: collision checking is logically const, but the
   *  solver carries per-query state -- the obstacle set is rebuilt for every robot pose. */
  mutable RodSolver solver_;
  std::string id_;
  bool valid_ = false;
  /** Warm start for the next solve. Mutable because collision checking is logically const.
   *  Not thread safe by itself -- CollisionEnvCarrier keeps one attacher per thread. */
  mutable CarrierShape last_shape_;
  /** Links whose geometry is skipped: the two the brackets sit on, because the run legitimately
   *  starts and ends against them and pushing off them would fight the anchors. */
  std::set<std::string> skip_links_;
};

using CarrierAttacherPtr = std::shared_ptr<CarrierAttacher>;

}  // namespace moveit_cable_carrier
