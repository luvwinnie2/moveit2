// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit_cable_carrier/carrier_params.hpp>

#include <Eigen/Geometry>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <vector>

namespace moveit_cable_carrier
{

/** Result of one quasi-static carrier shape solve: a polyline centreline in the planning frame. */
struct CarrierShape
{
  EigenSTL::vector_Vector3d nodes;   ///< num_segments+1 points
  double radius = 0.0;               ///< capsule radius that covers the cross-section
  bool feasible = false;             ///< false => the two brackets cannot be joined by this carrier
  double endpoint_error = 0.0;       ///< residual position error at the moving bracket [m]
  double max_curvature = 0.0;        ///< achieved max |kappa| [1/m]
  int iterations = 0;

  // ---- internal load, for lifetime screening -------------------------------
  /** Per-joint curvature [1/m]. Lets a caller see *where* along the run the carrier is worked
   *  hardest, not just how hard. */
  std::vector<double> curvature;
  /** Per-joint twist rate about the carrier axis [rad/m]. A 3D carrier has a torsion stop, so a
   *  large value here means the mounting is forcing torsion the hardware is meant to refuse. */
  std::vector<double> twist_rate;
  double max_twist_rate = 0.0;

  /** max_curvature * bend_radius. 1.0 = running exactly at the hardware's minimum radius,
   *  >1.0 = over-bent. This is the single number to watch. */
  double bend_utilisation = 0.0;
  /** Tightest radius actually reached [m] (= 1/max_curvature). */
  double min_bend_radius() const { return max_curvature > 1e-9 ? 1.0 / max_curvature : 0.0; }

  /** Peak bending stress from Euler-Bernoulli: sigma = E * y * kappa, with y the outer radius. */
  double max_bending_stress = 0.0;   ///< Pa
  /** Peak torsional shear stress: tau = G * r * dtheta/ds. */
  double max_torsional_stress = 0.0; ///< Pa

  bool empty() const { return nodes.size() < 2; }

  /** Decimate the polyline while staying conservative: any node removed contributes its
   *  perpendicular deviation to the capsule radius, so the simplified chain still encloses
   *  the original one. Returns the simplified copy. */
  CarrierShape simplify(double max_deviation) const;
};

/** Quasi-static shape solver for an inextensible, bend-radius-limited carrier.
 *
 * The carrier is modelled as a discrete inextensible rod with a hard curvature bound. Given the
 * world poses of the two brackets, the shape is the minimiser of bending energy subject to
 *   - fixed arc length L (inextensible),
 *   - |kappa| <= 1/R_min (and kappa >= 0 when the chain is back-stopped),
 *   - clamped endpoints: position *and* exit tangent at both brackets,
 *   - optional planarity and a gravity sag term.
 *
 * It is solved by Gauss-Seidel constraint projection (position based dynamics), warm-started from
 * a cubic Hermite guess or from a previous solution. This converges in tens of microseconds, which
 * is what makes per-state evaluation inside a motion planner affordable -- a full Cosserat rod FEM
 * or a GPU physics step would be several orders of magnitude too slow in that loop.
 *
 * The *accuracy* of this cheap model is meant to be established offline against a reference
 * simulator (NVIDIA Newton's Cosserat rod solver, or MuJoCo's cable composite) -- see
 * scripts/compare_reference.py. Residual model error is then folded into
 * CarrierParams::safety_margin so that planning stays conservative. */
class RodSolver
{
public:
  explicit RodSolver(const CarrierParams& params);

  /** Solve from a fresh Hermite initial guess. */
  CarrierShape solve(const Eigen::Isometry3d& base_bracket, const Eigen::Isometry3d& tip_bracket) const;

  /** Solve warm-started from `guess` (typically the previous state's shape). Falls back to a
   *  cold start when the guess has the wrong size or is far from admissible. */
  CarrierShape solve(const Eigen::Isometry3d& base_bracket, const Eigen::Isometry3d& tip_bracket,
                     const CarrierShape& guess) const;

  const CarrierParams& params() const { return params_; }

private:
  /** Cubic Hermite curve from base to tip honouring both exit tangents, resampled to equal
   *  arc length. */
  EigenSTL::vector_Vector3d initialGuess(const Eigen::Isometry3d& base_bracket,
                                         const Eigen::Isometry3d& tip_bracket) const;

  /** Relative rotation of each joint, expressed in the previous edge's frame. This *is* the
   *  state: edge lengths cannot drift because they never appear as unknowns, and the bend limit
   *  is imposed by clamping these rotations. */
  using JointRotations = std::vector<Eigen::Matrix3d>;

  /** Seed joint rotations from a guessed centreline by parallel transport. */
  JointRotations rotationsFromPolyline(const Eigen::Isometry3d& base_bracket,
                                       const EigenSTL::vector_Vector3d& nodes) const;

  /** Clamp one joint to the hardware's bend limit (and to the back-stop, and to the bend plane). */
  void clampJoint(Eigen::Matrix3d& rotation) const;

  /** Forward kinematics: joint rotations -> edge frames and node positions. */
  void forward(const Eigen::Isometry3d& base_bracket, const JointRotations& rotations,
               EigenSTL::vector_Vector3d& nodes, std::vector<Eigen::Matrix3d>& frames) const;

  CarrierParams params_;
};

/** The bracket's exit direction: local +X of the bracket frame. Both brackets point *into* the
 *  carrier, i.e. the tip bracket's stored direction is reversed when used as an end tangent. */
inline Eigen::Vector3d exitDirection(const Eigen::Isometry3d& bracket)
{
  return bracket.linear().col(0).normalized();
}

}  // namespace moveit_cable_carrier
