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
  /** Per-joint twist about the carrier axis [rad per link]. Directly comparable to the torsion
   *  stop a 3D carrier quotes (about +-10 deg per link), which is why it is per link and not
   *  per metre. */
  std::vector<double> twist_per_link;
  double max_twist_per_link = 0.0;
  /** Fraction of the torsion stop used up. >1 means the mounting is forcing more twist than the
   *  hardware allows, which shows up in service as links popping apart. */
  double twist_utilisation = 0.0;

  /** max_curvature * bend_radius. 1.0 = running exactly at the hardware's minimum radius,
   *  >1.0 = over-bent. This is the single number to watch. */
  double bend_utilisation = 0.0;
  /** Tightest radius actually reached [m] (= 1/max_curvature). */
  double min_bend_radius() const { return max_curvature > 1e-9 ? 1.0 / max_curvature : 0.0; }

  // ---- what the cables inside actually see ---------------------------------
  /** Tightest achieved radius divided by the worst inner cable's outer diameter. This is the
   *  number the cable industry sizes against: continuous-flex robot cable wants about 10x OD, and
   *  qualified high-flex constructions about 7.5x OD in compact runs. Going below the validated
   *  figure is what turns a ten-million-cycle life into a fraction of it.
   *  0 when no inner cables are declared. */
  double min_cable_bend_ratio = 0.0;
  /** Required ratio for the worst cable, from its own spec. */
  double required_cable_bend_ratio = 0.0;
  /** Name of the cable that is closest to its limit. */
  std::string worst_cable;
  /** Outer-fibre bending strain of that cable, (OD/2) / R. Dimensionless.
   *
   *  Deliberately not a stress in Pa: the carrier shell is an articulated ball-and-socket chain
   *  that bends at its joints, so Euler-Bernoulli beam stress on the shell is meaningless and
   *  produces numbers far past the yield of any polymer it is made from. Strain of the cable
   *  inside is the quantity that governs conductor fatigue. */
  double max_cable_strain = 0.0;

  /** True when every declared cable stays within its own minimum bend radius. */
  bool cablesWithinLimit() const
  {
    return required_cable_bend_ratio <= 0.0 || min_cable_bend_ratio >= required_cable_bend_ratio;
  }

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
