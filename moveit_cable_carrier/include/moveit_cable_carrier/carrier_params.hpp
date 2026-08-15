// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <string>
#include <vector>

namespace moveit_cable_carrier
{

/** How the carrier is allowed to bend.
 *
 * A drag chain / linkless cable chain (Silveyer KSL/KSH) is a *planar* device: it articulates
 * about one cross-section axis only and resists torsion. A 3D robot carrier (triflex style)
 * bends isotropically. Both share a hard minimum bend radius. */
enum class BendMode
{
  Planar,   ///< curvature only about the carrier's local bend axis
  Spatial   ///< curvature in any direction, magnitude bounded
};

/** Physical + mounting description of one cable carrier run.
 *
 * Lengths are metres, angles radians. Defaults describe a Kunimori Silveyer KSL-10
 * (inner 10x20 mm, R=40 mm option) with a short run suitable for a wrist. */
struct CarrierParams
{
  std::string name = "silveyer_ksl10";

  // ---- geometry ------------------------------------------------------------
  double length = 0.35;        ///< arc length of the carrier between the two brackets
  double bend_radius = 0.040;  ///< R_min, the hard minimum bend radius of the chain
  double outer_height = 0.016; ///< cross-section height (bend direction)
  double outer_width = 0.026;  ///< cross-section width

  // ---- mechanics -----------------------------------------------------------
  BendMode bend_mode = BendMode::Planar;
  /** Drag chains are back-stopped: they bend towards one side only and are held straight
   *  the other way. With `unilateral=true` the signed turn angle is clamped to [0, theta_max]
   *  instead of [-theta_max, theta_max]. */
  bool unilateral = true;
  /** Bend axis expressed in the *base bracket* frame. Only used when bend_mode==Planar. */
  Eigen::Vector3d bend_axis = Eigen::Vector3d::UnitZ();
  /** 0 = ignore gravity (pure elastica), 1 = full sag relaxation. The carrier is stiff, so
   *  small values are realistic; it mainly matters for long unsupported runs. */
  double gravity_sag = 0.15;
  /** Gravity direction in the planning frame. */
  Eigen::Vector3d gravity_dir = -Eigen::Vector3d::UnitZ();

  // ---- discretisation / solver --------------------------------------------
  int num_segments = 16;
  int max_iterations = 200;
  double tolerance = 1e-4;
  /** Under-relaxation of the curvature correction. Applying the full correction at every node in
   *  the same sweep makes the Gauss-Seidel projection overshoot and oscillate: a slack carrier has
   *  to relax a sharp initial bow into a wide loop, and correcting every node fully at once flings
   *  it apart. Small values converge more slowly but reliably. */
  double relaxation = 0.35;
  /** Strength of the secondary "straighten out" objective projected into the endpoint task's null
   *  space. This is what makes the solve pick the *minimum bending energy* shape among the many
   *  that reach the bracket, instead of one that sits against the bend limit. 0 disables it. */
  double energy_relaxation = 0.25;

  // ---- material (for internal-stress reporting) ----------------------------
  /** Young's modulus of the carrier body. Nylon 66 is ~2.0 GPa; steel-reinforced or PA-GF grades
   *  are stiffer. Only used to turn curvature into a stress number -- the *shape* comes from the
   *  bend-radius constraint, not from stiffness, so getting this wrong changes the reported
   *  stress but never the geometry. */
  double youngs_modulus = 2.0e9;   ///< Pa
  double shear_modulus = 0.7e9;    ///< Pa, ~E/2(1+nu) with nu~0.4
  double density = 1140.0;         ///< kg/m^3, nylon

  /** Fatigue screening threshold: fraction of the minimum bend radius that counts as "working the
   *  carrier hard". 1.0 means running exactly at R_min. Drag-chain makers normally want a working
   *  radius well above R_min for long life, so the default flags anything above 0.8. */
  double bend_utilisation_warn = 0.8;

  // ---- conservatism --------------------------------------------------------
  /** Added to every capsule radius. Absorbs model error, hysteresis and the fact that a
   *  rectangular cross-section is approximated by a capsule. */
  double safety_margin = 0.006;
  /** Extra radius added when the polyline is decimated (see CarrierShape::simplify). */
  double simplify_deviation = 0.0;

  // ---- mounting ------------------------------------------------------------
  std::string base_link;                                      ///< link carrying the fixed bracket
  std::string tip_link;                                       ///< link carrying the moving bracket
  Eigen::Isometry3d base_mount = Eigen::Isometry3d::Identity();  ///< bracket frame in base_link
  Eigen::Isometry3d tip_mount = Eigen::Isometry3d::Identity();   ///< bracket frame in tip_link
  /** Links the carrier is allowed to touch (its own mounts and whatever it legitimately
   *  rubs against). Passed straight to MoveIt as the attached body's touch_links, so no ACM
   *  surgery is needed. */
  std::vector<std::string> touch_links;

  // ---- derived -------------------------------------------------------------
  /** Conservative radius of the capsule that encloses the rectangular cross-section. */
  double capsuleRadius() const
  {
    return 0.5 * std::hypot(outer_height, outer_width) + safety_margin;
  }
  double segmentLength() const
  {
    return length / static_cast<double>(num_segments);
  }
  /** Maximum turn angle between consecutive segments implied by R_min. */
  double maxTurnAngle() const
  {
    return segmentLength() / bend_radius;
  }
};

}  // namespace moveit_cable_carrier
