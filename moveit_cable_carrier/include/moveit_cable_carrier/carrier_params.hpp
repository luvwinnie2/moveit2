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

/** What kind of flexible run this is. The kind decides which limits are meaningful, so it is not
 *  cosmetic: a ball-and-socket carrier has a hard bend radius and a per-link torsion stop, whereas
 *  a bare cable has no stop at all and is limited only by a multiple of its own diameter. */
enum class CarrierKind
{
  /** 3D robot dresspack, ball-and-socket links (igus triflex R and equivalents). Hard minimum
   *  bend radius, isotropic bending, and a torsion stop of roughly +-10 deg per link. */
  ArticulatedCarrier,
  /** Planar link chain / linkless cable chain for linear axes (Kunimori Silveyer KSL/KSH,
   *  classic drag chains). One bend axis, back-stopped, essentially no torsion. */
  PlanarChain,
  /** Bare cable or bundle with no carrier around it. No hard stop: the limit is the published
   *  minimum bend radius, conventionally a multiple of the outer diameter. */
  BareCable,
  /** Corrugated conduit. Bends freely in all directions; the limit comes from the conduit spec
   *  and it offers no torsion stop, which is precisely why triflex is preferred on robots. */
  CorrugatedHose,
};

/** A cable or hose routed *inside* the carrier.
 *
 *  The carrier's own bend limit is not the whole story: what actually fails in service is usually
 *  a conductor inside it. Industry practice sizes that by a multiple of the cable's outer
 *  diameter -- 10x OD is the usual figure for continuous-flex robot cables, with 7.5x OD only for
 *  qualified high-flex constructions in compact runs. */
struct InnerCable
{
  std::string name = "cable";
  double outer_diameter = 0.008;   ///< m
  /** Minimum bend radius as a multiple of the outer diameter. */
  double min_bend_factor = 10.0;
  int count = 1;
  /** Minimum bend radius in metres implied by the factor. */
  double minBendRadius() const { return min_bend_factor * outer_diameter; }
};

/** How the fixed end is attached, which changes what the run is allowed to do.
 *
 *  Robot dresspacks are mounted either rigidly to a link, or on a pivot/rotating flange that lets
 *  the run swing, or through a retraction unit whose spring takes up the service loop so it cannot
 *  flap into the work area. */
enum class MountStyle
{
  Fixed,
  Pivot,
  Retraction,
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

  // ---- kind ----------------------------------------------------------------
  CarrierKind kind = CarrierKind::ArticulatedCarrier;
  /** Cables/hoses routed inside. Used for the bend-radius check that actually predicts failure. */
  std::vector<InnerCable> inner_cables;

  /** Torsion stop, radians per link. A triflex-style carrier allows roughly +-10 deg of twist per
   *  link about its own axis; a planar chain allows essentially none; a bare cable has no stop.
   *  Negative disables the limit. */
  double twist_limit_per_link = 10.0 * M_PI / 180.0;

  // ---- mounting style ------------------------------------------------------
  MountStyle mount_style = MountStyle::Fixed;
  /** Retraction units pull the service loop back with a roughly constant spring force. Modelled
   *  as a directional bias on the slack rather than a force, since the shape solve is quasi-static:
   *  0 disables it, 1 pulls the slack fully towards `retraction_dir`. */
  double retraction_bias = 0.0;
  /** Direction, in the base bracket frame, that the retraction unit pulls the loop. */
  Eigen::Vector3d retraction_dir = -Eigen::Vector3d::UnitX();

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
  /** Radius that mechanically limits the *shape*.
   *
   *  For a carrier this is its own stop: the hardware physically refuses to bend tighter, and a
   *  cable inside that is unhappy about it gets damaged rather than stopping the chain. For a bare
   *  cable or a hose there is no stop, so its own published limit is what governs. */
  double shapeBendRadius() const
  {
    return (kind == CarrierKind::BareCable || kind == CarrierKind::CorrugatedHose)
               ? effectiveBendRadius()
               : bend_radius;
  }
  /** Maximum turn angle between consecutive segments implied by the shape limit. */
  double maxTurnAngle() const
  {
    return segmentLength() / shapeBendRadius();
  }
  /** Tightest bend radius any cable inside will tolerate, 0 if none are declared. */
  double innerCableLimitRadius() const
  {
    double worst = 0.0;
    for (const auto& c : inner_cables)
    {
      worst = std::max(worst, c.minBendRadius());
    }
    return worst;
  }
  /** The binding limit: the carrier's own stop, or its contents, whichever is tighter. */
  double effectiveBendRadius() const
  {
    return std::max(bend_radius, innerCableLimitRadius());
  }
};

}  // namespace moveit_cable_carrier
