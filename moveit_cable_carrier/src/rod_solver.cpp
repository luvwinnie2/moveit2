// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit_cable_carrier/rod_solver.hpp>

#include <algorithm>
#include <limits>

namespace moveit_cable_carrier
{
namespace
{
constexpr double kEps = 1e-12;

/** Weight of a node in the constraint projections: 0 for the four clamped nodes, 1 otherwise.
 *  Clamping *two* nodes at each end is what encodes the bracket orientation -- a single fixed
 *  point would only constrain position, and the carrier would be free to leave the bracket at
 *  any angle, which a real bracket does not allow. */
inline double nodeWeight(int i, int n)
{
  return (i <= 1 || i >= n - 1) ? 0.0 : 1.0;
}

/** Cubic Hermite between the two brackets plus a perpendicular bow.
 *
 *  The bow uses sin^2, which vanishes *and* has zero derivative at both ends, so it changes the
 *  curve's length without disturbing either bracket's exit tangent. */
EigenSTL::vector_Vector3d hermiteBow(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1,
                                     const Eigen::Vector3d& d0, const Eigen::Vector3d& d1,
                                     const Eigen::Vector3d& bow_dir, double tau, double amp, int dense)
{
  EigenSTL::vector_Vector3d curve;
  curve.reserve(dense + 1);
  const Eigen::Vector3d m0 = d0 * tau;
  const Eigen::Vector3d m1 = d1 * tau;
  for (int k = 0; k <= dense; ++k)
  {
    const double t = static_cast<double>(k) / dense;
    const double t2 = t * t, t3 = t2 * t;
    const double h00 = 2 * t3 - 3 * t2 + 1;
    const double h10 = t3 - 2 * t2 + t;
    const double h01 = -2 * t3 + 3 * t2;
    const double h11 = t3 - t2;
    const double s = std::sin(M_PI * t);
    curve.push_back(h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1 + (amp * s * s) * bow_dir);
  }
  return curve;
}

double polylineLength(const EigenSTL::vector_Vector3d& curve)
{
  double total = 0.0;
  for (size_t k = 1; k < curve.size(); ++k)
  {
    total += (curve[k] - curve[k - 1]).norm();
  }
  return total;
}
}  // namespace

RodSolver::RodSolver(const CarrierParams& params) : params_(params)
{
  params_.num_segments = std::max(4, params_.num_segments);
  params_.bend_axis.normalize();
  if (params_.gravity_dir.norm() > kEps)
  {
    params_.gravity_dir.normalize();
  }
}

EigenSTL::vector_Vector3d RodSolver::initialGuess(const Eigen::Isometry3d& base_bracket,
                                                  const Eigen::Isometry3d& tip_bracket) const
{
  const int n = params_.num_segments;
  const double L = params_.length;
  const Eigen::Vector3d p0 = base_bracket.translation();
  const Eigen::Vector3d p1 = tip_bracket.translation();
  // Both bracket frames have +X along the direction of travel (base -> tip), so both Hermite
  // tangents take the same sign. Negating the tip tangent makes the guess double back on itself
  // and the curvature clamp then cannot recover a straight run.
  const Eigen::Vector3d d0 = exitDirection(base_bracket);
  const Eigen::Vector3d d1 = exitDirection(tip_bracket);

  // Direction the slack bows into. For a back-stopped chain the permitted turning side is
  // bend_axis x chord, which is where a real drag chain loops.
  const Eigen::Vector3d chord = p1 - p0;
  const double chord_len = chord.norm();
  const Eigen::Vector3d chord_dir = chord_len > kEps ? (chord / chord_len).eval() : d0;
  const Eigen::Vector3d bend_axis_world = (base_bracket.linear() * params_.bend_axis).normalized();
  // chord x bend_axis, not the other way round: travelling along the chord and turning towards
  // the permitted side puts the middle of the loop on *this* side. Getting the sign wrong points
  // the guess away from the loop the chain can actually form, and the curvature clamp then has to
  // drag it all the way across.
  Eigen::Vector3d bow_dir = chord_dir.cross(bend_axis_world);
  bow_dir = bow_dir.norm() > 1e-6 ? bow_dir.normalized() : chord_dir.unitOrthogonal();

  // The guess must already have arc length L, otherwise the inextensibility projection has to
  // create the slack itself -- and pushing collinear nodes apart along a straight line simply
  // cannot buckle, so the solve stalls. Size the curve first, then let the projection clean up.
  const int dense_probe = 48;
  auto lengthAt = [&](double tau, double amp) {
    return polylineLength(hermiteBow(p0, p1, d0, d1, bow_dir, tau, amp, dense_probe));
  };

  // Absorb the slack with the perpendicular bow, NOT by growing the Hermite tangents. Long
  // tangents make the curve overshoot and fold back on itself; a fold is an almost-180-degree
  // turn, where the bend axis is numerically undefined and the curvature clamp has nothing to
  // work with. Bowing sideways keeps the guess smooth and turns straight into the U-loop a real
  // drag chain actually forms.
  const double tau = std::min(0.40 * L, 0.60 * chord_len + 0.10 * L);
  double amp = 0.0;
  if (lengthAt(tau, 0.0) < L)
  {
    double lo = 0.0, hi = 2.0 * L;
    for (int it = 0; it < 28; ++it)
    {
      const double mid = 0.5 * (lo + hi);
      (lengthAt(tau, mid) < L ? lo : hi) = mid;
    }
    amp = 0.5 * (lo + hi);
  }

  // Resample at equal arc length so every edge starts at the right length.
  const int dense = std::max(128, n * 8);
  EigenSTL::vector_Vector3d curve = hermiteBow(p0, p1, d0, d1, bow_dir, tau, amp, dense);

  std::vector<double> s(curve.size(), 0.0);
  for (size_t k = 1; k < curve.size(); ++k)
  {
    s[k] = s[k - 1] + (curve[k] - curve[k - 1]).norm();
  }
  const double total = s.back();

  EigenSTL::vector_Vector3d out(n + 1);
  if (total < kEps)
  {
    std::fill(out.begin(), out.end(), p0);
    return out;
  }
  size_t k = 0;
  for (int i = 0; i <= n; ++i)
  {
    const double target = total * static_cast<double>(i) / n;
    while (k + 1 < s.size() && s[k + 1] < target)
    {
      ++k;
    }
    const double seg = s[k + 1] - s[k];
    const double a = seg > kEps ? (target - s[k]) / seg : 0.0;
    out[i] = curve[k] + a * (curve[std::min(k + 1, curve.size() - 1)] - curve[k]);
  }
  return out;
}

namespace
{
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

inline Eigen::Vector3d logSO3(const Eigen::Matrix3d& rotation)
{
  const Eigen::AngleAxisd aa(rotation);
  return aa.axis() * aa.angle();
}

inline Eigen::Matrix3d expSO3(const Eigen::Vector3d& w)
{
  const double angle = w.norm();
  if (angle < 1e-12)
  {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(angle, w / angle).toRotationMatrix();
}
}  // namespace

void RodSolver::clampJoint(Eigen::Matrix3d& rotation) const
{
  Eigen::Vector3d v = logSO3(rotation);
  const double theta_max = params_.maxTurnAngle();

  // Split the joint rotation into twist about the carrier's own axis and bending perpendicular to
  // it. They are different mechanical stops and must be limited separately: a 3D dresspack quotes
  // a bend radius *and* a torsion stop of roughly +-10 deg per link, and lumping them into one
  // magnitude bound lets a solve trade one for the other.
  const Eigen::Vector3d axis = Eigen::Vector3d::UnitX();  // local +X runs along the carrier
  double twist = v.dot(axis);
  Eigen::Vector3d bend = v - twist * axis;

  if (params_.twist_limit_per_link >= 0.0)
  {
    twist = std::clamp(twist, -params_.twist_limit_per_link, params_.twist_limit_per_link);
  }

  if (params_.bend_mode == BendMode::Planar)
  {
    // A planar chain articulates about one axis only; the back-stop then makes it one-sided.
    const Eigen::Vector3d bend_axis = (params_.bend_axis - params_.bend_axis.dot(axis) * axis);
    if (bend_axis.norm() > 1e-9)
    {
      const Eigen::Vector3d unit = bend_axis.normalized();
      double sbend = bend.dot(unit);
      sbend = params_.unilateral ? std::clamp(sbend, 0.0, theta_max) : std::clamp(sbend, -theta_max, theta_max);
      bend = sbend * unit;
    }
    else if (bend.norm() > theta_max)
    {
      bend *= theta_max / bend.norm();
    }
  }
  else if (bend.norm() > theta_max)
  {
    bend *= theta_max / bend.norm();
  }

  rotation = expSO3(bend + twist * axis);
}

void RodSolver::forward(const Eigen::Isometry3d& base_bracket, const JointRotations& rotations,
                        EigenSTL::vector_Vector3d& nodes, std::vector<Eigen::Matrix3d>& frames) const
{
  const int n = params_.num_segments;
  const double seg_len = params_.segmentLength();
  frames.assign(n + 1, Eigen::Matrix3d::Identity());
  nodes.assign(n + 1, base_bracket.translation());

  // frames[k] is the frame of edge k, which runs from node k-1 to node k. Edge 1 is pinned to the
  // base bracket, so both the start point and the exit tangent are satisfied by construction.
  frames[1] = base_bracket.linear();
  nodes[1] = nodes[0] + seg_len * frames[1].col(0);
  for (int k = 2; k <= n; ++k)
  {
    frames[k] = frames[k - 1] * rotations[k - 2];
    nodes[k] = nodes[k - 1] + seg_len * frames[k].col(0);
  }
}

RodSolver::JointRotations RodSolver::rotationsFromPolyline(const Eigen::Isometry3d& base_bracket,
                                                          const EigenSTL::vector_Vector3d& nodes) const
{
  const int n = params_.num_segments;
  JointRotations rotations(static_cast<size_t>(std::max(0, n - 1)), Eigen::Matrix3d::Identity());
  if (static_cast<int>(nodes.size()) != n + 1)
  {
    return rotations;
  }

  // Parallel transport: each frame is the previous one turned by the minimal rotation that lines
  // its +X up with the next edge, so no spurious twist is introduced.
  Eigen::Matrix3d frame = base_bracket.linear();
  for (int i = 1; i < n; ++i)
  {
    const Eigen::Vector3d next = nodes[i + 1] - nodes[i];
    if (next.norm() < kEps)
    {
      continue;
    }
    const Eigen::Matrix3d align =
        Eigen::Quaterniond::FromTwoVectors(frame.col(0), next.normalized()).toRotationMatrix();
    Eigen::Matrix3d rel = frame.transpose() * (align * frame);
    clampJoint(rel);
    rotations[static_cast<size_t>(i - 1)] = rel;
    frame = frame * rel;
  }
  return rotations;
}

CarrierShape RodSolver::solve(const Eigen::Isometry3d& base_bracket,
                              const Eigen::Isometry3d& tip_bracket) const
{
  return solve(base_bracket, tip_bracket, CarrierShape{});
}

CarrierShape RodSolver::solve(const Eigen::Isometry3d& base_bracket, const Eigen::Isometry3d& tip_bracket,
                              const CarrierShape& guess) const
{
  const int n = params_.num_segments;
  const double seg_len = params_.segmentLength();

  CarrierShape out;
  out.radius = params_.capsuleRadius();

  // A carrier of arc length L can never span a chord longer than L. This is the single most common
  // way a mounting configuration is wrong and must not be reported as a converged shape.
  const double chord = (tip_bracket.translation() - base_bracket.translation()).norm();

  // The unknowns are the joint rotations, not the node positions. That is the whole point of this
  // formulation: edge lengths cannot drift because they are never solved for, and the minimum bend
  // radius is imposed by clamping each joint. Only the *far* bracket's pose is left to iterate on.
  // Solving in position space instead makes the length and curvature constraints fight each other
  // and the iteration settles on a compromise that satisfies neither.
  const EigenSTL::vector_Vector3d seed =
      (static_cast<int>(guess.nodes.size()) == n + 1) ? guess.nodes : initialGuess(base_bracket, tip_bracket);
  JointRotations rotations = rotationsFromPolyline(base_bracket, seed);

  const Eigen::Vector3d target_p = tip_bracket.translation();
  const Eigen::Vector3d target_d = exitDirection(tip_bracket);
  // Puts the orientation residual in the same units as the position residual so one tolerance and
  // one damping factor cover both.
  const double ang_scale = 0.5 * params_.length;

  EigenSTL::vector_Vector3d nodes;
  std::vector<Eigen::Matrix3d> frames;
  Eigen::MatrixXd jacobian(6, 3 * std::max(1, n - 1));
  Eigen::Matrix<double, 6, 1> residual;
  const double damping = 1e-6 + 1e-4 * params_.length * params_.length;

  double pos_err = 0.0;
  double ang_err = 0.0;
  int it = 0;
  for (; it < params_.max_iterations; ++it)
  {
    forward(base_bracket, rotations, nodes, frames);
    const Eigen::Vector3d tip_dir = frames[n].col(0);
    const Eigen::Vector3d dp = target_p - nodes[n];
    const Eigen::Vector3d dw = tip_dir.cross(target_d);
    pos_err = dp.norm();
    ang_err = dw.norm();
    residual.head<3>() = dp;
    residual.tail<3>() = ang_scale * dw;
    if (residual.norm() < params_.tolerance)
    {
      break;
    }

    for (int i = 1; i < n; ++i)
    {
      // A rotation dw applied at joint i moves the tip by dw x (p_n - p_i) and turns it by dw.
      jacobian.block<3, 3>(0, 3 * (i - 1)) = -skew(nodes[n] - nodes[i]);
      jacobian.block<3, 3>(3, 3 * (i - 1)) = ang_scale * Eigen::Matrix3d::Identity();
    }

    // Damped least squares in its transpose form: J J^T is only 6x6 however long the chain is.
    const Eigen::Matrix<double, 6, 6> normal =
        jacobian * jacobian.transpose() + damping * Eigen::Matrix<double, 6, 6>::Identity();
    const auto factorisation = normal.ldlt();
    Eigen::VectorXd step = jacobian.transpose() * factorisation.solve(residual);

    // Null-space term: relax every joint back towards straight, in whatever directions do not
    // disturb the bracket. Without it the minimum-norm step merely finds *a* feasible shape and
    // happily leaves joints pinned against the bend limit, so the carrier gets reported as running
    // at exactly R_min everywhere. A real carrier spreads the bend out to minimise elastic energy,
    // which is what this reproduces -- and it is what makes the shape comparable to an
    // energy-minimising reference solver such as MuJoCo's cable or Newton's rod.
    // Anneal the secondary objective away instead of gating it on the residual. The joint clamp is
    // a hard projection applied *after* the step, so null-space orthogonality is only approximate
    // and the straightening term always disturbs the bracket a little. Gating on the residual made
    // that a limit cycle -- converge, term switches on, residual grows, term switches off -- so
    // every solve ran to max_iterations. Annealing lets the early iterations pick a low-energy
    // shape and gives the endpoint task the last word, which converges cleanly.
    const double anneal = 1.0 - static_cast<double>(it) / (0.6 * params_.max_iterations);
    const double gain = anneal > 0.0 ? params_.energy_relaxation * anneal : 0.0;
    if (gain > 1e-6)
    {
      Eigen::VectorXd secondary(3 * std::max(1, n - 1));
      secondary.setZero();
      for (int i = 1; i < n; ++i)
      {
        const Eigen::Vector3d local = logSO3(rotations[static_cast<size_t>(i - 1)]);
        secondary.segment<3>(3 * (i - 1)) = -gain * (frames[i] * local);
      }
      step += secondary - jacobian.transpose() * factorisation.solve(jacobian * secondary);
    }

    for (int i = 1; i < n; ++i)
    {
      const Eigen::Vector3d world_increment = step.segment<3>(3 * (i - 1));
      // The increment is a world-frame rotation applied at the joint; express it in that joint's
      // own frame before composing it with the stored relative rotation.
      rotations[static_cast<size_t>(i - 1)] =
          expSO3(frames[i].transpose() * world_increment) * rotations[static_cast<size_t>(i - 1)];
      clampJoint(rotations[static_cast<size_t>(i - 1)]);
    }
  }

  forward(base_bracket, rotations, nodes, frames);
  out.nodes = nodes;
  out.iterations = it;
  out.endpoint_error = (target_p - nodes[n]).norm();

  // ---- internal load ----
  // Each joint rotation is the material frame's change over one segment, so dividing by the
  // segment length turns it straight into curvature and twist rate. The component along the
  // carrier's own axis is twist; what is perpendicular to it is bending.
  out.curvature.clear();
  out.twist_per_link.clear();
  out.curvature.reserve(rotations.size());
  out.twist_per_link.reserve(rotations.size());
  out.max_curvature = 0.0;
  out.max_twist_per_link = 0.0;
  for (const auto& rotation : rotations)
  {
    const Eigen::Vector3d v = logSO3(rotation);
    const double twist = std::abs(v.x());                    // per link, local +X is the axis
    const double bend = std::hypot(v.y(), v.z()) / seg_len;  // per metre => curvature
    out.curvature.push_back(bend);
    out.twist_per_link.push_back(twist);
    out.max_curvature = std::max(out.max_curvature, bend);
    out.max_twist_per_link = std::max(out.max_twist_per_link, twist);
  }

  out.bend_utilisation = out.max_curvature * params_.shapeBendRadius();
  out.twist_utilisation = params_.twist_limit_per_link > 0.0
                              ? out.max_twist_per_link / params_.twist_limit_per_link
                              : 0.0;

  // What the cables inside see. The achieved radius is compared against each cable's own published
  // minimum, expressed the way cable makers express it: as a multiple of the outer diameter.
  const double achieved_radius = out.min_bend_radius();
  out.min_cable_bend_ratio = 0.0;
  out.required_cable_bend_ratio = 0.0;
  out.max_cable_strain = 0.0;
  out.worst_cable.clear();
  double worst_headroom = std::numeric_limits<double>::max();
  for (const auto& cable : params_.inner_cables)
  {
    if (cable.outer_diameter <= 0.0)
    {
      continue;
    }
    const double ratio = achieved_radius > 0.0 ? achieved_radius / cable.outer_diameter
                                               : std::numeric_limits<double>::max();
    const double headroom = ratio - cable.min_bend_factor;
    if (headroom < worst_headroom)
    {
      worst_headroom = headroom;
      out.min_cable_bend_ratio = ratio;
      out.required_cable_bend_ratio = cable.min_bend_factor;
      out.worst_cable = cable.name;
    }
    if (achieved_radius > 0.0)
    {
      out.max_cable_strain = std::max(out.max_cable_strain, 0.5 * cable.outer_diameter / achieved_radius);
    }
  }

  // Edge lengths and the bend limit hold by construction, so feasibility is exactly "did the far
  // bracket end up where the robot says it is".
  out.feasible = (chord <= params_.length) && (out.endpoint_error < 50.0 * params_.tolerance) &&
                 (ang_err < 0.05);
  return out;
}

CarrierShape CarrierShape::simplify(double max_deviation) const
{
  CarrierShape out;
  out.feasible = feasible;
  out.endpoint_error = endpoint_error;
  out.max_curvature = max_curvature;
  out.iterations = iterations;
  out.radius = radius;
  if (nodes.size() < 3 || max_deviation <= 0.0)
  {
    out.nodes = nodes;
    return out;
  }

  // Greedy forward merge: extend the current segment as long as every skipped node stays within
  // max_deviation of it. The deviation is then added to the radius, so the simplified capsule
  // chain still encloses the original centreline.
  out.nodes.push_back(nodes.front());
  size_t anchor = 0;
  double worst = 0.0;
  for (size_t j = 2; j < nodes.size(); ++j)
  {
    const Eigen::Vector3d a = nodes[anchor];
    const Eigen::Vector3d b = nodes[j];
    const Eigen::Vector3d ab = b - a;
    const double ab2 = ab.squaredNorm();
    double local_worst = 0.0;
    for (size_t k = anchor + 1; k < j; ++k)
    {
      const Eigen::Vector3d ap = nodes[k] - a;
      const double t = ab2 > kEps ? std::clamp(ap.dot(ab) / ab2, 0.0, 1.0) : 0.0;
      local_worst = std::max(local_worst, (ap - t * ab).norm());
    }
    if (local_worst > max_deviation)
    {
      out.nodes.push_back(nodes[j - 1]);
      anchor = j - 1;
    }
    else
    {
      worst = std::max(worst, local_worst);
    }
  }
  out.nodes.push_back(nodes.back());
  out.radius = radius + worst;
  return out;
}

}  // namespace moveit_cable_carrier
