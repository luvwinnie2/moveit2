// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit_cable_carrier/carrier_attacher.hpp>

#include <set>

namespace moveit_cable_carrier
{

void buildCapsuleChain(const CarrierShape& shape, std::vector<shapes::ShapeConstPtr>& shapes_out,
                       EigenSTL::vector_Isometry3d& poses_out)
{
  shapes_out.clear();
  poses_out.clear();
  if (shape.nodes.size() < 2)
  {
    return;
  }
  const double r = shape.radius;
  const size_t n = shape.nodes.size();
  shapes_out.reserve(2 * n);
  poses_out.reserve(2 * n);

  for (size_t i = 0; i + 1 < n; ++i)
  {
    const Eigen::Vector3d a = shape.nodes[i];
    const Eigen::Vector3d b = shape.nodes[i + 1];
    const Eigen::Vector3d d = b - a;
    const double len = d.norm();
    if (len < 1e-9)
    {
      continue;
    }
    // shapes::Cylinder is centred at the origin with its axis along local +Z.
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = 0.5 * (a + b);
    pose.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), d / len).toRotationMatrix();
    shapes_out.push_back(std::make_shared<const shapes::Cylinder>(r, len));
    poses_out.push_back(pose);
  }

  // Spheres at the interior nodes turn the cylinder chain into a true capsule chain: without them
  // the outer side of every bend would be uncovered.
  for (size_t i = 1; i + 1 < n; ++i)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = shape.nodes[i];
    shapes_out.push_back(std::make_shared<const shapes::Sphere>(r));
    poses_out.push_back(pose);
  }
}

CarrierAttacher::CarrierAttacher(const CarrierParams& params, const moveit::core::RobotModelConstPtr& model)
  : params_(params), model_(model), solver_(params), id_("carrier__" + params.name)
{
  valid_ = model_ && model_->hasLinkModel(params_.base_link) && model_->hasLinkModel(params_.tip_link);
}

CarrierShape CarrierAttacher::computeShape(const moveit::core::RobotState& state) const
{
  if (!valid_)
  {
    return CarrierShape{};
  }
  const Eigen::Isometry3d base_bracket = state.getGlobalLinkTransform(params_.base_link) * params_.base_mount;
  const Eigen::Isometry3d tip_bracket = state.getGlobalLinkTransform(params_.tip_link) * params_.tip_mount;

  CarrierShape shape = solver_.solve(base_bracket, tip_bracket, last_shape_);
  // Only keep a converged solve as the warm start; a failed one would poison the next query.
  if (shape.feasible)
  {
    last_shape_ = shape;
  }
  else
  {
    last_shape_ = CarrierShape{};
  }
  return shape;
}

bool CarrierAttacher::attachShape(moveit::core::RobotState& state, const CarrierShape& shape) const
{
  if (!valid_ || shape.empty())
  {
    return false;
  }

  CarrierShape simplified =
      params_.simplify_deviation > 0.0 ? shape.simplify(params_.simplify_deviation) : shape;

  std::vector<shapes::ShapeConstPtr> body_shapes;
  EigenSTL::vector_Isometry3d world_poses;
  buildCapsuleChain(simplified, body_shapes, world_poses);
  if (body_shapes.empty())
  {
    return false;
  }

  // AttachedBody stores shape poses relative to the body pose, which is relative to the parent
  // link. The solve is in the model frame, so pull it back into the base link's frame.
  const Eigen::Isometry3d link_to_model = state.getGlobalLinkTransform(params_.base_link).inverse();
  EigenSTL::vector_Isometry3d shape_poses;
  shape_poses.reserve(world_poses.size());
  for (const auto& p : world_poses)
  {
    shape_poses.push_back(link_to_model * p);
  }

  const std::set<std::string> touch(params_.touch_links.begin(), params_.touch_links.end());
  state.clearAttachedBody(id_);
  state.attachBody(id_, Eigen::Isometry3d::Identity(), body_shapes, shape_poses, touch, params_.base_link);
  return true;
}

bool CarrierAttacher::attach(moveit::core::RobotState& state) const
{
  const CarrierShape shape = computeShape(state);
  if (!shape.feasible)
  {
    return false;
  }
  return attachShape(state, shape);
}

}  // namespace moveit_cable_carrier
