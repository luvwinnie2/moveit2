// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit_cable_carrier/carrier_attacher.hpp>

#include <geometric_shapes/shape_operations.h>

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
  skip_links_ = { params_.base_link, params_.tip_link };
}

std::vector<Obstacle> CarrierAttacher::buildObstacles(const moveit::core::RobotState& state) const
{
  std::vector<Obstacle> out;
  if (!model_)
  {
    return out;
  }
  for (const moveit::core::LinkModel* link : model_->getLinkModels())
  {
    if (!link || skip_links_.count(link->getName()))
    {
      continue;
    }
    const auto& shapes_in = link->getShapes();
    const auto& origins = link->getCollisionOriginTransforms();
    const Eigen::Isometry3d& to_world = state.getGlobalLinkTransform(link);

    for (size_t i = 0; i < shapes_in.size(); ++i)
    {
      if (!shapes_in[i])
      {
        continue;
      }
      const Eigen::Isometry3d pose = to_world * (i < origins.size() ? origins[i] : Eigen::Isometry3d::Identity());
      Obstacle o;
      o.name = link->getName();

      switch (shapes_in[i]->type)
      {
        case shapes::CYLINDER:
        {
          const auto* c = static_cast<const shapes::Cylinder*>(shapes_in[i].get());
          const Eigen::Vector3d axis = pose.linear().col(2) * (0.5 * c->length);
          o.a = pose.translation() - axis;
          o.b = pose.translation() + axis;
          o.radius = c->radius;
          break;
        }
        case shapes::BOX:
        {
          // Capsule along the longest side, radius covering the other two. Conservative: it
          // encloses the box, so the carrier is kept slightly further out than strictly needed.
          const auto* b = static_cast<const shapes::Box*>(shapes_in[i].get());
          const double sx = b->size[0], sy = b->size[1], sz = b->size[2];
          int longest = (sx >= sy && sx >= sz) ? 0 : (sy >= sz ? 1 : 2);
          const double half = 0.5 * b->size[longest];
          const double r1 = 0.5 * b->size[(longest + 1) % 3];
          const double r2 = 0.5 * b->size[(longest + 2) % 3];
          const Eigen::Vector3d axis = pose.linear().col(longest) * half;
          o.a = pose.translation() - axis;
          o.b = pose.translation() + axis;
          o.radius = std::hypot(r1, r2);
          break;
        }
        case shapes::SPHERE:
        {
          const auto* sp = static_cast<const shapes::Sphere*>(shapes_in[i].get());
          o.a = o.b = pose.translation();
          o.radius = sp->radius;
          break;
        }
        default:
        {
          // Mesh or anything else: fall back to its bounding sphere rather than skipping it, so an
          // unmodelled shape errs towards keeping the carrier away instead of letting it through.
          Eigen::Vector3d centre = Eigen::Vector3d::Zero();
          double radius = 0.0;
          shapes::computeShapeBoundingSphere(shapes_in[i].get(), centre, radius);
          o.a = o.b = pose * centre;
          o.radius = radius;
          break;
        }
      }
      if (o.radius > 0.0)
      {
        out.push_back(std::move(o));
      }
    }
  }
  return out;
}

CarrierShape CarrierAttacher::computeShape(const moveit::core::RobotState& state) const
{
  if (!valid_)
  {
    return CarrierShape{};
  }
  const Eigen::Isometry3d base_bracket = state.getGlobalLinkTransform(params_.base_link) * params_.base_mount;
  const Eigen::Isometry3d tip_bracket = state.getGlobalLinkTransform(params_.tip_link) * params_.tip_mount;

  solver_.setObstacles(buildObstacles(state));
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
