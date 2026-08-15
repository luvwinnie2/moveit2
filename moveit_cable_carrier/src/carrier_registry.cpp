// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit_cable_carrier/carrier_registry.hpp>

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <fstream>
#include <sstream>

namespace moveit_cable_carrier
{
namespace
{

Eigen::Isometry3d parsePose(const YAML::Node& node)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  if (!node)
  {
    return pose;
  }
  if (node["xyz"] && node["xyz"].size() == 3)
  {
    pose.translation() = Eigen::Vector3d(node["xyz"][0].as<double>(), node["xyz"][1].as<double>(),
                                         node["xyz"][2].as<double>());
  }
  if (node["rpy"] && node["rpy"].size() == 3)
  {
    const double r = node["rpy"][0].as<double>();
    const double p = node["rpy"][1].as<double>();
    const double y = node["rpy"][2].as<double>();
    pose.linear() = (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()))
                        .toRotationMatrix();
  }
  return pose;
}

template <typename T>
void assignIf(const YAML::Node& node, const char* key, T& target)
{
  if (node[key])
  {
    target = node[key].as<T>();
  }
}

}  // namespace

bool parseCarrierYaml(const std::string& yaml_text, std::vector<CarrierParams>& out, std::string* error)
{
  try
  {
    const YAML::Node root = YAML::Load(yaml_text);
    const YAML::Node list = root["carriers"] ? root["carriers"] : root;
    if (!list || !list.IsSequence())
    {
      if (error)
      {
        *error = "expected a top-level 'carriers' sequence";
      }
      return false;
    }

    std::vector<CarrierParams> parsed;
    for (const auto& node : list)
    {
      CarrierParams c;
      assignIf(node, "name", c.name);
      assignIf(node, "length", c.length);
      assignIf(node, "bend_radius", c.bend_radius);
      assignIf(node, "outer_height", c.outer_height);
      assignIf(node, "outer_width", c.outer_width);
      assignIf(node, "unilateral", c.unilateral);
      assignIf(node, "gravity_sag", c.gravity_sag);
      assignIf(node, "num_segments", c.num_segments);
      assignIf(node, "max_iterations", c.max_iterations);
      assignIf(node, "tolerance", c.tolerance);
      assignIf(node, "safety_margin", c.safety_margin);
      assignIf(node, "youngs_modulus", c.youngs_modulus);
      assignIf(node, "shear_modulus", c.shear_modulus);
      assignIf(node, "density", c.density);
      assignIf(node, "bend_utilisation_warn", c.bend_utilisation_warn);
      assignIf(node, "relaxation", c.relaxation);
      assignIf(node, "energy_relaxation", c.energy_relaxation);
      assignIf(node, "simplify_deviation", c.simplify_deviation);
      assignIf(node, "base_link", c.base_link);
      assignIf(node, "tip_link", c.tip_link);

      if (node["bend_mode"])
      {
        c.bend_mode = node["bend_mode"].as<std::string>() == "spatial" ? BendMode::Spatial : BendMode::Planar;
      }
      if (node["bend_axis"] && node["bend_axis"].size() == 3)
      {
        c.bend_axis = Eigen::Vector3d(node["bend_axis"][0].as<double>(), node["bend_axis"][1].as<double>(),
                                      node["bend_axis"][2].as<double>());
      }
      if (node["gravity_dir"] && node["gravity_dir"].size() == 3)
      {
        c.gravity_dir = Eigen::Vector3d(node["gravity_dir"][0].as<double>(), node["gravity_dir"][1].as<double>(),
                                        node["gravity_dir"][2].as<double>());
      }
      c.base_mount = parsePose(node["base_mount"]);
      c.tip_mount = parsePose(node["tip_mount"]);
      if (node["touch_links"] && node["touch_links"].IsSequence())
      {
        for (const auto& t : node["touch_links"])
        {
          c.touch_links.push_back(t.as<std::string>());
        }
      }

      if (c.base_link.empty() || c.tip_link.empty())
      {
        if (error)
        {
          *error = "carrier '" + c.name + "' is missing base_link or tip_link";
        }
        return false;
      }
      parsed.push_back(std::move(c));
    }
    out = std::move(parsed);
    return true;
  }
  catch (const std::exception& e)
  {
    if (error)
    {
      *error = e.what();
    }
    return false;
  }
}

CarrierRegistry& CarrierRegistry::instance()
{
  static CarrierRegistry registry;
  return registry;
}

void CarrierRegistry::setCarriers(std::vector<CarrierParams> carriers)
{
  std::lock_guard<std::mutex> lock(mutex_);
  carriers_ = std::move(carriers);
  env_checked_ = true;  // an explicit configuration wins over the environment variable
}

void CarrierRegistry::addCarrier(CarrierParams carrier)
{
  std::lock_guard<std::mutex> lock(mutex_);
  carriers_.push_back(std::move(carrier));
  env_checked_ = true;
}

void CarrierRegistry::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  carriers_.clear();
  env_checked_ = true;
}

void CarrierRegistry::maybeLoadFromEnvUnlocked()
{
  if (env_checked_)
  {
    return;
  }
  env_checked_ = true;
  const char* path = std::getenv("MOVEIT_CABLE_CARRIER_CONFIG");
  if (!path || !*path)
  {
    return;
  }
  std::ifstream file(path);
  if (!file)
  {
    return;
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  std::vector<CarrierParams> parsed;
  if (parseCarrierYaml(buffer.str(), parsed, nullptr))
  {
    carriers_ = std::move(parsed);
  }
}

std::vector<CarrierParams> CarrierRegistry::carriers() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const_cast<CarrierRegistry*>(this)->maybeLoadFromEnvUnlocked();
  return carriers_;
}

bool CarrierRegistry::loadFromYaml(const std::string& path, std::string* error)
{
  std::ifstream file(path);
  if (!file)
  {
    if (error)
    {
      *error = "cannot open " + path;
    }
    return false;
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  std::vector<CarrierParams> parsed;
  if (!parseCarrierYaml(buffer.str(), parsed, error))
  {
    return false;
  }
  setCarriers(std::move(parsed));
  return true;
}

}  // namespace moveit_cable_carrier
