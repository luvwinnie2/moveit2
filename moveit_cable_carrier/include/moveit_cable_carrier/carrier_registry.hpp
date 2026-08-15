// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit_cable_carrier/carrier_params.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace moveit_cable_carrier
{

/** Process-wide carrier configuration.
 *
 * The collision detector is instantiated by MoveIt's allocator template, which only passes a
 * robot model and a world -- there is no route for per-detector configuration. So the carrier
 * definitions live here and are read at CollisionEnv construction time.
 *
 * Populate it before the planning scene allocates the detector, either programmatically or from
 * YAML. As a convenience the registry also loads the file named by the
 * MOVEIT_CABLE_CARRIER_CONFIG environment variable on first use. */
class CarrierRegistry
{
public:
  static CarrierRegistry& instance();

  void setCarriers(std::vector<CarrierParams> carriers);
  void addCarrier(CarrierParams carrier);
  void clear();

  /** Snapshot of the configured carriers. Returns a copy so callers are not exposed to
   *  concurrent reconfiguration. */
  std::vector<CarrierParams> carriers() const;

  /** Load carrier definitions from a YAML file. Replaces the current set on success.
   *  Returns false and leaves the registry untouched if the file cannot be parsed. */
  bool loadFromYaml(const std::string& path, std::string* error = nullptr);

private:
  CarrierRegistry() = default;
  void maybeLoadFromEnvUnlocked();

  mutable std::mutex mutex_;
  std::vector<CarrierParams> carriers_;
  bool env_checked_ = false;
};

/** Parse a single carrier entry from a YAML node. Exposed for tests and for tools that read
 *  their own config layout. */
bool parseCarrierYaml(const std::string& yaml_text, std::vector<CarrierParams>& out, std::string* error);

}  // namespace moveit_cable_carrier
