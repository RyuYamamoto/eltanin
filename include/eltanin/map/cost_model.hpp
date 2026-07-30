// Copyright 2026 RyuYamamoto.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License

#ifndef ELTANIN__MAP__COST_MODEL_HPP_
#define ELTANIN__MAP__COST_MODEL_HPP_

#include <eltanin/core/footprint.hpp>
#include <eltanin/core/traversability.hpp>
#include <eltanin/map/cost_values.hpp>

#include <cstdint>
#include <optional>

namespace eltanin::map
{

/// Converts an obstacle distance [m] to a nav2-scale cost, decaying from inscribed_radius.
class InflationCostModel
{
public:
  static std::optional<InflationCostModel> create(
    const CollisionRadii & radii, double cost_scaling_factor);

  std::uint8_t cost_at_distance(double distance) const noexcept;

  /// Threshold for the circumscribed radius; never below 1 so FREE_SPACE stays traversable.
  std::uint8_t circumscribed_cost() const noexcept { return circumscribed_cost_; }

  const CollisionRadii & radii() const noexcept { return radii_; }

private:
  InflationCostModel(const CollisionRadii & radii, double cost_scaling_factor);

  CollisionRadii radii_;
  double cost_scaling_factor_{0.0};
  std::uint8_t circumscribed_cost_{1};
};

/// Cost-input traversability model; the one actually used in the first stage.
class CostTraversabilityModel
{
public:
  explicit CostTraversabilityModel(
    std::uint8_t circumscribed_cost, bool unknown_is_free = false)
  : circumscribed_cost_(circumscribed_cost), unknown_is_traversable_(unknown_is_free)
  {
  }

  Traversability classify(std::uint8_t cost) const noexcept
  {
    if (cost == NO_INFORMATION) {
      return unknown_is_traversable_ ? Traversability::Free : Traversability::Inscribed;
    }
    if (cost >= INSCRIBED_INFLATED_OBSTACLE) {
      return Traversability::Inscribed;
    }
    if (cost >= circumscribed_cost_) {
      return Traversability::Circumscribed;
    }
    return Traversability::Free;
  }

  /// Occupancy for the exact footprint check; inflated values below LETHAL are not obstacles.
  bool is_obstacle(std::uint8_t cost) const noexcept
  {
    if (cost == NO_INFORMATION) {
      return !unknown_is_traversable_;
    }
    return cost >= LETHAL_OBSTACLE;
  }

private:
  std::uint8_t circumscribed_cost_{1};
  bool unknown_is_traversable_{false};
};

static_assert(TraversabilityModel<CostTraversabilityModel, std::uint8_t>);
static_assert(ObstacleModel<CostTraversabilityModel, std::uint8_t>);

}  // namespace eltanin::map

#endif  // ELTANIN__MAP__COST_MODEL_HPP_
