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

#ifndef ELTANIN__PLANNER__CLEARANCE_MAP_HPP_
#define ELTANIN__PLANNER__CLEARANCE_MAP_HPP_

#include <eltanin/core/types.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/planner/traversability_view.hpp>

#include <optional>
#include <span>

namespace eltanin::planner
{

/// Distance [m] from every cell to the nearest obstacle, which is what "how much room" means here.
using ClearanceMap = map::DistanceMap;

namespace detail
{

/// Exact Euclidean distance transform of the classified grid over the whole map.
ClearanceMap build_clearance_map(const TraversabilityView & grid);

/// The same, over a window around `positions` only; distances saturate at `reach`.
std::optional<ClearanceMap> build_clearance_map(
  const TraversabilityView & grid, std::span<const Eigen::Vector2d> positions, double reach);

}  // namespace detail

/// Clearance at a world point, interpolated between cell centers so the field is continuous.
[[nodiscard]] double clearance_at(
  const ClearanceMap & field, const Eigen::Vector2d & world, double outside) noexcept;

/// Gradient of that interpolation; it points away from the nearest obstacle, or is 0 where flat.
[[nodiscard]] Eigen::Vector2d clearance_gradient(
  const ClearanceMap & field, const Eigen::Vector2d & world) noexcept;

/// Keeps a search off the walls by charging extra for travel that has little room either side.
struct ClearanceCost
{
  /// Extra cost as a fraction of the distance travelled, at zero clearance; 0 disables the term.
  double penalty{0.0};
  /// Clearance [m] at and above which nothing extra is charged.
  double distance{0.5};
};

/// Fraction to add to a step of travel, falling linearly from `penalty` at a wall to 0 at `distance`.
[[nodiscard]] inline double clearance_penalty(
  const ClearanceCost & cost, double clearance) noexcept
{
  if (cost.penalty <= 0.0 || cost.distance <= 0.0 || clearance >= cost.distance) {
    return 0.0;
  }
  return cost.penalty * (1.0 - clearance / cost.distance);
}

namespace detail
{

/// Throws std::invalid_argument for a negative penalty or a non-positive falloff distance.
void validate_clearance_cost(const ClearanceCost & cost);

}  // namespace detail

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__CLEARANCE_MAP_HPP_
