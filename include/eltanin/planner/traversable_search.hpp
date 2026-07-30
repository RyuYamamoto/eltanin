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

#ifndef ELTANIN__PLANNER__TRAVERSABLE_SEARCH_HPP_
#define ELTANIN__PLANNER__TRAVERSABLE_SEARCH_HPP_

#include <eltanin/core/traversability.hpp>
#include <eltanin/map/map_geometry.hpp>
#include <eltanin/map/cell_map.hpp>

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <limits>
#include <optional>

namespace eltanin::planner
{

namespace detail
{

/// Saturates to int so that in_bounds() can be evaluated without signed overflow.
inline int saturate_to_int(std::int64_t value) noexcept
{
  constexpr std::int64_t LOWEST = std::numeric_limits<int>::min();
  constexpr std::int64_t HIGHEST = std::numeric_limits<int>::max();
  return static_cast<int>(std::clamp(value, LOWEST, HIGHEST));
}

}  // namespace detail

/// Euclidean-nearest Traversability::Free cell within a Chebyshev radius of `from`.
template <map::CellMap Map, class Model>
  requires TraversabilityModel<Model, typename Map::value_type>
std::optional<map::MapIndex> find_nearest_traversable(
  const Map & map, const Model & model, const map::MapIndex & from, int max_radius_cells)
{
  assert(max_radius_cells >= 0);

  const map::MapGeometry & geometry = map.geometry();
  std::optional<map::MapIndex> best;
  std::int64_t best_distance_squared = std::numeric_limits<std::int64_t>::max();

  for (std::int64_t r = 0; r <= max_radius_cells; ++r) {
    // Every cell on ring r is at Euclidean distance >= r, so no later ring can improve.
    if (best.has_value() && r * r > best_distance_squared) {
      break;
    }
    for (std::int64_t dy = -r; dy <= r; ++dy) {
      for (std::int64_t dx = -r; dx <= r; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != r) {
          continue;
        }
        const int mx = detail::saturate_to_int(static_cast<std::int64_t>(from.x) + dx);
        const int my = detail::saturate_to_int(static_cast<std::int64_t>(from.y) + dy);
        if (!geometry.in_bounds(mx, my)) {
          continue;
        }
        if (model.classify(map(mx, my)) != Traversability::Free) {
          continue;
        }
        const std::int64_t distance_squared = dx * dx + dy * dy;
        if (distance_squared < best_distance_squared) {
          best_distance_squared = distance_squared;
          best = map::MapIndex{mx, my};
        }
      }
    }
  }
  return best;
}

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__TRAVERSABLE_SEARCH_HPP_
