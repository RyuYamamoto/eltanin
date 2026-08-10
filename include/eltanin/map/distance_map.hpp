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

#ifndef ELTANIN__MAP__DISTANCE_MAP_HPP_
#define ELTANIN__MAP__DISTANCE_MAP_HPP_

#include <eltanin/core/traversability.hpp>
#include <eltanin/map/cell_map.hpp>
#include <eltanin/map/grid_map.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace eltanin::map
{

struct DistanceMapParams
{
  /// Saturation distance [m]; must exceed the circumscribed radius or classify() never says Free.
  double max_distance{1.0};
};

namespace detail
{

/// Sentinel a non-source cell starts from, in squared cells; finite so the envelope stays defined.
double unreachable_squared(int size_x, int size_y) noexcept;

/// In-place squared Euclidean distance transform in cell units (Felzenszwalb & Huttenlocher).
void distance_transform_squared(std::vector<double> & squared, int size_x, int size_y);

}  // namespace detail

/// Exact Euclidean distance [m] to the nearest obstacle cell, saturated at params.max_distance.
template <CellMap Map, class Model>
  requires ObstacleModel<Model, typename Map::value_type>
std::optional<DistanceMap> build_distance_map(
  const Map & map, const Model & model, const DistanceMapParams & params)
{
  const MapGeometry & geometry = map.geometry();
  if (geometry.resolution() <= 0.0 || geometry.cell_count() == 0) {
    return std::nullopt;
  }
  if (!std::isfinite(params.max_distance) || params.max_distance <= 0.0) {
    return std::nullopt;
  }

  const int size_x = geometry.size_x();
  const int size_y = geometry.size_y();
  const double unreachable = detail::unreachable_squared(size_x, size_y);
  std::vector<double> squared(geometry.cell_count(), unreachable);
  for (int my = 0; my < size_y; ++my) {
    for (int mx = 0; mx < size_x; ++mx) {
      // Cells outside the map are not sources: a clipped wall leaves no distance behind it.
      if (model.is_obstacle(map(mx, my))) {
        squared[geometry.index(mx, my)] = 0.0;
      }
    }
  }

  detail::distance_transform_squared(squared, size_x, size_y);

  DistanceMap distances(geometry, static_cast<float>(params.max_distance));
  for (std::size_t index = 0; index < squared.size(); ++index) {
    // A map smaller than max_distance still saturates, so the sentinel is compared, not converted.
    const double metres = (squared[index] >= unreachable)
                            ? params.max_distance
                            : std::sqrt(squared[index]) * geometry.resolution();
    distances[index] = static_cast<float>(std::min(metres, params.max_distance));
  }
  return distances;
}

}  // namespace eltanin::map

#endif  // ELTANIN__MAP__DISTANCE_MAP_HPP_
