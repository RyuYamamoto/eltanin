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

#ifndef ELTANIN__PLANNER__PATH_SMOOTHER_HPP_
#define ELTANIN__PLANNER__PATH_SMOOTHER_HPP_

#include <eltanin/core/path.hpp>
#include <eltanin/core/traversability.hpp>
#include <eltanin/core/types.hpp>
#include <eltanin/map/map_geometry.hpp>
#include <eltanin/map/cell_map.hpp>

#include <cstddef>

namespace eltanin::planner
{

struct SmootherParams
{
  /// Pull back towards the input path.
  double weight_data{0.5};
  /// Pull towards the average of the two neighbours.
  double weight_smooth{0.3};
  /// Stops once the total displacement of one sweep falls below this [m].
  double tolerance{1e-4};
  int max_iterations{100};
};

namespace detail
{

/// Sets pose[i].yaw to the tangent towards pose[i+1] for i = 0..n-2; pose[n-1].yaw is left alone.
void assign_tangent_yaw(Path & path);

/// Throws std::invalid_argument for invalid weights; convergence requires weight_data + 4 * ws < 2.
void validate_smoother_params(const SmootherParams & params);

}  // namespace detail

/// Iterative smoothing with a per-point collision check; the two end points never move.
/// Throws std::invalid_argument when params are invalid.
template <map::CellMap Map, class Model>
  requires TraversabilityModel<Model, typename Map::value_type>
Path smooth(
  const Path & path, const Map & map, const Model & model, const SmootherParams & params = {})
{
  detail::validate_smoother_params(params);
  if (path.size() <= 1) {
    return path;
  }

  const map::MapGeometry & geometry = map.geometry();
  const auto is_traversable = [&](const Eigen::Vector2d & point) {
    const auto index = geometry.world_to_map(point);
    return index.has_value() && model.classify(map(index->x, index->y)) == Traversability::Free;
  };

  Path smoothed = path;
  const std::size_t n = smoothed.size();
  if (n >= 3) {
    for (int iteration = 0; iteration < params.max_iterations; ++iteration) {
      double displacement = 0.0;
      for (std::size_t i = 1; i + 1 < n; ++i) {
        const Eigen::Vector2d candidate =
          smoothed[i].position + params.weight_data * (path[i].position - smoothed[i].position) +
          params.weight_smooth * (smoothed[i - 1].position + smoothed[i + 1].position -
                                  2.0 * smoothed[i].position);
        // Rejected for this sweep only; a neighbour moving later may make the point movable.
        if (!is_traversable(candidate)) {
          continue;
        }
        displacement += (candidate - smoothed[i].position).norm();
        smoothed[i].position = candidate;
      }
      if (displacement < params.tolerance) {
        break;
      }
    }
  }
  detail::assign_tangent_yaw(smoothed);
  return smoothed;
}

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__PATH_SMOOTHER_HPP_
