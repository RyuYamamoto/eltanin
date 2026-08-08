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
#include <eltanin/map/cell_map.hpp>
#include <eltanin/map/map_geometry.hpp>
#include <eltanin/planner/traversability_view.hpp>

namespace eltanin::planner
{

struct SmootherParams
{
  /// Pull back towards the input path; a grid path is the artifact, so this stays weak.
  double weight_data{0.1};
  /// Pull towards the average of the two neighbours.
  double weight_smooth{0.4};
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

/// The smoothing sweep itself; the public smooth() and every planner share this one body.
Path smooth_on_grid(
  const Path & path, const TraversabilityView & grid, const SmootherParams & params);

}  // namespace detail

/// Iterative smoothing with a per-point collision check; end points never move, invalid params throw.
template <map::CellMap Map, class Model>
  requires TraversabilityModel<Model, typename Map::value_type>
Path smooth(
  const Path & path, const Map & map, const Model & model, const SmootherParams & params = {})
{
  detail::validate_smoother_params(params);
  if (path.size() <= 1) {
    return path;
  }
  const detail::TraversabilityGrid grid = detail::build_traversability_grid(map, model);
  return detail::smooth_on_grid(path, TraversabilityView{map.geometry(), grid}, params);
}

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__PATH_SMOOTHER_HPP_
