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
#include <eltanin/planner/clearance_map.hpp>
#include <eltanin/planner/traversability_view.hpp>

namespace eltanin::planner
{

struct SmootherParams
{
  /// Pull back towards the input path; a grid path is the artifact, so this stays weak.
  double weight_data{0.1};
  /// Pull towards the average of the two neighbours.
  double weight_smooth{0.4};
  /// Penalise bending, which is what a staircase of cell centers is made of.
  double weight_curvature{0.01};
  /// Push away from anything closer than obstacle_distance; 0 leaves clearance to the search.
  double weight_obstacle{0.05};
  /// Clearance [m] at and above which the obstacle term is silent; past 0.3 the path starts to weave.
  double obstacle_distance{0.3};
  /// Stops once the total displacement of one sweep falls below this [m].
  double tolerance{1e-4};
  int max_iterations{100};
};

namespace detail
{

/// Sets pose[i].yaw to the tangent towards pose[i+1] for i = 0..n-2; pose[n-1].yaw is left alone.
void assign_tangent_yaw(Path & path);

/// Throws std::invalid_argument unless weight_data + 4 * ws + 16 * wc < 2, which is what converges.
void validate_smoother_params(const SmootherParams & params);

/// The smoothing sweep itself; the public smooth() and every planner share this one body.
Path smooth_on_grid(
  const Path & path, const TraversabilityView & grid, const ClearanceMap & obstacle,
  const SmootherParams & params);

/// The clearance the obstacle term needs to see; 0 when the term is switched off.
[[nodiscard]] double smoother_reach(const SmootherParams & params) noexcept;

}  // namespace detail

/// Iterative smoothing with a per-point check under policy; ends never move, invalid params throw.
template <map::CellMap Map, class Model>
  requires TraversabilityModel<Model, typename Map::value_type>
Path smooth(
  const Path & path, const Map & map, const Model & model, const SmootherParams & params = {},
  const TraversabilityPolicy & policy = {})
{
  detail::validate_smoother_params(params);
  if (path.size() <= 1) {
    return path;
  }
  const detail::TraversabilityGrid grid = detail::build_traversability_grid(map, model);
  const TraversabilityView view{map.geometry(), grid, policy};

  // A window around the path, so the obstacle term costs the path's size and not the map's.
  const double reach = detail::smoother_reach(params);
  std::optional<ClearanceMap> window;
  if (reach > 0.0) {
    std::vector<Eigen::Vector2d> positions;
    positions.reserve(path.size());
    for (const Pose2D & pose : path) {
      positions.push_back(pose.position);
    }
    window = detail::build_clearance_map(view, positions, reach);
  }
  const ClearanceMap room = window.has_value() ? std::move(*window) : ClearanceMap{};
  return detail::smooth_on_grid(path, view, room, params);
}

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__PATH_SMOOTHER_HPP_
