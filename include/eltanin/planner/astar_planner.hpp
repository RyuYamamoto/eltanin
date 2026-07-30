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

#ifndef ELTANIN__PLANNER__ASTAR_PLANNER_HPP_
#define ELTANIN__PLANNER__ASTAR_PLANNER_HPP_

#include <eltanin/core/path.hpp>
#include <eltanin/core/traversability.hpp>
#include <eltanin/core/types.hpp>
#include <eltanin/map/map_geometry.hpp>
#include <eltanin/map/cell_map.hpp>
#include <eltanin/planner/traversable_search.hpp>

#include <cassert>
#include <cstdint>
#include <optional>
#include <vector>

namespace eltanin::planner
{

struct AStarParams
{
  /// Chebyshev radius used to nudge a blocked start onto a free cell; 0 disables the rescue.
  int start_search_radius_cells{8};
};

namespace detail
{

/// One classified cell per byte, holding the Traversability enumerator value.
using TraversabilityGrid = std::vector<std::uint8_t>;

static_assert(static_cast<int>(Traversability::Free) == 0);
static_assert(static_cast<int>(Traversability::Circumscribed) == 1);
static_assert(static_cast<int>(Traversability::Inscribed) == 2);

/// The only place the cell type and the traversability model are visible to the search.
template <map::CellMap Map, class Model>
  requires TraversabilityModel<Model, typename Map::value_type>
TraversabilityGrid build_traversability_grid(const Map & map, const Model & model)
{
  const map::MapGeometry & geometry = map.geometry();
  TraversabilityGrid grid(geometry.cell_count(), 0);
  for (int my = 0; my < geometry.size_y(); ++my) {
    for (int mx = 0; mx < geometry.size_x(); ++mx) {
      grid[geometry.index(mx, my)] = static_cast<std::uint8_t>(model.classify(map(mx, my)));
    }
  }
  return grid;
}

/// Non-template search core; sees neither the cell type nor the traversability model.
std::optional<Path> plan_on_grid(
  const map::MapGeometry & geometry, const TraversabilityGrid & grid, const map::MapIndex & start,
  const map::MapIndex & goal, double goal_yaw);

}  // namespace detail

/// Eight-connected A* over the cells classified Traversability::Free; nullopt when unreachable.
template <map::CellMap Map, class Model>
  requires TraversabilityModel<Model, typename Map::value_type>
std::optional<Path> plan(
  const Map & map, const Model & model, const Pose2D & start, const Pose2D & goal,
  const AStarParams & params = {})
{
  const map::MapGeometry & geometry = map.geometry();
  assert(geometry.resolution() > 0.0);
  assert(geometry.cell_count() > 0);
  assert(params.start_search_radius_cells >= 0);

  const auto start_index = geometry.world_to_map(start.position);
  const auto goal_index = geometry.world_to_map(goal.position);
  if (!start_index.has_value() || !goal_index.has_value()) {
    return std::nullopt;
  }
  // The goal is what the caller asked for, so a blocked goal is reported rather than moved.
  if (model.classify(map(goal_index->x, goal_index->y)) != Traversability::Free) {
    return std::nullopt;
  }
  const auto search_start =
    find_nearest_traversable(map, model, *start_index, params.start_search_radius_cells);
  if (!search_start.has_value()) {
    return std::nullopt;
  }

  return detail::plan_on_grid(
    geometry, detail::build_traversability_grid(map, model), *search_start, *goal_index, goal.yaw);
}

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__ASTAR_PLANNER_HPP_
