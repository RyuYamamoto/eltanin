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

#ifndef ELTANIN__PLANNER__PLANNER_HPP_
#define ELTANIN__PLANNER__PLANNER_HPP_

#include <eltanin/core/path.hpp>
#include <eltanin/core/traversability.hpp>
#include <eltanin/core/types.hpp>
#include <eltanin/map/cell_map.hpp>
#include <eltanin/map/map_geometry.hpp>
#include <eltanin/planner/traversable_search.hpp>

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

namespace eltanin::planner
{

namespace detail
{

/// One classified cell per byte, holding the Traversability enumerator value.
using TraversabilityGrid = std::vector<std::uint8_t>;

static_assert(static_cast<int>(Traversability::Free) == 0);
static_assert(static_cast<int>(Traversability::Circumscribed) == 1);
static_assert(static_cast<int>(Traversability::Inscribed) == 2);

/// The only place the cell type and traversability model are visible to a planner search core.
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

}  // namespace detail

/// Runtime-polymorphic interface for planners operating on the same classified 2D map.
///
/// The public template method owns map/model adaptation and common endpoint handling. Derived
/// planners only implement their search over a non-template traversability grid.
class Planner
{
public:
  virtual ~Planner() = default;

  Planner(const Planner &) = default;
  Planner & operator=(const Planner &) = default;
  Planner(Planner &&) = default;
  Planner & operator=(Planner &&) = default;

  template <map::CellMap Map, class Model>
    requires TraversabilityModel<Model, typename Map::value_type>
  [[nodiscard]] std::optional<Path> plan(
    const Map & map, const Model & model, const Pose2D & start, const Pose2D & goal) const
  {
    const map::MapGeometry & geometry = map.geometry();
    if (geometry.resolution() <= 0.0 || geometry.cell_count() == 0) {
      return std::nullopt;
    }

    const auto start_index = geometry.world_to_map(start.position);
    const auto goal_index = geometry.world_to_map(goal.position);
    if (!start_index.has_value() || !goal_index.has_value()) {
      return std::nullopt;
    }
    // A blocked goal is reported rather than silently moved away from the requested target.
    if (model.classify(map(goal_index->x, goal_index->y)) != Traversability::Free) {
      return std::nullopt;
    }
    const auto search_start =
      find_nearest_traversable(map, model, *start_index, start_search_radius_cells_);
    if (!search_start.has_value()) {
      return std::nullopt;
    }

    Pose2D effective_start = start;
    if (search_start->x != start_index->x || search_start->y != start_index->y) {
      effective_start.position = geometry.map_to_world(search_start->x, search_start->y);
    }
    return plan_on_grid(
      geometry, detail::build_traversability_grid(map, model), *search_start, *goal_index,
      effective_start, goal);
  }

protected:
  explicit Planner(int start_search_radius_cells)
  : start_search_radius_cells_(start_search_radius_cells)
  {
    if (start_search_radius_cells_ < 0) {
      throw std::invalid_argument("start search radius must be non-negative");
    }
  }

  [[nodiscard]] virtual std::optional<Path> plan_on_grid(
    const map::MapGeometry & geometry, const detail::TraversabilityGrid & grid,
    const map::MapIndex & start_index, const map::MapIndex & goal_index,
    const Pose2D & effective_start, const Pose2D & goal) const = 0;

private:
  int start_search_radius_cells_;
};

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__PLANNER_HPP_
