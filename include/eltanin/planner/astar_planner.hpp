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

#include <eltanin/planner/planner.hpp>

#include <optional>

namespace eltanin::planner
{

struct AStarParams
{
  /// Chebyshev radius used to nudge a blocked start onto a free cell; 0 disables the rescue.
  int start_search_radius_cells{8};
};

class AStarPlanner final : public Planner
{
public:
  /// Throws std::invalid_argument when start_search_radius_cells is negative.
  explicit AStarPlanner(const AStarParams & params = {})
  : Planner(params.start_search_radius_cells)
  {
  }

private:
  [[nodiscard]] std::optional<Path> plan_on_grid(
    const map::MapGeometry & geometry, const detail::TraversabilityGrid & grid,
    const map::MapIndex & start_index, const map::MapIndex & goal_index,
    const Pose2D & effective_start, const Pose2D & goal) const override;
};

/// Eight-connected A* over the cells classified Traversability::Free; nullopt when unreachable.
template <map::CellMap Map, class Model>
  requires TraversabilityModel<Model, typename Map::value_type>
std::optional<Path> plan(
  const Map & map, const Model & model, const Pose2D & start, const Pose2D & goal,
  const AStarParams & params = {})
{
  return AStarPlanner(params).plan(map, model, start, goal);
}

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__ASTAR_PLANNER_HPP_
