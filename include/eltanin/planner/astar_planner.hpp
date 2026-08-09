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

#include <eltanin/planner/clearance_map.hpp>
#include <eltanin/planner/path_smoother.hpp>
#include <eltanin/planner/planner.hpp>

#include <optional>

namespace eltanin::planner
{

struct AStarParams
{
  PlannerParams common{};
  /// nullopt returns the raw cell-center A* path; a value smooths it before returning.
  std::optional<SmootherParams> smoother{SmootherParams{}};
  /// Off by default: shaping the search costs a full-map distance field (docs §13.20).
  ClearanceCost clearance{0.0, 0.5};
};

/// Eight-connected A* over the cells classified Traversability::Free.
class AStarPlanner final : public Planner
{
public:
  /// Throws std::invalid_argument when the common or smoother parameters are out of range.
  explicit AStarPlanner(const AStarParams & params = {});

private:
  [[nodiscard]] PlanResult plan_on_grid(const PlanQuery & query) const override;

  AStarParams params_;
};

/// Convenience wrapper around AStarPlanner; smooths the result unless params.smoother is nullopt.
template <map::CellMap Map, class Model>
  requires TraversabilityModel<Model, typename Map::value_type>
PlanResult plan_astar(
  const Map & map, const Model & model, const Pose2D & start, const Pose2D & goal,
  const AStarParams & params = {})
{
  return AStarPlanner(params).plan(map, model, start, goal);
}

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__ASTAR_PLANNER_HPP_
