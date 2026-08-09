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
#include <eltanin/planner/plan_query.hpp>
#include <eltanin/planner/plan_result.hpp>
#include <eltanin/planner/traversability_view.hpp>
#include <eltanin/planner/traversable_search.hpp>

#include <cmath>
#include <stdexcept>

namespace eltanin::planner
{

/// Second pass for a start and goal the Free cells leave disconnected, such as across a narrow door.
struct NarrowPassageFallback
{
  /// Retry with Traversability::Circumscribed opened up when the Free-only pass finds nothing.
  bool enabled{true};
  /// What a metre of that band costs the retry, as a fraction of the distance travelled.
  double penalty{4.0};
};

/// Parameters every planner shares, whatever its search is.
struct PlannerParams
{
  /// Chebyshev radius used to nudge a blocked start onto a free cell; 0 disables the rescue.
  int start_search_radius_cells{8};
  NarrowPassageFallback narrow_passage{};
};

namespace detail
{

/// The failures that mean the Free-only cell set was too small, and therefore only those.
constexpr bool is_relaxable(PlannerError error) noexcept
{
  return error == PlannerError::Unreachable || error == PlannerError::GoalBlocked ||
         error == PlannerError::StartRescueFailed;
}

}  // namespace detail

/// Runtime-polymorphic planner interface; the base owns map adaptation and endpoint handling.
class Planner
{
public:
  virtual ~Planner() = default;

  /// Returns a directly followable path, or the reason there is none.
  template <map::CellMap Map, class Model>
    requires TraversabilityModel<Model, typename Map::value_type>
  [[nodiscard]] PlanResult plan(
    const Map & map, const Model & model, const Pose2D & start, const Pose2D & goal) const
  {
    const map::MapGeometry & geometry = map.geometry();
    if (geometry.resolution() <= 0.0 || geometry.cell_count() == 0) {
      return PlanResult{PlannerError::InvalidMap};
    }

    const auto start_index = geometry.world_to_map(start.position);
    if (!start_index.has_value()) {
      return PlanResult{PlannerError::StartOutsideMap};
    }
    const auto goal_index = geometry.world_to_map(goal.position);
    if (!goal_index.has_value()) {
      return PlanResult{PlannerError::GoalOutsideMap};
    }
    if (!std::isfinite(start.yaw) || !std::isfinite(goal.yaw)) {
      return PlanResult{PlannerError::NonFiniteYaw};
    }

    const detail::TraversabilityGrid grid = detail::build_traversability_grid(map, model);
    PlanResult strict =
      plan_one_pass(map, model, grid, *start_index, *goal_index, start, goal, {});
    if (
      strict.has_value() || !params_.narrow_passage.enabled ||
      !detail::is_relaxable(strict.error())) {
      return strict;
    }
    // Nothing else changes: the corridor the robot cannot fit through is what was in the way.
    PlanResult relaxed = plan_one_pass(
      map, model, grid, *start_index, *goal_index, start, goal,
      TraversabilityPolicy{Traversability::Circumscribed, params_.narrow_passage.penalty});
    if (relaxed.has_value()) {
      relaxed.mark_narrow_passage();
    }
    return relaxed;
  }

protected:
  /// Throws std::invalid_argument when the radius is negative or the fallback penalty is invalid.
  explicit Planner(const PlannerParams & params) : params_(params)
  {
    if (params_.start_search_radius_cells < 0) {
      throw std::invalid_argument("start search radius must be non-negative");
    }
    const double penalty = params_.narrow_passage.penalty;
    if (!std::isfinite(penalty) || penalty < 0.0) {
      throw std::invalid_argument("narrow passage penalty must be finite and non-negative");
    }
  }

  Planner(const Planner &) = default;
  Planner & operator=(const Planner &) = default;
  Planner(Planner &&) = default;
  Planner & operator=(Planner &&) = default;

  [[nodiscard]] virtual PlanResult plan_on_grid(const PlanQuery & query) const = 0;

private:
  /// One pass; the endpoints are judged by the same policy the search itself runs under.
  template <map::CellMap Map, class Model>
    requires TraversabilityModel<Model, typename Map::value_type>
  [[nodiscard]] PlanResult plan_one_pass(
    const Map & map, const Model & model, const detail::TraversabilityGrid & grid,
    const map::MapIndex & start_index, const map::MapIndex & goal_index, const Pose2D & start,
    const Pose2D & goal, const TraversabilityPolicy & policy) const
  {
    const map::MapGeometry & geometry = map.geometry();
    // A blocked goal is reported rather than silently moved away from the requested target.
    if (model.classify(map(goal_index.x, goal_index.y)) > policy.limit) {
      return PlanResult{PlannerError::GoalBlocked};
    }
    const auto search_start = find_nearest_traversable(
      map, model, start_index, params_.start_search_radius_cells, policy.limit);
    if (!search_start.has_value()) {
      return PlanResult{PlannerError::StartRescueFailed};
    }

    Pose2D effective_start = start;
    if (search_start->x != start_index.x || search_start->y != start_index.y) {
      effective_start.position = geometry.map_to_world(search_start->x, search_start->y);
    }
    return plan_on_grid(PlanQuery{
      TraversabilityView{geometry, grid, policy}, *search_start, goal_index, effective_start,
      goal});
  }

  PlannerParams params_;
};

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__PLANNER_HPP_
