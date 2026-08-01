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

#ifndef ELTANIN__PLANNER__HYBRID_ASTAR_PLANNER_HPP_
#define ELTANIN__PLANNER__HYBRID_ASTAR_PLANNER_HPP_

#include <eltanin/planner/planner.hpp>

#include <cstddef>
#include <optional>

namespace eltanin::planner
{

struct HybridAStarParams
{
  /// Chebyshev radius used to nudge a blocked start onto a free cell; 0 disables the rescue.
  int start_search_radius_cells{8};
  int heading_bins{72};
  double minimum_turning_radius{0.4};
  /// Length of one motion primitive [m]; 0 selects one map cell.
  double motion_step{0.0};
  /// Sampling interval used along each primitive [m]; 0 selects half a map cell.
  double collision_check_step{0.0};
  /// Dubins connection is attempted inside this Euclidean distance from the goal [m].
  double dubins_expansion_distance{1.0};
  double steering_penalty{0.05};
  double steering_change_penalty{0.10};
  /// 0 allows the complete discretized state space to be expanded.
  std::size_t max_expansions{0};
};

/// Forward-only Hybrid A* over (x, y, yaw), using constant-curvature motion primitives.
///
/// Traversability is checked at the vehicle reference point. Callers that require footprint
/// clearance must provide an appropriately inflated traversability map.
class HybridAStarPlanner final : public Planner
{
public:
  /// Throws std::invalid_argument when params are outside their documented range.
  explicit HybridAStarPlanner(const HybridAStarParams & params = {});

private:
  [[nodiscard]] std::optional<Path> plan_on_grid(
    const map::MapGeometry & geometry, const detail::TraversabilityGrid & grid,
    const map::MapIndex & start_index, const map::MapIndex & goal_index,
    const Pose2D & effective_start, const Pose2D & goal) const override;

  HybridAStarParams params_;
};

template <map::CellMap Map, class Model>
  requires TraversabilityModel<Model, typename Map::value_type>
std::optional<Path> plan_hybrid_astar(
  const Map & map, const Model & model, const Pose2D & start, const Pose2D & goal,
  const HybridAStarParams & params = {})
{
  return HybridAStarPlanner(params).plan(map, model, start, goal);
}

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__HYBRID_ASTAR_PLANNER_HPP_
