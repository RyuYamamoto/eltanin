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

namespace eltanin::planner
{

/// Covers every reachable case measured in docs/planner-design.md §13.7 and bounds one call.
inline constexpr std::size_t DEFAULT_MAX_EXPANSIONS = 4000000;

/// 256 MiB of search state, which covers roughly a 30 m square map at 0.05 m and 72 bins.
inline constexpr std::size_t DEFAULT_MAX_STATE_MEMORY_BYTES = 256UL * 1024UL * 1024UL;

/// What the vehicle can actually do, and therefore what the returned path may contain.
enum class MotionModel
{
  /// Forward arcs at or above the minimum turning radius; a car cannot do better.
  Dubins,
  /// Adds turning on the spot, which is what a differential drive does.
  Differential,
  /// Adds driving in reverse, so the analytic connection becomes a Reeds-Shepp path.
  ReedsShepp,
};

struct HybridAStarParams
{
  PlannerParams common{};
  /// Which primitives the search may use, and which the output is allowed to contain.
  MotionModel motion_model{MotionModel::Dubins};
  int heading_bins{72};
  double minimum_turning_radius{0.4};
  /// Length of one motion primitive [m]; 0 derives a step that always changes the discrete state.
  double motion_step{0.0};
  /// Collision check interval along each primitive [m]; 0 selects half a map cell.
  double collision_check_step{0.0};
  /// Dubins connection is attempted at every expanded node inside this distance from the goal [m].
  double dubins_expansion_distance{1.0};
  /// Beyond that distance it is attempted every (cells to goal / this) nodes; larger tries more.
  double analytic_expansion_ratio{1.0};
  /// Weight on the over-cells distance heuristic; above ~0.9 it goes greedy and the path weaves.
  double heuristic_weight{0.8};
  double steering_penalty{0.05};
  double steering_change_penalty{0.10};
  /// Multiplies the cost of driving in reverse; 1 makes it as cheap as going forward.
  double reverse_penalty{2.0};
  /// Charged once per switch between forward and reverse, as a multiple of one motion step.
  double direction_change_penalty{2.0};
  /// 0 allows the complete discretized state space to be expanded.
  std::size_t max_expansions{DEFAULT_MAX_EXPANSIONS};
  /// Upper bound on the search state arrays; bigger problems fail with StateSpaceTooLarge.
  std::size_t max_state_memory_bytes{DEFAULT_MAX_STATE_MEMORY_BYTES};
};

/// Forward-only Hybrid A* over (x, y, yaw); collisions are checked at the vehicle reference point.
class HybridAStarPlanner final : public Planner
{
public:
  /// Throws std::invalid_argument when params are outside their documented range.
  explicit HybridAStarPlanner(const HybridAStarParams & params = {});

private:
  [[nodiscard]] PlanResult plan_on_grid(const PlanQuery & query) const override;

  HybridAStarParams params_;
};

/// Convenience wrapper around HybridAStarPlanner.
template <map::CellMap Map, class Model>
  requires TraversabilityModel<Model, typename Map::value_type>
PlanResult plan_hybrid_astar(
  const Map & map, const Model & model, const Pose2D & start, const Pose2D & goal,
  const HybridAStarParams & params = {})
{
  return HybridAStarPlanner(params).plan(map, model, start, goal);
}

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__HYBRID_ASTAR_PLANNER_HPP_
