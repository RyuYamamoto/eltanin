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

#include <eltanin/core/polygon.hpp>
#include <eltanin/planner/clearance_map.hpp>
#include <eltanin/planner/planner.hpp>

#include <cstddef>

namespace eltanin::planner
{

/// Covers every reachable case measured in docs/planner-design.md §13.7 and bounds one call.
inline constexpr std::size_t DEFAULT_MAX_EXPANSIONS = 4000000;

/// 256 MiB of search state, which covers roughly a 30 m square map at 0.05 m and 72 bins.
inline constexpr std::size_t DEFAULT_MAX_STATE_MEMORY_BYTES = 256UL * 1024UL * 1024UL;

/// Second attempt with less room demanded, for gaps the roomy pass would rather walk around.
struct ClearanceFallback
{
  /// Off by default: a second search costs more than the room it buys (docs 14.9).
  bool enabled{false};
  ClearanceCost clearance{0.5, 0.4};
  /// Retry once the roomy path is longer than the shortest route by more than this fraction.
  double detour_tolerance{0.25};
};

/// What the body can do. The control set and the analytic connection both follow from this.
struct MotionModel
{
  /// Tightest arc the body can drive [m].
  double minimum_turning_radius{0.4};
  /// The body can drive backwards, which also makes the analytic connection a Reeds-Shepp path.
  bool reverse{false};
  /// The body can rotate without translating, as a differential drive does.
  bool turn_in_place{false};

  bool operator==(const MotionModel &) const = default;
};

struct HybridAStarParams
{
  PlannerParams common{};
  MotionModel motion_model{};
  int heading_bins{72};
  /// Length of one motion primitive [m]; 0 derives a step that always changes the discrete state.
  double motion_step{0.0};
  /// Collision check interval along each primitive [m]; 0 selects half a map cell.
  double collision_check_step{0.0};
  /// The analytic expansion is tried at every expanded node inside this distance of the goal [m].
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
  ClearanceCost clearance{0.5, 0.4};
  /// Extra cost per metre of travel through the circumscribed band, once the outline lets it in.
  double circumscribed_penalty{6.0};
  /// Emit the closing turn as its own pose; a follower that cannot execute one wants this off.
  bool emit_goal_rotation{true};
  ClearanceFallback clearance_fallback{};
  /// 0 allows the complete discretized state space to be expanded.
  std::size_t max_expansions{DEFAULT_MAX_EXPANSIONS};
  /// Upper bound on the search state arrays; bigger problems fail with StateSpaceTooLarge.
  std::size_t max_state_memory_bytes{DEFAULT_MAX_STATE_MEMORY_BYTES};
};

/// Hybrid A* over (x, y, yaw); what the body may do comes from MotionModel.
class HybridAStarPlanner final : public Planner
{
public:
  /// Throws std::invalid_argument when params are outside their documented range.
  explicit HybridAStarPlanner(const HybridAStarParams & params = {});

private:
  [[nodiscard]] PlanResult plan_on_grid(const PlanQuery & query) const override;

  /// One outline setting, with the clearance fallback applied on top of it.
  [[nodiscard]] static PlanResult plan_once(
    const PlanQuery & query, const HybridAStarParams & params);

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
