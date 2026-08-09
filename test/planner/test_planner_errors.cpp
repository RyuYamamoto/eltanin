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

#include <eltanin/core/path.hpp>
#include <eltanin/map/cost_values.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/planner/astar_planner.hpp>
#include <eltanin/planner/hybrid_astar_planner.hpp>

#include <map/costmap_fixture.hpp>
#include <planner/planner_fixture.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <numbers>

namespace
{

using Eigen::Vector2d;
using eltanin::Pose2D;
using eltanin::map::Costmap;
using eltanin::map::FREE_SPACE;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::MapGeometry;
using eltanin::planner::AStarParams;
using eltanin::planner::HybridAStarParams;
using eltanin::planner::PlannerError;
using eltanin::planner::plan_astar;
using eltanin::planner::plan_hybrid_astar;
using eltanin_test::make_cost_model;

constexpr double RESOLUTION = 0.1;

Costmap open_map(int size_x, int size_y, double resolution = RESOLUTION)
{
  return Costmap(MapGeometry(size_x, size_y, resolution, Vector2d::Zero()), FREE_SPACE);
}

Pose2D at_cell(const Costmap & map, int mx, int my, double yaw = 0.0)
{
  return Pose2D{map.geometry().map_to_world(mx, my), yaw};
}

Costmap split_map()
{
  Costmap map = open_map(12, 8);
  for (int my = 0; my < map.geometry().size_y(); ++my) {
    map(6, my) = LETHAL_OBSTACLE;
  }
  return map;
}

}  // namespace

TEST(PlannerErrors, ReportsAnInvalidMap)
{
  const Costmap empty;
  const Pose2D pose{Vector2d::Zero(), 0.0};

  EXPECT_EQ(plan_astar(empty, make_cost_model(), pose, pose).error(), PlannerError::InvalidMap);
  EXPECT_EQ(
    plan_hybrid_astar(empty, make_cost_model(), pose, pose).error(), PlannerError::InvalidMap);
}

TEST(PlannerErrors, ReportsAStartOutsideTheMap)
{
  const Costmap map = open_map(8, 8);
  const Pose2D outside{Vector2d{-1.0, 0.25}, 0.0};
  const Pose2D goal = at_cell(map, 7, 7);

  EXPECT_EQ(
    plan_astar(map, make_cost_model(), outside, goal).error(), PlannerError::StartOutsideMap);
  EXPECT_EQ(
    plan_hybrid_astar(map, make_cost_model(), outside, goal).error(),
    PlannerError::StartOutsideMap);
}

TEST(PlannerErrors, ReportsANonFiniteStartPositionAsOutsideTheMap)
{
  const Costmap map = open_map(8, 8);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Pose2D start{Vector2d{nan, nan}, 0.0};

  EXPECT_EQ(
    plan_astar(map, make_cost_model(), start, at_cell(map, 7, 7)).error(),
    PlannerError::StartOutsideMap);
}

TEST(PlannerErrors, ReportsAGoalOutsideTheMap)
{
  const Costmap map = open_map(8, 8);
  const Pose2D start = at_cell(map, 0, 0);
  const Pose2D outside{Vector2d{0.25, 9.0}, 0.0};

  EXPECT_EQ(
    plan_astar(map, make_cost_model(), start, outside).error(), PlannerError::GoalOutsideMap);
  EXPECT_EQ(
    plan_hybrid_astar(map, make_cost_model(), start, outside).error(),
    PlannerError::GoalOutsideMap);
}

TEST(PlannerErrors, ReportsANonFiniteYaw)
{
  const Costmap map = open_map(8, 8);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_EQ(
    plan_astar(map, make_cost_model(), at_cell(map, 0, 0, nan), at_cell(map, 7, 7)).error(),
    PlannerError::NonFiniteYaw);
  EXPECT_EQ(
    plan_astar(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 7, 7, inf)).error(),
    PlannerError::NonFiniteYaw);
  EXPECT_EQ(
    plan_hybrid_astar(map, make_cost_model(), at_cell(map, 0, 0, nan), at_cell(map, 7, 7)).error(),
    PlannerError::NonFiniteYaw);
  EXPECT_EQ(
    plan_hybrid_astar(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 7, 7, nan)).error(),
    PlannerError::NonFiniteYaw);
}

TEST(PlannerErrors, ReportsABlockedGoal)
{
  Costmap map = open_map(8, 8);
  map(7, 7) = LETHAL_OBSTACLE;

  EXPECT_EQ(
    plan_astar(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 7, 7)).error(),
    PlannerError::GoalBlocked);
  EXPECT_EQ(
    plan_hybrid_astar(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 7, 7)).error(),
    PlannerError::GoalBlocked);
}

TEST(PlannerErrors, ReportsAFailedStartRescue)
{
  Costmap map = open_map(8, 8);
  map(0, 0) = LETHAL_OBSTACLE;
  AStarParams astar;
  astar.common.start_search_radius_cells = 0;
  HybridAStarParams hybrid;
  hybrid.common.start_search_radius_cells = 0;

  EXPECT_EQ(
    plan_astar(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 7, 7), astar).error(),
    PlannerError::StartRescueFailed);
  EXPECT_EQ(
    plan_hybrid_astar(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 7, 7), hybrid)
      .error(),
    PlannerError::StartRescueFailed);
}

TEST(PlannerErrors, ReportsAnUnreachableGoal)
{
  const Costmap map = split_map();

  EXPECT_EQ(
    plan_astar(map, make_cost_model(), at_cell(map, 1, 4), at_cell(map, 10, 4)).error(),
    PlannerError::Unreachable);
  EXPECT_EQ(
    plan_hybrid_astar(map, make_cost_model(), at_cell(map, 1, 4), at_cell(map, 10, 4)).error(),
    PlannerError::Unreachable);
}

TEST(PlannerErrors, DistinguishesTheExpansionLimitFromUnreachability)
{
  // A gap at the top keeps the goal reachable, so the search stops on the limit, not on exhaustion.
  Costmap map = open_map(36, 30);
  for (int my = 0; my < 21; ++my) {
    map(18, my) = LETHAL_OBSTACLE;
  }
  const Pose2D start = at_cell(map, 5, 10);
  const Pose2D goal = at_cell(map, 30, 10);
  HybridAStarParams params;
  params.max_expansions = 1;

  EXPECT_EQ(
    plan_hybrid_astar(map, make_cost_model(), start, goal, params).error(),
    PlannerError::ExpansionLimitReached);
  EXPECT_TRUE(plan_hybrid_astar(map, make_cost_model(), start, goal).has_value());
}

TEST(PlannerErrors, ASplitMapIsUnreachableRatherThanCutOffByTheLimit)
{
  // The distance-over-cells heuristic marks the far side unreachable, so no expansion is wasted.
  const Costmap map = split_map();
  HybridAStarParams params;
  params.max_expansions = 1;

  EXPECT_EQ(
    plan_hybrid_astar(map, make_cost_model(), at_cell(map, 1, 4), at_cell(map, 10, 4), params)
      .error(),
    PlannerError::Unreachable);
}

TEST(PlannerErrors, ReportsAStateSpaceThatExceedsTheMemoryBudget)
{
  const Costmap map = open_map(64, 64);
  HybridAStarParams params;
  params.max_state_memory_bytes = 1024;

  const auto result =
    plan_hybrid_astar(map, make_cost_model(), at_cell(map, 1, 1), at_cell(map, 60, 60), params);

  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), PlannerError::StateSpaceTooLarge);
}

TEST(PlannerErrors, ReportsAMotionStepThatCannotLeaveTheCurrentCell)
{
  const Costmap map = open_map(40, 40);
  const Pose2D start = at_cell(map, 4, 20);
  const Pose2D goal = at_cell(map, 34, 20);

  for (const double factor : {0.2, 0.5, 1.0}) {
    HybridAStarParams params;
    params.motion_step = factor * RESOLUTION;
    const auto result = plan_hybrid_astar(map, make_cost_model(), start, goal, params);
    EXPECT_EQ(result.error(), PlannerError::ParamsIncompatibleWithMap) << "factor " << factor;
  }
}

TEST(PlannerErrors, SucceedsForEveryMotionStepAtOrAboveTheCellDiagonal)
{
  const Costmap map = open_map(40, 40);
  const Pose2D start = at_cell(map, 4, 20);
  const Pose2D goal = at_cell(map, 34, 20);

  HybridAStarParams defaults;
  EXPECT_TRUE(plan_hybrid_astar(map, make_cost_model(), start, goal, defaults).has_value());

  for (const double factor : {std::numbers::sqrt2, 2.0, 3.0}) {
    HybridAStarParams params;
    params.motion_step = factor * RESOLUTION;
    const auto result = plan_hybrid_astar(map, make_cost_model(), start, goal, params);
    EXPECT_TRUE(result.has_value())
      << "factor " << factor << " failed with " << eltanin::planner::to_string(result.error());
  }
}

TEST(PlannerErrors, ASuccessfulResultCarriesNoError)
{
  const Costmap map = open_map(8, 8);
  const auto result = plan_astar(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 7, 7));

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.error(), PlannerError::None);
  EXPECT_EQ(result.path()->size(), result->size());
}

namespace
{

/// Poses whose cell sits in the circumscribed band; how much of the band a route actually uses.
int band_crossings(const eltanin::Path & path, const eltanin::map::Costmap & map)
{
  int count = 0;
  for (const Pose2D & pose : path) {
    const auto index = map.geometry().world_to_map(pose.position);
    if (index.has_value() && map(index->x, index->y) == eltanin_test::CIRCUMSCRIBED_BAND_COST) {
      ++count;
    }
  }
  return count;
}

}  // namespace

TEST(NarrowPassage, TheBandPenaltySendsTheRelaxedPassToTheThinnestCrossing)
{
  // The band cuts the map in two, so every route is a relaxed one; only its thickness varies.
  eltanin::map::Costmap map(
    MapGeometry(31, 21, 0.1, Eigen::Vector2d::Zero()), eltanin::map::FREE_SPACE);
  for (int my = 0; my <= 10; ++my) {
    for (int mx = 12; mx <= 18; ++mx) {
      map(mx, my) = eltanin_test::CIRCUMSCRIBED_BAND_COST;
    }
  }
  for (int my = 11; my < 21; ++my) {
    map(15, my) = eltanin_test::CIRCUMSCRIBED_BAND_COST;
  }
  const Pose2D start{map.geometry().map_to_world(2, 5), 0.0};
  const Pose2D goal{map.geometry().map_to_world(28, 5), 0.0};

  auto cheap = eltanin_test::raw_astar_params();
  cheap.common.traversability_fallback.enabled = true;
  cheap.common.traversability_fallback.penalty = 0.0;
  auto dear = cheap;
  dear.common.traversability_fallback.penalty = 20.0;

  const auto through = eltanin::planner::plan_astar(map, make_cost_model(), start, goal, cheap);
  const auto around = eltanin::planner::plan_astar(map, make_cost_model(), start, goal, dear);

  ASSERT_TRUE(through.has_value());
  ASSERT_TRUE(around.has_value());
  EXPECT_TRUE(through.relaxed());
  EXPECT_TRUE(around.relaxed());
  // Charging for the band buys a detour to where it is one cell thick instead of seven.
  EXPECT_NEAR(eltanin::path_length(*through), 2.6, 1e-9);
  EXPECT_GT(eltanin::path_length(*around), eltanin::path_length(*through) + 0.3);
  EXPECT_LT(band_crossings(*around, map), band_crossings(*through, map));
}

TEST(NarrowPassage, ARelaxedPassPaysThePenaltyForCrossingTheBand)
{
  // The band walls the map off completely, so only the relaxed pass can get across it.
  eltanin::map::Costmap map(
    MapGeometry(31, 11, 0.1, Eigen::Vector2d::Zero()), eltanin::map::FREE_SPACE);
  for (int my = 0; my < 11; ++my) {
    map(15, my) = eltanin_test::CIRCUMSCRIBED_BAND_COST;
  }
  const Pose2D start{map.geometry().map_to_world(2, 5), 0.0};
  const Pose2D goal{map.geometry().map_to_world(28, 5), 0.0};

  auto params = eltanin_test::raw_astar_params();
  params.common.traversability_fallback.enabled = true;
  const auto path = eltanin::planner::plan_astar(map, make_cost_model(), start, goal, params);

  ASSERT_TRUE(path.has_value());
  EXPECT_TRUE(path.relaxed());
  // Crossing costs the band penalty, so the reported length is still the plain geometric one.
  EXPECT_NEAR(eltanin::path_length(*path), 2.6, 1e-9);
}

TEST(NarrowPassage, BothPlannersGetThroughADoorwayInflationHasClosed)
{
  // A gap the inflation band fills completely: no Free cell links the two rooms.
  Costmap map = open_map(60, 40);
  for (int my = 0; my < 40; ++my) {
    if (my >= 18 && my <= 21) {
      continue;
    }
    for (int mx = 28; mx <= 31; ++mx) {
      map(mx, my) = LETHAL_OBSTACLE;
    }
  }
  for (int my = 18; my <= 21; ++my) {
    for (int mx = 28; mx <= 31; ++mx) {
      map(mx, my) = eltanin_test::CIRCUMSCRIBED_BAND_COST;
    }
  }
  const Pose2D start = at_cell(map, 5, 20);
  const Pose2D goal = at_cell(map, 55, 20);

  AStarParams relaxed_astar;
  relaxed_astar.common.traversability_fallback.enabled = true;
  HybridAStarParams relaxed_hybrid;
  relaxed_hybrid.common.traversability_fallback.enabled = true;

  const auto astar = plan_astar(map, make_cost_model(), start, goal, relaxed_astar);
  const auto hybrid = plan_hybrid_astar(map, make_cost_model(), start, goal, relaxed_hybrid);

  ASSERT_TRUE(astar.has_value());
  ASSERT_TRUE(hybrid.has_value());
  EXPECT_TRUE(astar.relaxed());
  EXPECT_TRUE(hybrid.relaxed());

  // The default is strict, so nothing gets through unless the caller asked for the fallback.
  EXPECT_EQ(
    plan_astar(map, make_cost_model(), start, goal).error(), PlannerError::Unreachable);
  EXPECT_EQ(
    plan_hybrid_astar(map, make_cost_model(), start, goal).error(), PlannerError::Unreachable);
}
