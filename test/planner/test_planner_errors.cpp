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
  const Costmap map = split_map();
  HybridAStarParams params;
  params.max_expansions = 1;

  EXPECT_EQ(
    plan_hybrid_astar(map, make_cost_model(), at_cell(map, 1, 4), at_cell(map, 10, 4), params)
      .error(),
    PlannerError::ExpansionLimitReached);
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
