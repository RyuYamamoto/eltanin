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

#include <map/costmap_fixture.hpp>
#include <planner/planner_fixture.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <vector>

namespace
{

using Eigen::Vector2d;
using eltanin::Pose2D;
using eltanin::path_length;
using eltanin::map::Costmap;
using eltanin::map::CostTraversabilityModel;
using eltanin::map::FREE_SPACE;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::MapGeometry;
using eltanin_test::CIRCUMSCRIBED_BAND_COST;
using eltanin_test::expect_all_free;
using eltanin_test::expect_grid_connected;
using eltanin_test::expect_same_path;
using eltanin_test::make_cost_model;

constexpr double RESOLUTION = 0.1;
constexpr double TOLERANCE = 1e-9;
constexpr double SQRT2 = std::numbers::sqrt2;

Costmap open_map(int size_x, int size_y, double resolution = RESOLUTION,
                 const Vector2d & origin = Vector2d::Zero())
{
  return Costmap(MapGeometry(size_x, size_y, resolution, origin), FREE_SPACE);
}

Pose2D at_cell(const Costmap & map, int mx, int my, double yaw = 0.0)
{
  return Pose2D{map.geometry().map_to_world(mx, my), yaw};
}

/// Two obstacles leave a diagonal gap that only corner cutting could squeeze through.
Costmap diagonal_gap_map()
{
  Costmap map = open_map(4, 3);
  map(1, 1) = LETHAL_OBSTACLE;
  map(2, 0) = LETHAL_OBSTACLE;
  return map;
}

/// A single wall spanning my = 0..2 of column mx = 2, so the path detours over the top.
Costmap wall_map()
{
  Costmap map = open_map(5, 5);
  for (int my = 0; my <= 2; ++my) {
    map(2, my) = LETHAL_OBSTACLE;
  }
  return map;
}

/// The octile expectations in this file describe the raw cell-center path, so smoothing is off.
template <class Map, class Model>
eltanin::planner::PlanResult plan_raw(
  const Map & map, const Model & model, const Pose2D & start, const Pose2D & goal,
  const eltanin::planner::AStarParams & params = eltanin_test::raw_astar_params())
{
  return eltanin::planner::plan_astar(map, model, start, goal, params);
}

}  // namespace

TEST(AStarPlanner, DiagonalAcrossAnOpenGrid)
{
  const Costmap map = open_map(8, 8);
  const auto path = plan_raw(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 7, 7));
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->size(), 8u);
  EXPECT_NEAR(path_length(*path), 7.0 * SQRT2 * RESOLUTION, TOLERANCE);
  expect_grid_connected(*path, RESOLUTION);
  expect_all_free(*path, map, make_cost_model());
}

TEST(AStarPlanner, StraightLineAcrossAnOpenGrid)
{
  const Costmap map = open_map(8, 3);
  const auto path = plan_raw(map, make_cost_model(), at_cell(map, 0, 1), at_cell(map, 7, 1));
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->size(), 8u);
  EXPECT_NEAR(path_length(*path), 7.0 * RESOLUTION, TOLERANCE);
  for (std::size_t i = 0; i < path->size(); ++i) {
    EXPECT_NEAR((*path)[i].position.y(), map.geometry().map_to_world(0, 1).y(), TOLERANCE);
  }
}

TEST(AStarPlanner, DetoursTheDiagonalGapBecauseCornerCuttingIsForbidden)
{
  const Costmap map = diagonal_gap_map();
  const auto path = plan_raw(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 3, 2));
  ASSERT_TRUE(path.has_value());
  // Corner cutting would allow 1 + 2 * sqrt2 = 3.828 cells; the detour costs 5 cells.
  EXPECT_NEAR(path_length(*path), 5.0 * RESOLUTION, TOLERANCE);
  expect_grid_connected(*path, RESOLUTION);
  expect_all_free(*path, map, make_cost_model());
}

TEST(AStarPlanner, DetoursASingleWallWithTheHandComputedLength)
{
  const Costmap map = wall_map();
  const auto path = plan_raw(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 4, 0));
  ASSERT_TRUE(path.has_value());
  // Six orthogonal and two diagonal steps; corner cutting would give 2 + 4 * sqrt2 instead.
  EXPECT_NEAR(path_length(*path), (6.0 + 2.0 * SQRT2) * RESOLUTION, TOLERANCE);
  expect_grid_connected(*path, RESOLUTION);
  expect_all_free(*path, map, make_cost_model());
}

TEST(AStarPlanner, StartEqualsGoalGivesASinglePose)
{
  const Costmap map = open_map(5, 5);
  const auto path = plan_raw(map, make_cost_model(), at_cell(map, 2, 2, 0.5), at_cell(map, 2, 2, 1.25));
  ASSERT_TRUE(path.has_value());
  ASSERT_EQ(path->size(), 1u);
  EXPECT_EQ(path_length(*path), 0.0);
  EXPECT_EQ((*path)[0].yaw, 1.25);
  EXPECT_EQ((*path)[0].position.x(), map.geometry().map_to_world(2, 2).x());
  EXPECT_EQ((*path)[0].position.y(), map.geometry().map_to_world(2, 2).y());
}

TEST(AStarPlanner, AdjacentCellsGiveTwoPoses)
{
  const Costmap map = open_map(5, 5);

  const auto orthogonal = plan_raw(map, make_cost_model(), at_cell(map, 2, 2), at_cell(map, 3, 2));
  ASSERT_TRUE(orthogonal.has_value());
  EXPECT_EQ(orthogonal->size(), 2u);
  EXPECT_NEAR(path_length(*orthogonal), RESOLUTION, TOLERANCE);

  const auto diagonal = plan_raw(map, make_cost_model(), at_cell(map, 2, 2), at_cell(map, 3, 3));
  ASSERT_TRUE(diagonal.has_value());
  EXPECT_EQ(diagonal->size(), 2u);
  EXPECT_NEAR(path_length(*diagonal), SQRT2 * RESOLUTION, TOLERANCE);
}

TEST(AStarPlanner, EndpointsAreCellCentersNotCorners)
{
  const Costmap map = open_map(6, 6, RESOLUTION, Vector2d{-0.35, 0.7});
  const Vector2d start_center = map.geometry().map_to_world(1, 1);
  const Vector2d goal_center = map.geometry().map_to_world(4, 3);
  // Offset inside the cell so that a corner-based conversion would show up here.
  const Pose2D start{start_center + Vector2d{0.03, -0.02}, 0.0};
  const Pose2D goal{goal_center + Vector2d{-0.04, 0.01}, 0.75};

  const auto path = plan_raw(map, make_cost_model(), start, goal);
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ((*path)[0].position.x(), start_center.x());
  EXPECT_EQ((*path)[0].position.y(), start_center.y());
  EXPECT_EQ((*path)[path->size() - 1].position.x(), goal_center.x());
  EXPECT_EQ((*path)[path->size() - 1].position.y(), goal_center.y());
  EXPECT_EQ((*path)[path->size() - 1].yaw, 0.75);
}

TEST(AStarPlanner, LengthScalesWithResolution)
{
  const Costmap coarse = open_map(8, 8, 0.2);
  const Costmap fine = open_map(8, 8, 0.05);
  const auto coarse_path =
    plan_raw(coarse, make_cost_model(), at_cell(coarse, 0, 0), at_cell(coarse, 7, 5));
  const auto fine_path = plan_raw(fine, make_cost_model(), at_cell(fine, 0, 0), at_cell(fine, 7, 5));
  ASSERT_TRUE(coarse_path.has_value());
  ASSERT_TRUE(fine_path.has_value());
  EXPECT_NEAR(path_length(*coarse_path), 4.0 * path_length(*fine_path), TOLERANCE);
}

TEST(AStarPlanner, LengthIsIndependentOfOrigin)
{
  const Costmap at_zero = open_map(8, 8);
  const Costmap shifted = open_map(8, 8, RESOLUTION, Vector2d{-4.25, -13.5});
  const auto zero_path =
    plan_raw(at_zero, make_cost_model(), at_cell(at_zero, 0, 0), at_cell(at_zero, 7, 5));
  const auto shifted_path =
    plan_raw(shifted, make_cost_model(), at_cell(shifted, 0, 0), at_cell(shifted, 7, 5));
  ASSERT_TRUE(zero_path.has_value());
  ASSERT_TRUE(shifted_path.has_value());
  EXPECT_EQ(zero_path->size(), shifted_path->size());
  EXPECT_NEAR(path_length(*zero_path), path_length(*shifted_path), TOLERANCE);
}

TEST(AStarPlanner, IsDeterministic)
{
  const Costmap map = wall_map();
  const auto first = plan_raw(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 4, 0, 0.3));
  const auto second = plan_raw(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 4, 0, 0.3));
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  expect_same_path(*first, *second);
}

TEST(AStarPlanner, YawIsTangentExceptAtTheGoal)
{
  const Costmap map = open_map(6, 3);
  const auto path = plan_raw(map, make_cost_model(), at_cell(map, 0, 1), at_cell(map, 5, 1, -2.0));
  ASSERT_TRUE(path.has_value());
  ASSERT_EQ(path->size(), 6u);
  for (std::size_t i = 0; i + 1 < path->size(); ++i) {
    EXPECT_NEAR((*path)[i].yaw, 0.0, TOLERANCE) << "pose " << i;
  }
  EXPECT_EQ((*path)[5].yaw, -2.0);
}

TEST(AStarPlanner, GoalEnclosedByObstaclesIsUnreachable)
{
  const Costmap map = eltanin_test::make_costmap(
    {
      ".....",
      ".###.",
      ".#.#.",
      ".###.",
      ".....",
    },
    RESOLUTION);
  EXPECT_FALSE(plan_raw(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 2, 2)).has_value());
}

TEST(AStarPlanner, WallSplittingTheMapIsUnreachable)
{
  const Costmap map = eltanin_test::make_costmap(
    {
      "..#..",
      "..#..",
      "..#..",
      "..#..",
      "..#..",
    },
    RESOLUTION);
  EXPECT_FALSE(plan_raw(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 4, 4)).has_value());
}

TEST(AStarPlanner, StartOutsideTheMapIsRejected)
{
  const Costmap map = open_map(5, 5);
  const Pose2D outside{Vector2d{-1.0, 0.25}, 0.0};
  EXPECT_FALSE(plan_raw(map, make_cost_model(), outside, at_cell(map, 4, 4)).has_value());
}

TEST(AStarPlanner, GoalOutsideTheMapIsRejected)
{
  const Costmap map = open_map(5, 5);
  const Pose2D outside{Vector2d{0.25, 5.0}, 0.0};
  EXPECT_FALSE(plan_raw(map, make_cost_model(), at_cell(map, 0, 0), outside).has_value());
}

TEST(AStarPlanner, InscribedGoalIsRejected)
{
  Costmap map = open_map(5, 5);
  map(4, 4) = LETHAL_OBSTACLE;
  EXPECT_FALSE(plan_raw(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 4, 4)).has_value());
}

TEST(AStarPlanner, CircumscribedGoalIsRejected)
{
  Costmap map = open_map(5, 5);
  map(4, 4) = CIRCUMSCRIBED_BAND_COST;
  EXPECT_FALSE(plan_raw(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 4, 4)).has_value());
}

TEST(AStarPlanner, UnknownGoalFollowsTheModelSetting)
{
  Costmap map = open_map(5, 5);
  map(4, 4) = eltanin::map::NO_INFORMATION;
  EXPECT_FALSE(
    plan_raw(map, make_cost_model(false), at_cell(map, 0, 0), at_cell(map, 4, 4)).has_value());

  const auto path = plan_raw(map, make_cost_model(true), at_cell(map, 0, 0), at_cell(map, 4, 4));
  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path_length(*path), 4.0 * SQRT2 * RESOLUTION, TOLERANCE);
}

TEST(AStarPlanner, UnknownCellsOpenUpTheSearchSpace)
{
  const Costmap map = eltanin_test::make_costmap(
    {
      "..?..",
      "..?..",
      "..?..",
      "..?..",
      "..?..",
    },
    RESOLUTION);
  EXPECT_FALSE(
    plan_raw(map, make_cost_model(false), at_cell(map, 0, 0), at_cell(map, 4, 4)).has_value());
  EXPECT_TRUE(plan_raw(map, make_cost_model(true), at_cell(map, 0, 0), at_cell(map, 4, 4)).has_value());
}

TEST(AStarPlanner, BlockedStartIsRescuedToTheNearestFreeCell)
{
  const Costmap map = eltanin_test::make_costmap(
    {
      ".....",
      ".....",
      "###..",
      "###..",
      "###..",
    },
    RESOLUTION);
  const Pose2D start = at_cell(map, 1, 1);
  const Pose2D goal = at_cell(map, 4, 4);

  EXPECT_FALSE(
    plan_raw(map, make_cost_model(), start, goal, eltanin_test::raw_astar_params(0)).has_value());
  EXPECT_FALSE(
    plan_raw(map, make_cost_model(), start, goal, eltanin_test::raw_astar_params(1)).has_value());

  const auto path =
    plan_raw(map, make_cost_model(), start, goal, eltanin_test::raw_astar_params(2));
  ASSERT_TRUE(path.has_value());
  const Vector2d rescued = map.geometry().map_to_world(3, 1);
  EXPECT_EQ((*path)[0].position.x(), rescued.x());
  EXPECT_EQ((*path)[0].position.y(), rescued.y());
  expect_all_free(*path, map, make_cost_model());
}

TEST(AStarPlanner, FullyBlockedMapIsRejected)
{
  const Costmap map(MapGeometry(6, 6, RESOLUTION, Vector2d::Zero()), LETHAL_OBSTACLE);
  EXPECT_FALSE(plan_raw(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 5, 5)).has_value());
}

TEST(AStarPlanner, NonFiniteCoordinatesAreRejected)
{
  const Costmap map = open_map(5, 5);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_FALSE(
    plan_raw(map, make_cost_model(), Pose2D{Vector2d{nan, nan}, 0.0}, at_cell(map, 4, 4)).has_value());
  EXPECT_FALSE(
    plan_raw(map, make_cost_model(), at_cell(map, 0, 0), Pose2D{Vector2d{nan, 0.25}, 0.0}).has_value());
  EXPECT_FALSE(
    plan_raw(map, make_cost_model(), Pose2D{Vector2d{inf, inf}, 0.0}, at_cell(map, 4, 4)).has_value());
  EXPECT_FALSE(
    plan_raw(map, make_cost_model(), at_cell(map, 0, 0), Pose2D{Vector2d{0.25, -inf}, 0.0}).has_value());
}

TEST(AStarPlanner, DefaultOutputEqualsTheRawPathSmoothedSeparately)
{
  const Costmap map = wall_map();
  const Pose2D start = at_cell(map, 0, 0);
  const Pose2D goal = at_cell(map, 4, 0, 0.3);

  const auto raw = plan_raw(map, make_cost_model(), start, goal);
  ASSERT_TRUE(raw.has_value());
  const auto planned = eltanin::planner::plan_astar(map, make_cost_model(), start, goal);
  ASSERT_TRUE(planned.has_value());

  expect_same_path(*planned, eltanin::planner::smooth(*raw, map, make_cost_model()));
}

TEST(AStarPlanner, EveryReturnedYawIsFinite)
{
  const Costmap wall = wall_map();
  const Costmap open = open_map(5, 5);
  const Costmap rescued = eltanin_test::make_costmap(
    {
      ".....",
      ".....",
      "###..",
      "###..",
      "###..",
    },
    RESOLUTION);

  const eltanin::planner::PlanResult results[] = {
    eltanin::planner::plan_astar(wall, make_cost_model(), at_cell(wall, 0, 0), at_cell(wall, 4, 0)),
    plan_raw(wall, make_cost_model(), at_cell(wall, 0, 0), at_cell(wall, 4, 0)),
    plan_raw(open, make_cost_model(), at_cell(open, 2, 2, 0.5), at_cell(open, 2, 2, 1.25)),
    plan_raw(
      rescued, make_cost_model(), at_cell(rescued, 1, 1), at_cell(rescued, 4, 4),
      eltanin_test::raw_astar_params(2))};

  for (const eltanin::planner::PlanResult & result : results) {
    ASSERT_TRUE(result.has_value());
    for (std::size_t i = 0; i < result->size(); ++i) {
      EXPECT_TRUE(std::isfinite((*result)[i].yaw)) << "pose " << i;
    }
  }
}
