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

#include <array>
#include <numbers>

namespace
{

using Eigen::Vector2d;
using eltanin::Pose2D;
using eltanin::path_length;
using eltanin::map::Costmap;
using eltanin::map::FREE_SPACE;
using eltanin::map::MapGeometry;
using eltanin::map::MapIndex;
using eltanin::planner::plan;
using eltanin_test::expect_grid_connected;
using eltanin_test::make_cost_model;

constexpr double RESOLUTION = 0.1;
constexpr double TOLERANCE = 1e-9;

Costmap open_map(int size_x, int size_y)
{
  return Costmap(MapGeometry(size_x, size_y, RESOLUTION, Vector2d::Zero()), FREE_SPACE);
}

Pose2D at_cell(const Costmap & map, int mx, int my)
{
  return Pose2D{map.geometry().map_to_world(mx, my), 0.0};
}

/// Only the two outer columns are free, so the left and right halves are disconnected.
Costmap split_two_row_map()
{
  return eltanin_test::make_costmap({".##.", ".##."}, RESOLUTION);
}

}  // namespace

TEST(AStarBounds, DoesNotWrapAroundTheLeftEdge)
{
  const Costmap map = split_two_row_map();
  // Indexing (0, 1) with dx = -1 before the range check yields -1 + 4 * 1 = 3, the goal cell.
  EXPECT_FALSE(plan(map, make_cost_model(), at_cell(map, 0, 1), at_cell(map, 3, 0)).has_value());
}

TEST(AStarBounds, DoesNotWrapAroundTheRightEdge)
{
  const Costmap map = split_two_row_map();
  // Indexing (3, 0) with dx = +1 before the range check yields 4 + 4 * 0 = 4, the goal cell.
  EXPECT_FALSE(plan(map, make_cost_model(), at_cell(map, 3, 0), at_cell(map, 0, 1)).has_value());
}

TEST(AStarBounds, ConnectsWithinTheSameEdgeColumn)
{
  const Costmap map = split_two_row_map();
  const auto path = plan(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 0, 1));
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->size(), 2u);
  EXPECT_NEAR(path_length(*path), RESOLUTION, TOLERANCE);
}

TEST(AStarBounds, ExpandsTheOriginCellWithoutNegativeIndices)
{
  const Costmap map = open_map(3, 3);
  const auto path = plan(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 2, 2));
  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path_length(*path), 2.0 * std::numbers::sqrt2 * RESOLUTION, TOLERANCE);
  expect_grid_connected(*path, RESOLUTION);
}

TEST(AStarBounds, PlansBetweenEveryPairOfCorners)
{
  const Costmap map = open_map(6, 4);
  const std::array<MapIndex, 4> corners{
    MapIndex{0, 0}, MapIndex{5, 0}, MapIndex{0, 3}, MapIndex{5, 3}};

  for (const MapIndex & start : corners) {
    for (const MapIndex & goal : corners) {
      const auto path = plan(
        map, make_cost_model(), at_cell(map, start.x, start.y), at_cell(map, goal.x, goal.y));
      ASSERT_TRUE(path.has_value()) << start.x << "," << start.y << " -> " << goal.x << ","
                                    << goal.y;
      expect_grid_connected(*path, RESOLUTION);
      const Vector2d first = map.geometry().map_to_world(start.x, start.y);
      const Vector2d last = map.geometry().map_to_world(goal.x, goal.y);
      EXPECT_EQ((*path)[0].position.x(), first.x());
      EXPECT_EQ((*path)[0].position.y(), first.y());
      EXPECT_EQ((*path)[path->size() - 1].position.x(), last.x());
      EXPECT_EQ((*path)[path->size() - 1].position.y(), last.y());
    }
  }
}

TEST(AStarBounds, PlansAlongEachBorderLine)
{
  const Costmap map = open_map(6, 5);
  const int last_x = map.size_x() - 1;
  const int last_y = map.size_y() - 1;

  const std::array<std::array<MapIndex, 2>, 4> borders{
    std::array<MapIndex, 2>{MapIndex{0, 0}, MapIndex{0, last_y}},
    std::array<MapIndex, 2>{MapIndex{last_x, 0}, MapIndex{last_x, last_y}},
    std::array<MapIndex, 2>{MapIndex{0, 0}, MapIndex{last_x, 0}},
    std::array<MapIndex, 2>{MapIndex{0, last_y}, MapIndex{last_x, last_y}}};

  for (const auto & border : borders) {
    const auto path = plan(
      map, make_cost_model(), at_cell(map, border[0].x, border[0].y),
      at_cell(map, border[1].x, border[1].y));
    ASSERT_TRUE(path.has_value());
    expect_grid_connected(*path, RESOLUTION);
    const int steps = std::abs(border[1].x - border[0].x) + std::abs(border[1].y - border[0].y);
    EXPECT_NEAR(path_length(*path), steps * RESOLUTION, TOLERANCE);
  }
}

TEST(AStarBounds, PlansOnASingleColumnGrid)
{
  const Costmap map = open_map(1, 5);
  const auto path = plan(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 0, 4));
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->size(), 5u);
  EXPECT_NEAR(path_length(*path), 4.0 * RESOLUTION, TOLERANCE);
  expect_grid_connected(*path, RESOLUTION);
}

TEST(AStarBounds, PlansOnASingleRowGrid)
{
  const Costmap map = open_map(5, 1);
  const auto path = plan(map, make_cost_model(), at_cell(map, 4, 0), at_cell(map, 0, 0));
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->size(), 5u);
  EXPECT_NEAR(path_length(*path), 4.0 * RESOLUTION, TOLERANCE);
  expect_grid_connected(*path, RESOLUTION);
}

TEST(AStarBounds, PlansOnASingleCellGrid)
{
  const Costmap map = open_map(1, 1);
  const auto path = plan(map, make_cost_model(), at_cell(map, 0, 0), at_cell(map, 0, 0));
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->size(), 1u);
  EXPECT_EQ(path_length(*path), 0.0);
}
