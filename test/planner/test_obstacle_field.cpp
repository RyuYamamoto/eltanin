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
#include <eltanin/planner/obstacle_field.hpp>

#include <map/costmap_fixture.hpp>
#include <planner/planner_fixture.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>

namespace
{

using Eigen::Vector2d;
using eltanin::map::Costmap;
using eltanin::map::FREE_SPACE;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::MapGeometry;
using eltanin::planner::ObstacleField;
using eltanin::planner::TraversabilityView;
using eltanin::planner::detail::build_obstacle_distance;
using eltanin::planner::detail::build_traversability_grid;
using eltanin_test::make_cost_model;

/// Nearest blocked cell by exhaustive search, with the first cell outside the map counting too.
double nearest_blocked(const TraversabilityView & view, int mx, int my, double resolution)
{
  const auto & geometry = view.geometry();
  double best = static_cast<double>(std::min(
                  {mx + 1, my + 1, geometry.size_x() - mx, geometry.size_y() - my})) *
                resolution;
  for (int qy = 0; qy < geometry.size_y(); ++qy) {
    for (int qx = 0; qx < geometry.size_x(); ++qx) {
      if (view.traversable(qx, qy)) {
        continue;
      }
      best = std::min(best, std::hypot(mx - qx, my - qy) * resolution);
    }
  }
  return best;
}

}  // namespace

TEST(ObstacleField, MatchesAnExhaustiveNearestObstacleSearch)
{
  constexpr double resolution = 0.07;
  std::mt19937 rng(20260809);
  for (int trial = 0; trial < 8; ++trial) {
    const int size_x = 12 + static_cast<int>(rng() % 25);
    const int size_y = 12 + static_cast<int>(rng() % 25);
    Costmap map(MapGeometry(size_x, size_y, resolution, Vector2d::Zero()), FREE_SPACE);
    for (int i = 0; i < size_x * size_y / 12; ++i) {
      map(static_cast<int>(rng() % static_cast<unsigned>(size_x)),
          static_cast<int>(rng() % static_cast<unsigned>(size_y))) = LETHAL_OBSTACLE;
    }

    const auto grid = build_traversability_grid(map, make_cost_model());
    const TraversabilityView view{map.geometry(), grid};
    const auto distance = build_obstacle_distance(view);
    const ObstacleField field{map.geometry(), distance};

    for (int my = 0; my < size_y; ++my) {
      for (int mx = 0; mx < size_x; ++mx) {
        EXPECT_NEAR(field.at(mx, my), nearest_blocked(view, mx, my, resolution), 1e-6)
          << "trial " << trial << " cell " << mx << ", " << my;
      }
    }
  }
}

TEST(ObstacleField, ReportsZeroOnBlockedCellsAndOutsideTheMap)
{
  Costmap map(MapGeometry(10, 10, 0.1, Vector2d::Zero()), FREE_SPACE);
  map(4, 4) = LETHAL_OBSTACLE;

  const auto grid = build_traversability_grid(map, make_cost_model());
  const TraversabilityView view{map.geometry(), grid};
  const auto distance = build_obstacle_distance(view);
  const ObstacleField field{map.geometry(), distance};

  EXPECT_DOUBLE_EQ(field.at(4, 4), 0.0);
  EXPECT_DOUBLE_EQ(field.at(-1, 4), 0.0);
  EXPECT_DOUBLE_EQ(field.at(Vector2d{-5.0, -5.0}), 0.0);
  EXPECT_NEAR(field.at(5, 4), 0.1, 1e-6);
  EXPECT_NEAR(field.at(4, 6), 0.2, 1e-6);
}

TEST(ObstacleField, GradientPointsAwayFromTheObstacle)
{
  Costmap map(MapGeometry(41, 41, 0.1, Vector2d::Zero()), FREE_SPACE);
  for (int my = 0; my < 41; ++my) {
    map(20, my) = LETHAL_OBSTACLE;
  }

  const auto grid = build_traversability_grid(map, make_cost_model());
  const TraversabilityView view{map.geometry(), grid};
  const auto distance = build_obstacle_distance(view);
  const ObstacleField field{map.geometry(), distance};

  // A wall down the middle: the distance rises to the right of it, so the gradient points right.
  const Vector2d right = map.geometry().map_to_world(25, 20);
  EXPECT_GT(field.gradient(right).x(), 0.0);
  EXPECT_NEAR(field.gradient(right).y(), 0.0, 1e-9);

  const Vector2d left = map.geometry().map_to_world(15, 20);
  EXPECT_LT(field.gradient(left).x(), 0.0);
  EXPECT_NEAR(field.gradient(left).y(), 0.0, 1e-9);
}

TEST(ObstacleField, AnEmptyFieldIsSafeToQuery)
{
  const ObstacleField field;

  EXPECT_TRUE(field.empty());
  EXPECT_DOUBLE_EQ(field.at(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(field.at(Vector2d{1.0, 1.0}), 0.0);
  EXPECT_EQ(field.gradient(Vector2d{1.0, 1.0}), Vector2d::Zero());
}
