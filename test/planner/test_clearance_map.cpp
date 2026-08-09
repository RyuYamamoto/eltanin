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
#include <eltanin/planner/clearance_map.hpp>

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
using eltanin::planner::ClearanceMap;
using eltanin::planner::clearance_at;
using eltanin::planner::clearance_gradient;
using eltanin::planner::TraversabilityView;
using eltanin::planner::detail::build_clearance_map;
using eltanin::planner::detail::build_traversability_grid;
using eltanin_test::make_cost_model;

/// Nearest blocked cell by exhaustive search; the map edge is not one, so it never bounds this.
double nearest_blocked(const TraversabilityView & view, int mx, int my, double resolution)
{
  const auto & geometry = view.geometry();
  double best = std::hypot(geometry.size_x(), geometry.size_y()) * resolution;
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

TEST(ClearanceMap, MatchesAnExhaustiveNearestObstacleSearch)
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
    const ClearanceMap field = build_clearance_map(view);

    for (int my = 0; my < size_y; ++my) {
      for (int mx = 0; mx < size_x; ++mx) {
        EXPECT_NEAR(field(mx, my), nearest_blocked(view, mx, my, resolution), 1e-6)
          << "trial " << trial << " cell " << mx << ", " << my;
      }
    }
  }
}

TEST(ClearanceMap, TreatsTheMapEdgeAsOpenRatherThanAsAWall)
{
  // A cropped corridor has map edges everywhere; taking them for walls would squeeze it shut.
  Costmap map(MapGeometry(10, 10, 0.1, Vector2d::Zero()), FREE_SPACE);
  map(9, 9) = LETHAL_OBSTACLE;

  const auto grid = build_traversability_grid(map, make_cost_model());
  const TraversabilityView view{map.geometry(), grid};
  const ClearanceMap field = build_clearance_map(view);

  // The far corner sits one cell from the edge but nine cells from the only obstacle.
  EXPECT_NEAR(field(0, 0), std::hypot(9.0, 9.0) * 0.1, 1e-5);
}

TEST(ClearanceMap, ReportsZeroOnBlockedCellsAndOutsideTheMap)
{
  Costmap map(MapGeometry(10, 10, 0.1, Vector2d::Zero()), FREE_SPACE);
  map(4, 4) = LETHAL_OBSTACLE;

  const auto grid = build_traversability_grid(map, make_cost_model());
  const TraversabilityView view{map.geometry(), grid};
  const ClearanceMap field = build_clearance_map(view);

  EXPECT_DOUBLE_EQ(field(4, 4), 0.0);
  EXPECT_FALSE(field.get(-1, 4).has_value());
  EXPECT_DOUBLE_EQ(clearance_at(field, Vector2d{-5.0, -5.0}, 0.0), 0.0);
  EXPECT_NEAR(field(5, 4), 0.1, 1e-6);
  EXPECT_NEAR(field(4, 6), 0.2, 1e-6);
}

TEST(ClearanceMap, GradientPointsAwayFromTheObstacle)
{
  Costmap map(MapGeometry(41, 41, 0.1, Vector2d::Zero()), FREE_SPACE);
  for (int my = 0; my < 41; ++my) {
    map(20, my) = LETHAL_OBSTACLE;
  }

  const auto grid = build_traversability_grid(map, make_cost_model());
  const TraversabilityView view{map.geometry(), grid};
  const ClearanceMap field = build_clearance_map(view);

  // A wall down the middle: the distance rises to the right of it, so the gradient points right.
  const Vector2d right = map.geometry().map_to_world(25, 20);
  EXPECT_GT(clearance_gradient(field, right).x(), 0.0);
  EXPECT_NEAR(clearance_gradient(field, right).y(), 0.0, 1e-9);

  const Vector2d left = map.geometry().map_to_world(15, 20);
  EXPECT_LT(clearance_gradient(field, left).x(), 0.0);
  EXPECT_NEAR(clearance_gradient(field, left).y(), 0.0, 1e-9);
}

TEST(ClearanceMap, AnEmptyMapIsSafeToQuery)
{
  const ClearanceMap field;

  EXPECT_EQ(field.cell_count(), 0u);
  EXPECT_DOUBLE_EQ(clearance_at(field, Vector2d{1.0, 1.0}, 0.25), 0.25);
  EXPECT_EQ(clearance_gradient(field, Vector2d{1.0, 1.0}), Vector2d::Zero());
}
