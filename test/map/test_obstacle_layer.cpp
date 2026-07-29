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
#include <eltanin/map/layers/obstacle_layer.hpp>

#include <map/costmap_fixture.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace
{

using Eigen::Vector2d;
using eltanin::map::Costmap;
using eltanin::map::FREE_SPACE;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::MapGeometry;
using eltanin::map::NO_INFORMATION;
using eltanin::map::ObstacleLayer;
using eltanin_test::cells_top_down;

constexpr std::uint8_t L = LETHAL_OBSTACLE;
constexpr std::uint8_t U = NO_INFORMATION;
constexpr std::uint8_t F = FREE_SPACE;

MapGeometry sample_geometry() { return MapGeometry(4, 3, 0.1, Vector2d{0.0, 0.0}); }

}  // namespace

TEST(ObstacleLayer, DoesNothingBeforeAnyPointIsSupplied)
{
  ObstacleLayer layer;
  Costmap master(sample_geometry(), FREE_SPACE);

  layer.update_costs(master);

  for (std::size_t i = 0; i < master.cell_count(); ++i) {
    EXPECT_EQ(master[i], FREE_SPACE);
  }
}

TEST(ObstacleLayer, MarksTheCellsHitByThePoints)
{
  const std::vector<Vector2d> points = {Vector2d{0.05, 0.05}, Vector2d{0.35, 0.25}};
  ObstacleLayer layer;
  layer.set_points(points);
  Costmap master(sample_geometry(), FREE_SPACE);

  layer.update_costs(master);

  const std::vector<std::uint8_t> expected = {
    F, F, F, L,
    F, F, F, F,
    L, F, F, F};
  EXPECT_EQ(cells_top_down(master), expected);
}

TEST(ObstacleLayer, DiscardsPointsBelowTheOrigin)
{
  const std::vector<Vector2d> points = {Vector2d{-0.01, 0.05}, Vector2d{0.05, -0.01}};
  ObstacleLayer layer;
  layer.set_points(points);
  Costmap master(sample_geometry(), FREE_SPACE);

  layer.update_costs(master);

  for (std::size_t i = 0; i < master.cell_count(); ++i) {
    EXPECT_EQ(master[i], FREE_SPACE);
  }
}

TEST(ObstacleLayer, DiscardsPointsBeyondTheUpperEdge)
{
  const std::vector<Vector2d> points = {Vector2d{0.45, 0.05}, Vector2d{0.05, 0.35}};
  ObstacleLayer layer;
  layer.set_points(points);
  Costmap master(sample_geometry(), FREE_SPACE);

  layer.update_costs(master);

  for (std::size_t i = 0; i < master.cell_count(); ++i) {
    EXPECT_EQ(master[i], FREE_SPACE);
  }
}

TEST(ObstacleLayer, OverwritesUnknownCells)
{
  const std::vector<Vector2d> points = {Vector2d{0.15, 0.15}};
  ObstacleLayer layer;
  layer.set_points(points);
  Costmap master(sample_geometry(), NO_INFORMATION);

  layer.update_costs(master);

  EXPECT_EQ(master(1, 1), L);
  EXPECT_EQ(master(0, 0), U);
}

TEST(ObstacleLayer, KeepsItsOwnCopyOfThePoints)
{
  ObstacleLayer layer;
  {
    const std::vector<Vector2d> points = {Vector2d{0.15, 0.15}};
    layer.set_points(points);
  }
  Costmap master(sample_geometry(), FREE_SPACE);

  layer.update_costs(master);

  EXPECT_EQ(master(1, 1), L);
}

TEST(ObstacleLayer, ReplacesThePreviousPointsOnEveryCall)
{
  ObstacleLayer layer;
  const std::vector<Vector2d> first = {Vector2d{0.05, 0.05}};
  const std::vector<Vector2d> second = {Vector2d{0.15, 0.15}};
  Costmap master(sample_geometry(), FREE_SPACE);

  layer.set_points(first);
  layer.set_points(second);
  layer.update_costs(master);

  EXPECT_EQ(master(0, 0), F);
  EXPECT_EQ(master(1, 1), L);
}

TEST(ObstacleLayer, LeavesTheGeometryUntouched)
{
  const std::vector<Vector2d> points = {Vector2d{0.15, 0.15}, Vector2d{-5.0, 3.0}};
  ObstacleLayer layer;
  layer.set_points(points);
  Costmap master(sample_geometry(), FREE_SPACE);

  layer.update_costs(master);

  EXPECT_EQ(master.geometry(), sample_geometry());
}
