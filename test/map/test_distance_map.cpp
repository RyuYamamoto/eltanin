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

#include <eltanin/map/distance_map.hpp>

#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/cost_values.hpp>
#include <eltanin/map/grid_map.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace
{

using eltanin::map::build_distance_map;
using eltanin::map::Costmap;
using eltanin::map::CostTraversabilityModel;
using eltanin::map::DistanceMap;
using eltanin::map::DistanceMapParams;
using eltanin::map::MapGeometry;

constexpr double RESOLUTION = 0.05;

CostTraversabilityModel obstacle_model()
{
  return CostTraversabilityModel(eltanin::map::INSCRIBED_INFLATED_OBSTACLE, false);
}

Costmap free_map(int size_x, int size_y)
{
  return Costmap(
    MapGeometry(size_x, size_y, RESOLUTION, Eigen::Vector2d::Zero()), eltanin::map::FREE_SPACE);
}

/// Brute-force nearest-source distance in cells; the reference the transform must reproduce.
double brute_force_cells(const Costmap & map, const CostTraversabilityModel & model, int mx, int my)
{
  double best = std::numeric_limits<double>::infinity();
  for (int sy = 0; sy < map.size_y(); ++sy) {
    for (int sx = 0; sx < map.size_x(); ++sx) {
      if (!model.is_obstacle(map(sx, sy))) {
        continue;
      }
      const double dx = static_cast<double>(mx - sx);
      const double dy = static_cast<double>(my - sy);
      best = std::min(best, std::sqrt(dx * dx + dy * dy));
    }
  }
  return best;
}

}  // namespace

TEST(DistanceMap, MatchesTheBruteForceDistanceEverywhere)
{
  Costmap map = free_map(11, 9);
  map(2, 3) = eltanin::map::LETHAL_OBSTACLE;
  map(8, 1) = eltanin::map::LETHAL_OBSTACLE;
  map(5, 7) = eltanin::map::LETHAL_OBSTACLE;

  DistanceMapParams params;
  params.max_distance = 100.0;
  const std::optional<DistanceMap> distances = build_distance_map(map, obstacle_model(), params);

  ASSERT_TRUE(distances.has_value());
  for (int my = 0; my < map.size_y(); ++my) {
    for (int mx = 0; mx < map.size_x(); ++mx) {
      const double expected = brute_force_cells(map, obstacle_model(), mx, my) * RESOLUTION;
      EXPECT_NEAR((*distances)(mx, my), expected, 1e-6) << "cell " << mx << ", " << my;
    }
  }
}

TEST(DistanceMap, AnObstacleCellIsExactlyZero)
{
  Costmap map = free_map(6, 6);
  map(3, 4) = eltanin::map::LETHAL_OBSTACLE;

  const std::optional<DistanceMap> distances =
    build_distance_map(map, obstacle_model(), DistanceMapParams{});

  ASSERT_TRUE(distances.has_value());
  EXPECT_EQ((*distances)(3, 4), 0.0F);
  EXPECT_FALSE(eltanin::DistanceTraversabilityModel::from_radii(0.1, 0.2, 0.3)
                 ->is_obstacle((*distances)(2, 4)));
  EXPECT_TRUE(eltanin::DistanceTraversabilityModel::from_radii(0.1, 0.2, 0.3)
                ->is_obstacle((*distances)(3, 4)));
}

TEST(DistanceMap, SaturatesAtMaxDistance)
{
  Costmap map = free_map(40, 40);
  map(0, 0) = eltanin::map::LETHAL_OBSTACLE;

  DistanceMapParams params;
  params.max_distance = 0.3;
  const std::optional<DistanceMap> distances = build_distance_map(map, obstacle_model(), params);

  ASSERT_TRUE(distances.has_value());
  EXPECT_FLOAT_EQ((*distances)(39, 39), 0.3F);
  EXPECT_FLOAT_EQ((*distances)(4, 0), 0.2F);
  EXPECT_FLOAT_EQ((*distances)(8, 0), 0.3F);
}

TEST(DistanceMap, SaturatesEvenWhenTheMapIsSmallerThanMaxDistance)
{
  Costmap map = free_map(8, 8);
  map(0, 0) = eltanin::map::LETHAL_OBSTACLE;

  DistanceMapParams params;
  params.max_distance = 1.0;
  const std::optional<DistanceMap> distances = build_distance_map(map, obstacle_model(), params);

  ASSERT_TRUE(distances.has_value());
  EXPECT_FLOAT_EQ((*distances)(7, 7), static_cast<float>(std::sqrt(98.0) * RESOLUTION));
  EXPECT_LT((*distances)(7, 7), 1.0F);
}

TEST(DistanceMap, AMapWithoutObstaclesIsUniformlyMaxDistance)
{
  const Costmap map = free_map(12, 12);

  DistanceMapParams params;
  params.max_distance = 0.75;
  const std::optional<DistanceMap> distances = build_distance_map(map, obstacle_model(), params);

  ASSERT_TRUE(distances.has_value());
  for (const float distance : distances->data()) {
    EXPECT_FLOAT_EQ(distance, 0.75F);
  }
}

TEST(DistanceMap, UnknownIsASourceOnlyWhenTheModelSaysSo)
{
  Costmap map = free_map(8, 8);
  map(4, 4) = eltanin::map::NO_INFORMATION;

  const std::optional<DistanceMap> strict =
    build_distance_map(map, CostTraversabilityModel(1, false), DistanceMapParams{});
  const std::optional<DistanceMap> lenient =
    build_distance_map(map, CostTraversabilityModel(1, true), DistanceMapParams{});

  ASSERT_TRUE(strict.has_value());
  ASSERT_TRUE(lenient.has_value());
  EXPECT_EQ((*strict)(4, 4), 0.0F);
  EXPECT_FLOAT_EQ((*lenient)(4, 4), 1.0F);
}

TEST(DistanceMap, KeepsTheGeometryOfTheSourceMap)
{
  const Costmap map(MapGeometry(7, 5, RESOLUTION, Eigen::Vector2d{1.5, -2.0}), eltanin::map::FREE_SPACE);

  const std::optional<DistanceMap> distances =
    build_distance_map(map, obstacle_model(), DistanceMapParams{});

  ASSERT_TRUE(distances.has_value());
  EXPECT_TRUE(distances->geometry() == map.geometry());
}

TEST(DistanceMap, RejectsAnUnusableMapOrSaturation)
{
  const Costmap empty;
  EXPECT_FALSE(build_distance_map(empty, obstacle_model(), DistanceMapParams{}).has_value());

  const Costmap map = free_map(4, 4);
  DistanceMapParams zero;
  zero.max_distance = 0.0;
  EXPECT_FALSE(build_distance_map(map, obstacle_model(), zero).has_value());

  DistanceMapParams infinite;
  infinite.max_distance = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(build_distance_map(map, obstacle_model(), infinite).has_value());
}

TEST(DistanceTransformSquared, HandlesASingleRowAndASingleColumn)
{
  std::vector<double> row(5, eltanin::map::detail::unreachable_squared(5, 1));
  row[0] = 0.0;
  eltanin::map::detail::distance_transform_squared(row, 5, 1);
  for (int index = 0; index < 5; ++index) {
    EXPECT_DOUBLE_EQ(row[static_cast<std::size_t>(index)], static_cast<double>(index * index));
  }

  std::vector<double> column(4, eltanin::map::detail::unreachable_squared(1, 4));
  column[3] = 0.0;
  eltanin::map::detail::distance_transform_squared(column, 1, 4);
  EXPECT_DOUBLE_EQ(column[0], 9.0);
  EXPECT_DOUBLE_EQ(column[3], 0.0);
}
