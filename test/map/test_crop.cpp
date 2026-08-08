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
#include <eltanin/map/crop.hpp>
#include <eltanin/map/grid_map.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace
{

using Eigen::Vector2d;
using eltanin::map::CellRect;
using eltanin::map::Costmap;
using eltanin::map::FREE_SPACE;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::MapGeometry;
using eltanin::map::bounding_cells;
using eltanin::map::crop_around;

constexpr double RESOLUTION = 0.1;

Costmap numbered_map(int size_x, int size_y, const Vector2d & origin = Vector2d::Zero())
{
  Costmap map(MapGeometry(size_x, size_y, RESOLUTION, origin), FREE_SPACE);
  for (int my = 0; my < size_y; ++my) {
    for (int mx = 0; mx < size_x; ++mx) {
      map(mx, my) = static_cast<std::uint8_t>((mx * 7 + my * 13) % 250);
    }
  }
  return map;
}

}  // namespace

TEST(MapCrop, CoversEveryPositionWithTheRequestedMargin)
{
  const Costmap map = numbered_map(40, 40);
  const std::vector<Vector2d> positions{
    map.geometry().map_to_world(10, 12), map.geometry().map_to_world(20, 15)};

  const auto rect = bounding_cells(map.geometry(), positions, 3);

  ASSERT_TRUE(rect.has_value());
  EXPECT_EQ(rect->min_x, 7);
  EXPECT_EQ(rect->min_y, 9);
  EXPECT_EQ(rect->max_x, 23);
  EXPECT_EQ(rect->max_y, 18);
}

TEST(MapCrop, ClampsTheMarginToTheMap)
{
  const Costmap map = numbered_map(12, 12);
  const std::vector<Vector2d> positions{map.geometry().map_to_world(1, 10)};

  const auto rect = bounding_cells(map.geometry(), positions, 5);

  ASSERT_TRUE(rect.has_value());
  EXPECT_EQ(rect->min_x, 0);
  EXPECT_EQ(rect->min_y, 5);
  EXPECT_EQ(rect->max_x, 6);
  EXPECT_EQ(rect->max_y, 11);
}

TEST(MapCrop, KeepsWorldCoordinatesThroughTheOrigin)
{
  const Costmap map = numbered_map(40, 40, Vector2d{-3.25, 7.5});
  const std::vector<Vector2d> positions{
    map.geometry().map_to_world(10, 12), map.geometry().map_to_world(20, 15)};

  const auto cropped = crop_around(map, positions, 2);

  ASSERT_TRUE(cropped.has_value());
  for (const Vector2d & position : positions) {
    const auto source = map.geometry().world_to_map(position);
    const auto target = cropped->geometry().world_to_map(position);
    ASSERT_TRUE(source.has_value());
    ASSERT_TRUE(target.has_value());
    EXPECT_EQ(map(source->x, source->y), (*cropped)(target->x, target->y));
    EXPECT_DOUBLE_EQ(
      map.geometry().map_to_world(source->x, source->y).x(),
      cropped->geometry().map_to_world(target->x, target->y).x());
    EXPECT_DOUBLE_EQ(
      map.geometry().map_to_world(source->x, source->y).y(),
      cropped->geometry().map_to_world(target->x, target->y).y());
  }
}

TEST(MapCrop, CopiesEveryCellOfTheRectangle)
{
  const Costmap map = numbered_map(30, 20, Vector2d{1.0, -2.0});
  const std::vector<Vector2d> positions{
    map.geometry().map_to_world(5, 4), map.geometry().map_to_world(9, 11)};

  const auto cropped = crop_around(map, positions, 1);

  ASSERT_TRUE(cropped.has_value());
  ASSERT_EQ(cropped->size_x(), 7);
  ASSERT_EQ(cropped->size_y(), 10);
  for (int my = 0; my < cropped->size_y(); ++my) {
    for (int mx = 0; mx < cropped->size_x(); ++mx) {
      EXPECT_EQ((*cropped)(mx, my), map(4 + mx, 3 + my)) << "cell " << mx << "," << my;
    }
  }
}

TEST(MapCrop, RefusesPositionsThatAreAllOutsideTheMap)
{
  const Costmap map = numbered_map(10, 10);
  const std::vector<Vector2d> outside{Vector2d{-5.0, -5.0}, Vector2d{50.0, 50.0}};

  EXPECT_FALSE(bounding_cells(map.geometry(), outside, 2).has_value());
  EXPECT_FALSE(crop_around(map, outside, 2).has_value());
  EXPECT_FALSE(bounding_cells(map.geometry(), {}, 2).has_value());
}

TEST(MapCrop, RefusesANegativeMargin)
{
  const Costmap map = numbered_map(10, 10);
  const std::vector<Vector2d> positions{map.geometry().map_to_world(5, 5)};

  EXPECT_FALSE(bounding_cells(map.geometry(), positions, -1).has_value());
}

TEST(MapCrop, IgnoresPositionsOutsideTheMapWhenOthersAreInside)
{
  const Costmap map = numbered_map(20, 20);
  const std::vector<Vector2d> positions{
    Vector2d{-9.0, -9.0}, map.geometry().map_to_world(8, 8), Vector2d{99.0, 99.0}};

  const auto rect = bounding_cells(map.geometry(), positions, 0);

  ASSERT_TRUE(rect.has_value());
  EXPECT_EQ(rect->min_x, 8);
  EXPECT_EQ(rect->min_y, 8);
  EXPECT_EQ(rect->max_x, 8);
  EXPECT_EQ(rect->max_y, 8);
}

TEST(MapCrop, CropsADistanceMapTheSameWay)
{
  eltanin::map::DistanceMap map(MapGeometry(16, 16, RESOLUTION, Vector2d::Zero()), 0.0F);
  map(6, 6) = 1.5F;
  const std::vector<Vector2d> positions{map.geometry().map_to_world(6, 6)};

  const auto cropped = crop_around(map, positions, 2);

  ASSERT_TRUE(cropped.has_value());
  const auto cell = cropped->geometry().world_to_map(map.geometry().map_to_world(6, 6));
  ASSERT_TRUE(cell.has_value());
  EXPECT_FLOAT_EQ((*cropped)(cell->x, cell->y), 1.5F);
}
