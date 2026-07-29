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

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <utility>

namespace
{

using eltanin::Vec2;
using eltanin::map::Costmap;
using eltanin::map::GridMap;
using eltanin::map::MapGeometry;

MapGeometry sample_geometry()
{
  return MapGeometry(6, 4, 0.1, Vec2{0.0, 0.0});
}

}  // namespace

TEST(GridMap, DefaultIsEmpty)
{
  const Costmap map;
  EXPECT_EQ(map.cell_count(), 0u);
  EXPECT_EQ(map.size_x(), 0);
  EXPECT_EQ(map.size_y(), 0);
  EXPECT_FALSE(map.get(0, 0).has_value());
}

TEST(GridMap, ConstructionInitializesEveryCell)
{
  const Costmap map(sample_geometry(), eltanin::map::NO_INFORMATION);
  ASSERT_EQ(map.cell_count(), 24u);
  for (std::size_t i = 0; i < map.cell_count(); ++i) {
    EXPECT_EQ(map[i], eltanin::map::NO_INFORMATION);
  }
}

TEST(GridMap, ReadWriteThroughCoordinates)
{
  Costmap map(sample_geometry(), eltanin::map::FREE_SPACE);
  map(2, 1) = eltanin::map::LETHAL_OBSTACLE;
  EXPECT_EQ(map(2, 1), eltanin::map::LETHAL_OBSTACLE);
  EXPECT_EQ(map[map.geometry().index(2, 1)], eltanin::map::LETHAL_OBSTACLE);
  EXPECT_EQ(map(2, 0), eltanin::map::FREE_SPACE);
}

TEST(GridMap, SafeAccessorsHandleOutOfBounds)
{
  Costmap map(sample_geometry(), eltanin::map::FREE_SPACE);
  EXPECT_TRUE(map.set(0, 0, 42));
  EXPECT_FALSE(map.set(-1, 0, 42));
  EXPECT_FALSE(map.set(6, 0, 42));
  EXPECT_FALSE(map.set(0, 4, 42));

  const auto value = map.get(0, 0);
  ASSERT_TRUE(value.has_value());
  EXPECT_EQ(*value, 42);
  EXPECT_FALSE(map.get(-1, -1).has_value());
  EXPECT_EQ(map.get_or(-1, -1, eltanin::map::NO_INFORMATION), eltanin::map::NO_INFORMATION);
  EXPECT_EQ(map.get_or(0, 0, eltanin::map::NO_INFORMATION), 42);
}

TEST(GridMap, FillOverwritesEveryCell)
{
  Costmap map(sample_geometry(), eltanin::map::FREE_SPACE);
  map(1, 1) = eltanin::map::LETHAL_OBSTACLE;
  map.fill(eltanin::map::NO_INFORMATION);
  for (std::size_t i = 0; i < map.cell_count(); ++i) {
    EXPECT_EQ(map[i], eltanin::map::NO_INFORMATION);
  }
}

TEST(GridMap, DataGivesRawAccess)
{
  Costmap map(sample_geometry(), eltanin::map::FREE_SPACE);
  map.data()[3] = 7;
  EXPECT_EQ(map[3], 7);
  EXPECT_EQ(map.data().size(), map.cell_count());
}

TEST(GridMap, MakeLikeKeepsGeometryAndChangesCellType)
{
  const Costmap map(sample_geometry(), eltanin::map::FREE_SPACE);
  const GridMap<float> distances = map.make_like<float>(-1.0F);
  EXPECT_EQ(distances.geometry(), map.geometry());
  EXPECT_EQ(distances.cell_count(), map.cell_count());
  EXPECT_FLOAT_EQ(distances[0], -1.0F);
}

TEST(GridMap, InstantiatesForSeveralCellTypes)
{
  const GridMap<std::uint8_t> bytes(sample_geometry(), 1);
  const GridMap<float> floats(sample_geometry(), 1.5F);
  const GridMap<std::int32_t> ints(sample_geometry(), -3);

  EXPECT_EQ(bytes[0], 1);
  EXPECT_FLOAT_EQ(floats[0], 1.5F);
  EXPECT_EQ(ints[0], -3);
}

TEST(GridMap, CopyAndMoveArePermitted)
{
  Costmap map(sample_geometry(), eltanin::map::FREE_SPACE);
  map(1, 1) = 9;

  Costmap copy = map;
  EXPECT_EQ(copy(1, 1), 9);
  copy(1, 1) = 10;
  EXPECT_EQ(map(1, 1), 9);

  const Costmap moved = std::move(copy);
  EXPECT_EQ(moved(1, 1), 10);
}

TEST(GridMap, LargeMapAllocatesEveryCell)
{
  const Costmap map(MapGeometry(4000, 4000, 0.05, Vec2{-100.0, -100.0}), eltanin::map::FREE_SPACE);
  EXPECT_EQ(map.cell_count(), 16000000u);
  EXPECT_EQ(map(3999, 3999), eltanin::map::FREE_SPACE);
}
