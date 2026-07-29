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

#include <eltanin/map/map_geometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace
{

using Eigen::Vector2d;
using eltanin::map::MapGeometry;
using eltanin::map::MapIndex;

constexpr double kTol = 1e-12;

MapGeometry sample_geometry()
{
  return MapGeometry(10, 8, 0.05, Vector2d{-1.0, -2.0});
}

}  // namespace

TEST(MapGeometry, DefaultIsEmpty)
{
  const MapGeometry geometry;
  EXPECT_EQ(geometry.size_x(), 0);
  EXPECT_EQ(geometry.size_y(), 0);
  EXPECT_EQ(geometry.cell_count(), 0u);
  EXPECT_FALSE(geometry.in_bounds(0, 0));
}

TEST(MapGeometry, MapToWorldUsesCellCenter)
{
  const MapGeometry geometry = sample_geometry();
  const Vector2d center = geometry.map_to_world(0, 0);
  EXPECT_NEAR(center.x(), -1.0 + 0.025, kTol);
  EXPECT_NEAR(center.y(), -2.0 + 0.025, kTol);

  const Vector2d other = geometry.map_to_world(3, 2);
  EXPECT_NEAR(other.x(), -1.0 + 3.5 * 0.05, kTol);
  EXPECT_NEAR(other.y(), -2.0 + 2.5 * 0.05, kTol);
}

TEST(MapGeometry, RowZeroIsAtTheOriginSideInY)
{
  const MapGeometry geometry = sample_geometry();
  EXPECT_LT(geometry.map_to_world(0, 0).y(), geometry.map_to_world(0, 1).y());
}

TEST(MapGeometry, IndexIsRowMajor)
{
  const MapGeometry geometry = sample_geometry();
  EXPECT_EQ(geometry.index(0, 0), 0u);
  EXPECT_EQ(geometry.index(1, 0), 1u);
  EXPECT_EQ(geometry.index(0, 1), 10u);
  EXPECT_EQ(geometry.index(9, 7), 79u);
}

TEST(MapGeometry, WorldToMapToWorldRoundTrip)
{
  const MapGeometry geometry = sample_geometry();
  for (int i = 0; i < 40; ++i) {
    const double t = static_cast<double>(i);
    const Vector2d world{-1.0 + 0.0123 * t, -2.0 + 0.0091 * t};
    const auto index = geometry.world_to_map(world);
    ASSERT_TRUE(index.has_value()) << "i=" << i;
    const Vector2d recovered = geometry.map_to_world(index->x, index->y);
    const double bound = 0.5 * geometry.resolution() + 1e-12;
    EXPECT_LE(std::abs(recovered.x() - world.x()), bound);
    EXPECT_LE(std::abs(recovered.y() - world.y()), bound);
  }
}

TEST(MapGeometry, MapToWorldToMapRoundTrip)
{
  const MapGeometry geometry = sample_geometry();
  for (int my = 0; my < geometry.size_y(); ++my) {
    for (int mx = 0; mx < geometry.size_x(); ++mx) {
      const auto recovered = geometry.world_to_map(geometry.map_to_world(mx, my));
      ASSERT_TRUE(recovered.has_value());
      EXPECT_EQ(recovered->x, mx);
      EXPECT_EQ(recovered->y, my);
    }
  }
}

TEST(MapGeometry, NegativeOffsetIsDetectedAsOutOfBounds)
{
  const MapGeometry geometry = sample_geometry();
  // A truncating cast would map this to 0 and wrongly report it in bounds.
  EXPECT_FALSE(geometry.world_to_map(Vector2d{-1.0 - 0.025, -2.0 + 0.025}).has_value());
  EXPECT_FALSE(geometry.world_to_map(Vector2d{-1.0 + 0.025, -2.0 - 0.025}).has_value());
  EXPECT_FALSE(geometry.world_to_map(Vector2d{-1.0 - 1e-9, -2.0 + 0.025}).has_value());
  EXPECT_TRUE(geometry.world_to_map(Vector2d{-1.0, -2.0}).has_value());
}

TEST(MapGeometry, UnboundedConversionReturnsNegativeIndices)
{
  const MapGeometry geometry = sample_geometry();
  const MapIndex index = geometry.world_to_map_no_bounds(Vector2d{-1.0 - 0.06, -2.0 - 0.11});
  EXPECT_EQ(index.x, -2);
  EXPECT_EQ(index.y, -3);

  const MapIndex beyond = geometry.world_to_map_no_bounds(Vector2d{-1.0 + 0.57, -2.0 + 0.46});
  EXPECT_EQ(beyond.x, 11);
  EXPECT_EQ(beyond.y, 9);
}

TEST(MapGeometry, UnboundedConversionSaturatesInsteadOfOverflowing)
{
  const MapGeometry geometry = sample_geometry();
  const MapIndex huge = geometry.world_to_map_no_bounds(Vector2d{1e18, -1e18});
  EXPECT_EQ(huge.x, std::numeric_limits<int>::max());
  EXPECT_EQ(huge.y, std::numeric_limits<int>::min());
}

TEST(MapGeometry, InBoundsAtCornersAndOneCellOutside)
{
  const MapGeometry geometry = sample_geometry();
  EXPECT_TRUE(geometry.in_bounds(0, 0));
  EXPECT_TRUE(geometry.in_bounds(9, 0));
  EXPECT_TRUE(geometry.in_bounds(0, 7));
  EXPECT_TRUE(geometry.in_bounds(9, 7));
  EXPECT_FALSE(geometry.in_bounds(-1, 0));
  EXPECT_FALSE(geometry.in_bounds(0, -1));
  EXPECT_FALSE(geometry.in_bounds(10, 0));
  EXPECT_FALSE(geometry.in_bounds(0, 8));
}

TEST(MapGeometry, EqualityIsExact)
{
  const MapGeometry geometry = sample_geometry();
  EXPECT_EQ(geometry, MapGeometry(10, 8, 0.05, Vector2d{-1.0, -2.0}));
  EXPECT_NE(geometry, MapGeometry(11, 8, 0.05, Vector2d{-1.0, -2.0}));
  EXPECT_NE(geometry, MapGeometry(10, 9, 0.05, Vector2d{-1.0, -2.0}));
  EXPECT_NE(geometry, MapGeometry(10, 8, 0.0500001, Vector2d{-1.0, -2.0}));
  EXPECT_NE(geometry, MapGeometry(10, 8, 0.05, Vector2d{-1.0, -2.0000001}));
}

TEST(MapGeometry, CellCountDoesNotOverflowIntArithmetic)
{
  const MapGeometry large(4000, 4000, 0.05, Vector2d{-100.0, -100.0});
  EXPECT_EQ(large.cell_count(), 16000000u);
  EXPECT_EQ(large.index(3999, 3999), 15999999u);
  EXPECT_EQ(large.index(0, 3999), 15996000u);
}
