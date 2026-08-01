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
#include <stdexcept>

namespace
{

using Eigen::Vector2d;
using eltanin::map::MapGeometry;

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

TEST(MapGeometry, ConstructorRejectsInvalidGeometry)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(MapGeometry(0, 8, 0.05, Vector2d::Zero()), std::invalid_argument);
  EXPECT_THROW(MapGeometry(10, -1, 0.05, Vector2d::Zero()), std::invalid_argument);
  EXPECT_THROW(MapGeometry(10, 8, 0.0, Vector2d::Zero()), std::invalid_argument);
  EXPECT_THROW(MapGeometry(10, 8, nan, Vector2d::Zero()), std::invalid_argument);
  EXPECT_THROW(MapGeometry(10, 8, 0.05, Vector2d{nan, 0.0}), std::invalid_argument);
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

TEST(MapGeometry, PointsBeyondEitherEdgeAreOutOfBounds)
{
  const MapGeometry geometry = sample_geometry();
  EXPECT_FALSE(geometry.world_to_map(Vector2d{-1.0 - 0.06, -2.0 - 0.11}).has_value());
  EXPECT_FALSE(geometry.world_to_map(Vector2d{-1.0 + 0.57, -2.0 + 0.46}).has_value());
  EXPECT_FALSE(geometry.world_to_map(Vector2d{-1.0 + 0.52, -2.0 + 0.2}).has_value());
  EXPECT_FALSE(geometry.world_to_map(Vector2d{-1.0 + 0.2, -2.0 + 0.43}).has_value());
  EXPECT_TRUE(geometry.world_to_map(Vector2d{-1.0 + 0.2, -2.0 + 0.2}).has_value());
}

TEST(MapGeometry, ExtremeCoordinatesAreRejectedWithoutOverflow)
{
  const MapGeometry geometry = sample_geometry();
  EXPECT_FALSE(geometry.world_to_map(Vector2d{1e18, -1e18}).has_value());
  EXPECT_FALSE(geometry.world_to_map(Vector2d{-1e300, 1e300}).has_value());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(geometry.world_to_map(Vector2d{nan, nan}).has_value());
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

TEST(MapGeometry, SetOriginMovesTheWindowOnly)
{
  MapGeometry geometry = sample_geometry();
  geometry.set_origin(Vector2d{3.0, -4.0});

  EXPECT_EQ(geometry.origin().x(), 3.0);
  EXPECT_EQ(geometry.origin().y(), -4.0);
  EXPECT_EQ(geometry.size_x(), 10);
  EXPECT_EQ(geometry.size_y(), 8);
  EXPECT_EQ(geometry.resolution(), 0.05);
  EXPECT_EQ(geometry, MapGeometry(10, 8, 0.05, Vector2d{3.0, -4.0}));
}

TEST(MapGeometry, SetOriginRejectsNonFiniteCoordinates)
{
  MapGeometry geometry = sample_geometry();
  const MapGeometry before = geometry;
  const double nan = std::numeric_limits<double>::quiet_NaN();

  EXPECT_THROW(geometry.set_origin(Vector2d{nan, 0.0}), std::invalid_argument);
  EXPECT_EQ(geometry, before);
}

TEST(MapGeometry, SetOriginShiftsTheWorldMapping)
{
  MapGeometry geometry = sample_geometry();
  geometry.set_origin(Vector2d{0.0, 0.0});

  const auto index = geometry.world_to_map(Vector2d{0.025, 0.025});
  ASSERT_TRUE(index.has_value());
  EXPECT_EQ(index->x, 0);
  EXPECT_EQ(index->y, 0);
  EXPECT_FALSE(geometry.world_to_map(Vector2d{-1.0, -2.0}).has_value());
}

TEST(MapGeometry, WorldRectToCellsCoversAnInteriorWindow)
{
  const MapGeometry geometry(10, 8, 0.05, Vector2d{-1.0, -2.0});
  const auto rect = geometry.world_rect_to_cells(Vector2d{-0.94, -1.94}, Vector2d{-0.81, -1.86});

  ASSERT_TRUE(rect.has_value());
  EXPECT_EQ(rect->min_x, 1);
  EXPECT_EQ(rect->min_y, 1);
  EXPECT_EQ(rect->max_x, 3);
  EXPECT_EQ(rect->max_y, 2);
}

TEST(MapGeometry, WorldRectToCellsClampsAPartiallyOutsideWindow)
{
  const MapGeometry geometry(10, 8, 0.05, Vector2d::Zero());
  const auto rect = geometry.world_rect_to_cells(Vector2d{-0.3, -0.3}, Vector2d{0.6, 0.6});

  ASSERT_TRUE(rect.has_value());
  EXPECT_EQ(rect->min_x, 0);
  EXPECT_EQ(rect->min_y, 0);
  EXPECT_EQ(rect->max_x, 9);
  EXPECT_EQ(rect->max_y, 7);
}

TEST(MapGeometry, WorldRectToCellsRejectsAWindowOutsideTheMap)
{
  const MapGeometry geometry(10, 8, 0.05, Vector2d::Zero());
  EXPECT_FALSE(
    geometry.world_rect_to_cells(Vector2d{-1.0, -1.0}, Vector2d{-0.1, -0.1}).has_value());
  EXPECT_FALSE(geometry.world_rect_to_cells(Vector2d{0.5, 0.5}, Vector2d{1.0, 1.0}).has_value());
  EXPECT_FALSE(geometry.world_rect_to_cells(Vector2d{0.1, 0.5}, Vector2d{0.2, 1.0}).has_value());
}

TEST(MapGeometry, WorldRectToCellsRejectsReversedBounds)
{
  const MapGeometry geometry = sample_geometry();
  EXPECT_FALSE(
    geometry.world_rect_to_cells(Vector2d{0.2, 0.1}, Vector2d{0.1, 0.2}).has_value());
  EXPECT_FALSE(
    geometry.world_rect_to_cells(Vector2d{0.1, 0.2}, Vector2d{0.2, 0.1}).has_value());
}

TEST(MapGeometry, WorldRectToCellsFloorsNegativeCoordinates)
{
  const MapGeometry geometry(10, 8, 0.05, Vector2d::Zero());
  // A static_cast<int> truncation towards zero would report min_x = 0 here instead of nullopt.
  EXPECT_FALSE(
    geometry.world_rect_to_cells(Vector2d{-0.04, -0.04}, Vector2d{-0.01, -0.01}).has_value());

  const auto rect = geometry.world_rect_to_cells(Vector2d{-0.04, -0.04}, Vector2d{0.06, 0.06});
  ASSERT_TRUE(rect.has_value());
  EXPECT_EQ(rect->min_x, 0);
  EXPECT_EQ(rect->max_x, 1);
}

TEST(MapGeometry, WorldRectToCellsPlacesCellBoundariesInTheUpperCell)
{
  const MapGeometry geometry(8, 8, 0.25, Vector2d::Zero());
  const auto rect = geometry.world_rect_to_cells(Vector2d{0.25, 0.25}, Vector2d{0.75, 0.75});

  ASSERT_TRUE(rect.has_value());
  EXPECT_EQ(rect->min_x, 1);
  EXPECT_EQ(rect->min_y, 1);
  EXPECT_EQ(rect->max_x, 3);
  EXPECT_EQ(rect->max_y, 3);
}

TEST(MapGeometry, WorldRectToCellsAcceptsADegenerateWindow)
{
  const MapGeometry geometry(10, 8, 0.05, Vector2d::Zero());
  const Vector2d point{0.13, 0.13};
  const auto rect = geometry.world_rect_to_cells(point, point);

  ASSERT_TRUE(rect.has_value());
  EXPECT_EQ(rect->min_x, 2);
  EXPECT_EQ(rect->max_x, 2);
  EXPECT_EQ(rect->min_y, 2);
  EXPECT_EQ(rect->max_y, 2);
}

TEST(MapGeometry, WorldRectToCellsRejectsEverythingOnAnEmptyMap)
{
  const MapGeometry geometry;
  EXPECT_FALSE(geometry.world_rect_to_cells(Vector2d::Zero(), Vector2d{1.0, 1.0}).has_value());
}

TEST(MapGeometry, WorldRectToCellsAgreesWithWorldToMapOnTheCorners)
{
  const MapGeometry geometry(10, 8, 0.05, Vector2d{-1.0, -2.0});
  const Vector2d min{-0.87, -1.83};
  const Vector2d max{-0.62, -1.68};
  const auto rect = geometry.world_rect_to_cells(min, max);
  const auto lower = geometry.world_to_map(min);
  const auto upper = geometry.world_to_map(max);

  ASSERT_TRUE(rect.has_value());
  ASSERT_TRUE(lower.has_value());
  ASSERT_TRUE(upper.has_value());
  EXPECT_EQ(rect->min_x, lower->x);
  EXPECT_EQ(rect->min_y, lower->y);
  EXPECT_EQ(rect->max_x, upper->x);
  EXPECT_EQ(rect->max_y, upper->y);
}
