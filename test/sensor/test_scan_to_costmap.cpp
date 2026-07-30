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

#include <eltanin/core/types.hpp>
#include <eltanin/map/cost_values.hpp>
#include <eltanin/map/layered_costmap.hpp>
#include <eltanin/map/layers/obstacle_layer.hpp>
#include <eltanin/sensor/scan_projection.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <vector>

namespace
{

using eltanin::Transform2D;
using eltanin::map::FREE_SPACE;
using eltanin::map::LayeredCostmap;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::MapGeometry;
using eltanin::map::ObstacleLayer;
using eltanin::sensor::project_scan;
using eltanin::sensor::ScanData;
using eltanin::sensor::ScanFilter;

constexpr double kPi = std::numbers::pi;
constexpr float kNanF = std::numeric_limits<float>::quiet_NaN();

MapGeometry test_geometry()
{
  return MapGeometry(20, 20, 0.1, Eigen::Vector2d{-1.0, -1.0});
}

/// Four beams on the axes; only beams 0 and 1 survive the effective range [0.1, 0.6].
ScanData reference_scan()
{
  ScanData scan;
  scan.angle_min = 0.0;
  scan.angle_increment = 0.5 * kPi;
  scan.range_min = 0.1;
  scan.range_max = 30.0;
  scan.ranges = {0.5F, 0.5F, kNanF, 0.8F, 0.05F};
  return scan;
}

ScanFilter reference_filter()
{
  ScanFilter filter;
  filter.min_range = 0.0;
  filter.max_range = 0.6;
  return filter;
}

std::size_t count_cost(const eltanin::map::Costmap & costmap, std::uint8_t cost)
{
  return static_cast<std::size_t>(
    std::count(costmap.data().begin(), costmap.data().end(), cost));
}

}  // namespace

TEST(ScanToCostmap, ProjectedPointsFeedTheObstacleLayerDirectly)
{
  LayeredCostmap costmap(test_geometry(), FREE_SPACE);
  ObstacleLayer & obstacles = costmap.add_layer<ObstacleLayer>();

  std::vector<Eigen::Vector2d> points;
  project_scan(
    reference_scan(), reference_filter(), Transform2D(Eigen::Vector2d{0.05, 0.05}, 0.0), points);
  ASSERT_EQ(points.size(), 2u);
  obstacles.set_points(points);
  costmap.update();

  EXPECT_EQ(costmap.costmap()(15, 10), LETHAL_OBSTACLE);
  EXPECT_EQ(costmap.costmap()(10, 15), LETHAL_OBSTACLE);
}

TEST(ScanToCostmap, FilteredBeamsLeaveTheirCellsFree)
{
  LayeredCostmap costmap(test_geometry(), FREE_SPACE);
  ObstacleLayer & obstacles = costmap.add_layer<ObstacleLayer>();

  std::vector<Eigen::Vector2d> points;
  project_scan(
    reference_scan(), reference_filter(), Transform2D(Eigen::Vector2d{0.05, 0.05}, 0.0), points);
  obstacles.set_points(points);
  costmap.update();

  EXPECT_EQ(costmap.costmap()(5, 10), FREE_SPACE);
  EXPECT_EQ(costmap.costmap()(10, 2), FREE_SPACE);
  EXPECT_EQ(costmap.costmap()(11, 10), FREE_SPACE);
}

TEST(ScanToCostmap, LethalCellCountEqualsTheSurvivingBeamCount)
{
  LayeredCostmap costmap(test_geometry(), FREE_SPACE);
  ObstacleLayer & obstacles = costmap.add_layer<ObstacleLayer>();

  std::vector<Eigen::Vector2d> points;
  project_scan(
    reference_scan(), reference_filter(), Transform2D(Eigen::Vector2d{0.05, 0.05}, 0.0), points);
  obstacles.set_points(points);
  costmap.update();

  EXPECT_EQ(count_cost(costmap.costmap(), LETHAL_OBSTACLE), points.size());
  EXPECT_EQ(
    count_cost(costmap.costmap(), FREE_SPACE), costmap.costmap().cell_count() - points.size());
}

TEST(ScanToCostmap, PointsOutsideTheMapAreDiscarded)
{
  LayeredCostmap costmap(test_geometry(), FREE_SPACE);
  ObstacleLayer & obstacles = costmap.add_layer<ObstacleLayer>();

  ScanData scan = reference_scan();
  scan.range_max = 100.0;
  scan.ranges = {50.0F, 0.5F, 50.0F, 50.0F};
  ScanFilter filter;
  filter.max_range = 100.0;

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, filter, Transform2D(Eigen::Vector2d{0.05, 0.05}, 0.0), points);
  ASSERT_EQ(points.size(), 4u);
  obstacles.set_points(points);
  costmap.update();

  EXPECT_EQ(count_cost(costmap.costmap(), LETHAL_OBSTACLE), 1u);
  EXPECT_EQ(costmap.costmap()(10, 15), LETHAL_OBSTACLE);
}

TEST(ScanToCostmap, GeometryIsUnchangedByTheUpdate)
{
  LayeredCostmap costmap(test_geometry(), FREE_SPACE);
  ObstacleLayer & obstacles = costmap.add_layer<ObstacleLayer>();
  const MapGeometry before = costmap.geometry();

  std::vector<Eigen::Vector2d> points;
  project_scan(
    reference_scan(), reference_filter(), Transform2D(Eigen::Vector2d{0.05, 0.05}, 0.0), points);
  obstacles.set_points(points);
  costmap.update();

  EXPECT_EQ(costmap.geometry(), before);
}

TEST(ScanToCostmap, EmptyProjectionLeavesTheMapAtTheDefaultCost)
{
  LayeredCostmap costmap(test_geometry(), FREE_SPACE);
  ObstacleLayer & obstacles = costmap.add_layer<ObstacleLayer>();

  ScanData scan = reference_scan();
  scan.ranges.assign(8, kNanF);

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, reference_filter(), Transform2D(Eigen::Vector2d{0.05, 0.05}, 0.0), points);
  ASSERT_TRUE(points.empty());
  obstacles.set_points(points);
  costmap.update();

  EXPECT_EQ(count_cost(costmap.costmap(), FREE_SPACE), costmap.costmap().cell_count());
}
