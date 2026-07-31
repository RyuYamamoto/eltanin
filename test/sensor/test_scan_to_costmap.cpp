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
#include <eltanin/map/layers/raytrace_layer.hpp>
#include <eltanin/map/layers/static_layer.hpp>
#include <eltanin/sensor/scan_clearing.hpp>
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
using eltanin::map::Costmap;
using eltanin::map::FREE_SPACE;
using eltanin::map::LayeredCostmap;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::MapGeometry;
using eltanin::map::NO_INFORMATION;
using eltanin::map::ObstacleLayer;
using eltanin::map::RaytraceLayer;
using eltanin::map::StaticLayer;
using eltanin::sensor::project_scan;
using eltanin::sensor::project_scan_for_clearing;
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

std::size_t count_cost(const Costmap & costmap, std::uint8_t cost)
{
  return static_cast<std::size_t>(
    std::count(costmap.data().begin(), costmap.data().end(), cost));
}

/// The sensor sits at the centre cell of the window, exactly on a cell centre.
const Eigen::Vector2d kSensorPosition{0.05, 0.05};
constexpr float kInfF = std::numeric_limits<float>::infinity();
constexpr double kMarkingMaxRange = 1.0;
constexpr double kClearingMaxRange = 0.6;

/// One hit inside the clearing range, one beam with no return, one hit beyond it, one too close.
ScanData local_map_scan()
{
  ScanData scan;
  scan.angle_min = 0.0;
  scan.angle_increment = 0.5 * kPi;
  scan.range_min = 0.1;
  scan.range_max = 30.0;
  scan.ranges = {0.5F, kInfF, 0.8F, 0.05F};
  return scan;
}

ScanFilter local_map_filter()
{
  ScanFilter filter;
  filter.min_range = 0.0;
  filter.max_range = kMarkingMaxRange;
  return filter;
}

/// The static map of a real robot is mostly unexplored; only the wall cell is known here.
Costmap static_window_with_a_wall()
{
  Costmap map(test_geometry(), NO_INFORMATION);
  map(12, 10) = LETHAL_OBSTACLE;
  return map;
}

/// The local map stack of the ROS layer: unknown by default, static first, then the observation.
struct LocalMap
{
  LocalMap() : layers(test_geometry(), NO_INFORMATION)
  {
    layers.add_layer<StaticLayer>(static_window_with_a_wall());
    raytrace = &layers.add_layer<RaytraceLayer>();
  }

  std::uint8_t cost_at(int mx, int my) const { return layers.costmap()(mx, my); }

  LayeredCostmap layers;
  RaytraceLayer * raytrace{nullptr};
};

void project_local_map_scan(
  std::vector<Eigen::Vector2d> & marks, std::vector<Eigen::Vector2d> & endpoints)
{
  const Transform2D sensor_to_world(kSensorPosition, 0.0);
  project_scan(local_map_scan(), local_map_filter(), sensor_to_world, marks);
  project_scan_for_clearing(
    local_map_scan(), local_map_filter(), kClearingMaxRange, sensor_to_world, endpoints);
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
TEST(ScanToLocalMap, ClearingAndMarkingProduceThreeValues)
{
  LocalMap local;
  std::vector<Eigen::Vector2d> marks;
  std::vector<Eigen::Vector2d> endpoints;
  project_local_map_scan(marks, endpoints);
  ASSERT_EQ(marks.size(), 2u);
  ASSERT_EQ(endpoints.size(), 3u);

  local.raytrace->set_observation(kSensorPosition, marks, endpoints);
  local.layers.update();

  EXPECT_EQ(local.cost_at(15, 10), LETHAL_OBSTACLE);
  EXPECT_EQ(local.cost_at(2, 10), LETHAL_OBSTACLE);
  EXPECT_EQ(local.cost_at(11, 10), FREE_SPACE);
  EXPECT_EQ(local.cost_at(13, 10), FREE_SPACE);
  EXPECT_EQ(local.cost_at(14, 10), FREE_SPACE);
  EXPECT_EQ(local.cost_at(10, 15), FREE_SPACE);
  EXPECT_GT(count_cost(local.layers.costmap(), NO_INFORMATION), 0u);
}

TEST(ScanToLocalMap, TheStaticWallSurvivesAClearingRayThatCrossesIt)
{
  LocalMap local;
  std::vector<Eigen::Vector2d> marks;
  std::vector<Eigen::Vector2d> endpoints;
  project_local_map_scan(marks, endpoints);

  local.raytrace->set_observation(kSensorPosition, {}, endpoints);
  local.layers.update();

  EXPECT_EQ(local.cost_at(12, 10), LETHAL_OBSTACLE);
  EXPECT_EQ(local.cost_at(13, 10), FREE_SPACE);
}

TEST(ScanToLocalMap, CellsBeyondTheClearingRangeStayUnknown)
{
  LocalMap local;
  std::vector<Eigen::Vector2d> marks;
  std::vector<Eigen::Vector2d> endpoints;
  project_local_map_scan(marks, endpoints);

  local.raytrace->set_observation(kSensorPosition, marks, endpoints);
  local.layers.update();

  EXPECT_EQ(local.cost_at(4, 10), NO_INFORMATION);
  EXPECT_EQ(local.cost_at(3, 10), NO_INFORMATION);
  EXPECT_EQ(local.cost_at(10, 16), NO_INFORMATION);
  EXPECT_EQ(local.cost_at(10, 5), NO_INFORMATION);
}

TEST(ScanToLocalMap, AStaleScanLeavesTheStaticWindowAlone)
{
  LocalMap local;
  std::vector<Eigen::Vector2d> marks;
  std::vector<Eigen::Vector2d> endpoints;
  project_local_map_scan(marks, endpoints);
  local.raytrace->set_observation(kSensorPosition, marks, endpoints);
  local.layers.update();

  local.raytrace->clear_observation();
  local.layers.update();

  EXPECT_EQ(local.cost_at(12, 10), LETHAL_OBSTACLE);
  EXPECT_EQ(count_cost(local.layers.costmap(), FREE_SPACE), 0u);
  EXPECT_EQ(
    count_cost(local.layers.costmap(), NO_INFORMATION), local.layers.costmap().cell_count() - 1u);
}
