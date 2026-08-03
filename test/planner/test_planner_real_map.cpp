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

#include <eltanin/core/footprint.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/core/polygon.hpp>
#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/map/layers/inflation_layer.hpp>
#include <eltanin/map_io/map_loader.hpp>
#include <eltanin/planner/astar_planner.hpp>
#include <eltanin/planner/path_smoother.hpp>

#include <planner/planner_fixture.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <iostream>
#include <optional>

namespace
{

using Eigen::Vector2d;
using eltanin::CollisionRadii;
using eltanin::Path;
using eltanin::Polygon2D;
using eltanin::Pose2D;
using eltanin::Traversability;
using eltanin::path_length;
using eltanin::map::Costmap;
using eltanin::map::CostTraversabilityModel;
using eltanin::map::InflationCostModel;
using eltanin::map::InflationLayer;
using eltanin::map::MapIndex;
using eltanin_test::expect_all_free;
using eltanin_test::expect_grid_connected;

/// Cap on the flood fill so the test stays bounded on a 16 M cell map.
constexpr std::size_t MAX_VISITED_CELLS = 200000;

/// Four-connected flood fill; its reachable set is a subset of what the planner can reach.
std::optional<MapIndex> farthest_reachable_cell(
  const Costmap & map, const CostTraversabilityModel & model, const MapIndex & from)
{
  const auto & geometry = map.geometry();
  std::vector<std::uint8_t> visited(map.cell_count(), 0);
  std::deque<MapIndex> queue;
  queue.push_back(from);
  visited[geometry.index(from.x, from.y)] = 1;

  MapIndex last = from;
  std::size_t discovered = 1;
  constexpr std::array<MapIndex, 4> neighbor_offsets{
    MapIndex{1, 0}, MapIndex{-1, 0}, MapIndex{0, 1}, MapIndex{0, -1}};
  while (!queue.empty() && discovered < MAX_VISITED_CELLS) {
    last = queue.front();
    queue.pop_front();
    for (const MapIndex & offset : neighbor_offsets) {
      const int nx = last.x + offset.x;
      const int ny = last.y + offset.y;
      if (!geometry.in_bounds(nx, ny)) {
        continue;
      }
      const std::size_t index = geometry.index(nx, ny);
      if (visited[index] != 0 || model.classify(map(nx, ny)) != Traversability::Free) {
        continue;
      }
      visited[index] = 1;
      ++discovered;
      queue.push_back(MapIndex{nx, ny});
    }
  }
  if (discovered < 2) {
    return std::nullopt;
  }
  return last;
}

std::optional<MapIndex> first_free_cell(
  const Costmap & map, const CostTraversabilityModel & model)
{
  for (int my = 0; my < map.size_y(); ++my) {
    for (int mx = 0; mx < map.size_x(); ++mx) {
      if (model.classify(map(mx, my)) == Traversability::Free) {
        return MapIndex{mx, my};
      }
    }
  }
  return std::nullopt;
}

}  // namespace

TEST(PlannerRealMap, PlansAndSmoothsOnTheInflatedReferenceMap)
{
  const std::filesystem::path yaml = std::filesystem::path(ELTANIN_TEST_MAP_DIR) / "map.yaml";
  if (!std::filesystem::is_regular_file(yaml)) {
    GTEST_SKIP() << "reference map not available at " << yaml;
  }

  Costmap costmap = eltanin::map_io::load_map(yaml);
  const Polygon2D footprint = {
    Vector2d{0.22, 0.15}, Vector2d{-0.22, 0.15}, Vector2d{-0.22, -0.15}, Vector2d{0.22, -0.15}};
  const auto radii = CollisionRadii::from_footprint(footprint, 0.55);
  ASSERT_TRUE(radii.has_value());
  const auto inflation = InflationCostModel::create(*radii, 10.0);
  ASSERT_TRUE(inflation.has_value());
  InflationLayer(*inflation, false).update_costs(costmap);

  const CostTraversabilityModel model(inflation->circumscribed_cost(), false);
  const auto start_cell = first_free_cell(costmap, model);
  ASSERT_TRUE(start_cell.has_value());
  const auto goal_cell = farthest_reachable_cell(costmap, model, *start_cell);
  ASSERT_TRUE(goal_cell.has_value());

  const Pose2D start{costmap.geometry().map_to_world(start_cell->x, start_cell->y), 0.0};
  const Pose2D goal{costmap.geometry().map_to_world(goal_cell->x, goal_cell->y), 1.25};

  const auto raw = eltanin::planner::plan_astar(
    costmap, model, start, goal, eltanin_test::raw_astar_params());
  ASSERT_TRUE(raw.has_value());
  expect_grid_connected(*raw, costmap.geometry().resolution());
  expect_all_free(*raw, costmap, model);
  EXPECT_EQ((*raw)[raw->size() - 1].yaw, 1.25);

  const Path smoothed = eltanin::planner::smooth(*raw, costmap, model);
  EXPECT_EQ(smoothed.size(), raw->size());
  expect_all_free(smoothed, costmap, model);

  std::cout << "start " << start_cell->x << "," << start_cell->y << " goal " << goal_cell->x << ","
            << goal_cell->y << " poses " << raw->size() << " raw length " << path_length(*raw)
            << " smoothed length " << path_length(smoothed) << std::endl;
}
