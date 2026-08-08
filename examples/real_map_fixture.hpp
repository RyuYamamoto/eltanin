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

#ifndef ELTANIN_EXAMPLES__REAL_MAP_FIXTURE_HPP_
#define ELTANIN_EXAMPLES__REAL_MAP_FIXTURE_HPP_

#include <eltanin/core/footprint.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/core/polygon.hpp>
#include <eltanin/core/traversability.hpp>
#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/crop.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/map/layers/inflation_layer.hpp>
#include <eltanin/map_io/map_loader.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ostream>
#include <optional>
#include <span>
#include <vector>

namespace eltanin_examples
{

/// Same robot as test/planner/test_planner_real_map.cpp, so the numbers stay comparable.
inline constexpr double INFLATION_RADIUS = 0.55;
inline constexpr double COST_SCALING_FACTOR = 10.0;

/// Cells kept around the drawn geometry when cropping, so surrounding obstacles stay visible.
inline constexpr int CROP_MARGIN_CELLS = 30;

/// Bound on the flood fill used to pick an automatic goal.
inline constexpr std::size_t MAX_FLOOD_FILL_CELLS = 200000;

inline eltanin::Polygon2D robot_footprint()
{
  return eltanin::Polygon2D{
    Eigen::Vector2d{0.22, 0.15}, Eigen::Vector2d{-0.22, 0.15}, Eigen::Vector2d{-0.22, -0.15},
    Eigen::Vector2d{0.22, -0.15}};
}

/// The footprint-derived models; the same robot on any map.
struct RobotModel
{
  eltanin::map::InflationCostModel inflation;
  eltanin::CollisionRadii radii;
  eltanin::map::CostTraversabilityModel model;
};

/// An inflated map together with everything derived from the footprint it was inflated for.
struct InflatedMap
{
  eltanin::map::Costmap map;
  eltanin::map::InflationCostModel inflation;
  eltanin::CollisionRadii radii;
  eltanin::map::CostTraversabilityModel model;
};

/// Radii and cost models for robot_footprint(); prints the reason and returns nullopt on failure.
inline std::optional<RobotModel> make_robot_model()
{
  const auto radii = eltanin::CollisionRadii::from_footprint(robot_footprint(), INFLATION_RADIUS);
  const auto inflation = radii.has_value()
                           ? eltanin::map::InflationCostModel::create(*radii, COST_SCALING_FACTOR)
                           : std::nullopt;
  if (!inflation.has_value()) {
    std::cerr << "failed to build the inflation model\n";
    return std::nullopt;
  }
  return RobotModel{
    *inflation, *radii,
    eltanin::map::CostTraversabilityModel(inflation->circumscribed_cost(), false)};
}

/// Loads the YAML map and leaves it uninflated, as a StaticLayer expects.
inline std::optional<eltanin::map::Costmap> load_raw_map(const std::filesystem::path & yaml)
{
  try {
    return eltanin::map_io::load_map(yaml);
  } catch (const eltanin::map_io::MapIoError & error) {
    std::cerr << "failed to load " << yaml << ": " << error.what() << '\n';
    return std::nullopt;
  }
}

/// Loads the YAML map and inflates it in place; prints the reason and returns nullopt on failure.
inline std::optional<InflatedMap> load_and_inflate(const std::filesystem::path & yaml)
{
  std::optional<eltanin::map::Costmap> costmap = load_raw_map(yaml);
  if (!costmap.has_value()) {
    return std::nullopt;
  }
  const std::optional<RobotModel> robot = make_robot_model();
  if (!robot.has_value()) {
    return std::nullopt;
  }
  eltanin::map::InflationLayer(robot->inflation, false).update_costs(*costmap);
  return InflatedMap{std::move(*costmap), robot->inflation, robot->radii, robot->model};
}

inline std::optional<eltanin::map::MapIndex> first_free_cell(
  const eltanin::map::Costmap & map, const eltanin::map::CostTraversabilityModel & model)
{
  for (int my = 0; my < map.size_y(); ++my) {
    for (int mx = 0; mx < map.size_x(); ++mx) {
      if (model.classify(map(mx, my)) == eltanin::Traversability::Free) {
        return eltanin::map::MapIndex{mx, my};
      }
    }
  }
  return std::nullopt;
}

/// Four-connected flood fill; its reachable set is a subset of what the planner can reach.
inline std::optional<eltanin::map::MapIndex> farthest_reachable_cell(
  const eltanin::map::Costmap & map, const eltanin::map::CostTraversabilityModel & model,
  const eltanin::map::MapIndex & from)
{
  const eltanin::map::MapGeometry & geometry = map.geometry();
  std::vector<std::uint8_t> visited(map.cell_count(), 0);
  std::deque<eltanin::map::MapIndex> queue;
  queue.push_back(from);
  visited[geometry.index(from.x, from.y)] = 1;

  eltanin::map::MapIndex last = from;
  std::size_t discovered = 1;
  constexpr std::array<eltanin::map::MapIndex, 4> neighbor_offsets{
    eltanin::map::MapIndex{1, 0}, eltanin::map::MapIndex{-1, 0},
    eltanin::map::MapIndex{0, 1}, eltanin::map::MapIndex{0, -1}};
  while (!queue.empty() && discovered < MAX_FLOOD_FILL_CELLS) {
    last = queue.front();
    queue.pop_front();
    for (const eltanin::map::MapIndex & offset : neighbor_offsets) {
      const int nx = last.x + offset.x;
      const int ny = last.y + offset.y;
      if (!geometry.in_bounds(nx, ny)) {
        continue;
      }
      const std::size_t index = geometry.index(nx, ny);
      if (visited[index] != 0 || model.classify(map(nx, ny)) != eltanin::Traversability::Free) {
        continue;
      }
      visited[index] = 1;
      ++discovered;
      queue.push_back(eltanin::map::MapIndex{nx, ny});
    }
  }
  if (discovered < 2) {
    return std::nullopt;
  }
  return last;
}

/// Start and goal positions picked automatically; nullopt when the map has no reachable pair.
inline std::optional<std::pair<eltanin::Pose2D, eltanin::Pose2D>> auto_start_goal(
  const eltanin::map::Costmap & map, const eltanin::map::CostTraversabilityModel & model)
{
  const auto start_cell = first_free_cell(map, model);
  if (!start_cell.has_value()) {
    std::cerr << "the inflated map has no traversable cell\n";
    return std::nullopt;
  }
  const auto goal_cell = farthest_reachable_cell(map, model, *start_cell);
  if (!goal_cell.has_value()) {
    std::cerr << "no cell is reachable from the automatic start\n";
    return std::nullopt;
  }
  const eltanin::map::MapGeometry & geometry = map.geometry();
  return std::pair{
    eltanin::Pose2D{geometry.map_to_world(start_cell->x, start_cell->y), 0.0},
    eltanin::Pose2D{geometry.map_to_world(goal_cell->x, goal_cell->y), 0.0}};
}

/// Cells of `map` covering every position, so the dumped image is not the whole 4000x4000 grid.
inline eltanin::map::Costmap crop_around(
  const eltanin::map::Costmap & map, std::span<const Eigen::Vector2d> positions,
  eltanin::map::MapIndex & lower_left)
{
  const std::optional<eltanin::map::CellRect> rect =
    eltanin::map::bounding_cells(map.geometry(), positions, CROP_MARGIN_CELLS);
  if (!rect.has_value()) {
    lower_left = eltanin::map::MapIndex{0, 0};
    return map;
  }
  lower_left = eltanin::map::MapIndex{rect->min_x, rect->min_y};
  return eltanin::map::crop(map, *rect);
}

inline std::vector<Eigen::Vector2d> positions_of(const eltanin::Path & path)
{
  std::vector<Eigen::Vector2d> positions;
  positions.reserve(path.size());
  for (const eltanin::Pose2D & pose : path) {
    positions.push_back(pose.position);
  }
  return positions;
}

inline bool write_path_csv(const std::filesystem::path & file, const eltanin::Path & path)
{
  std::ofstream out(file);
  if (!out) {
    return false;
  }
  out << "x,y,yaw\n";
  for (const eltanin::Pose2D & pose : path) {
    out << pose.position.x() << ',' << pose.position.y() << ',' << pose.yaw << '\n';
  }
  return static_cast<bool>(out);
}

/// Geometry of the cropped image plus the radii, which every plotting script needs.
inline void write_meta(
  std::ostream & meta, const eltanin::map::Costmap & crop,
  const eltanin::map::MapIndex & lower_left,
  const eltanin::map::InflationCostModel & inflation, const eltanin::CollisionRadii & radii)
{
  const eltanin::map::MapGeometry & geometry = crop.geometry();
  meta << "resolution " << geometry.resolution() << '\n'
       << "origin_x " << geometry.origin().x() << '\n'
       << "origin_y " << geometry.origin().y() << '\n'
       << "size_x " << geometry.size_x() << '\n'
       << "size_y " << geometry.size_y() << '\n'
       << "crop_offset_x " << lower_left.x << '\n'
       << "crop_offset_y " << lower_left.y << '\n'
       << "circumscribed_cost " << static_cast<int>(inflation.circumscribed_cost()) << '\n'
       << "inscribed_radius " << radii.inscribed_radius() << '\n'
       << "circumscribed_radius " << radii.circumscribed_radius() << '\n'
       << "inflation_radius " << radii.inflation_radius() << '\n';
}

inline void write_meta(
  std::ostream & meta, const eltanin::map::Costmap & crop,
  const eltanin::map::MapIndex & lower_left, const InflatedMap & inflated)
{
  write_meta(meta, crop, lower_left, inflated.inflation, inflated.radii);
}

/// Poses whose cell is not Free; the planner must produce none, a tracker may.
inline std::size_t count_non_free(
  const eltanin::Path & path, const eltanin::map::Costmap & map,
  const eltanin::map::CostTraversabilityModel & model)
{
  std::size_t count = 0;
  for (const eltanin::Pose2D & pose : path) {
    const auto index = map.geometry().world_to_map(pose.position);
    if (
      !index.has_value() ||
      model.classify(map(index->x, index->y)) != eltanin::Traversability::Free) {
      ++count;
    }
  }
  return count;
}

}  // namespace eltanin_examples

#endif  // ELTANIN_EXAMPLES__REAL_MAP_FIXTURE_HPP_
