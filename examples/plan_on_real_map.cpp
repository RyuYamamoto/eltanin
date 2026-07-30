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
#include <eltanin/core/traversability.hpp>
#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/map/layers/inflation_layer.hpp>
#include <eltanin/map_io/map_loader.hpp>
#include <eltanin/map_io/pgm.hpp>
#include <eltanin/planner/astar_planner.hpp>
#include <eltanin/planner/path_smoother.hpp>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace
{

using eltanin::CollisionRadii;
using eltanin::Path;
using eltanin::Polygon2D;
using eltanin::Pose2D;
using eltanin::Traversability;
using eltanin::map::Costmap;
using eltanin::map::CostTraversabilityModel;
using eltanin::map::InflationCostModel;
using eltanin::map::InflationLayer;
using eltanin::map::MapGeometry;
using eltanin::map::MapIndex;

/// Same robot as test/planner/test_planner_real_map.cpp, so the artifacts stay comparable.
constexpr double INFLATION_RADIUS = 0.55;
constexpr double COST_SCALING_FACTOR = 10.0;

/// Cells kept around the path when cropping, so the surrounding obstacles stay visible.
constexpr int CROP_MARGIN_CELLS = 30;

/// Bound on the flood fill used to pick an automatic goal.
constexpr std::size_t MAX_FLOOD_FILL_CELLS = 200000;

std::optional<MapIndex> first_free_cell(const Costmap & map, const CostTraversabilityModel & model)
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

/// Four-connected flood fill; its reachable set is a subset of what the planner can reach.
std::optional<MapIndex> farthest_reachable_cell(
  const Costmap & map, const CostTraversabilityModel & model, const MapIndex & from)
{
  const MapGeometry & geometry = map.geometry();
  std::vector<std::uint8_t> visited(map.cell_count(), 0);
  std::deque<MapIndex> queue;
  queue.push_back(from);
  visited[geometry.index(from.x, from.y)] = 1;

  MapIndex last = from;
  std::size_t discovered = 1;
  constexpr int DX[4] = {1, -1, 0, 0};
  constexpr int DY[4] = {0, 0, 1, -1};
  while (!queue.empty() && discovered < MAX_FLOOD_FILL_CELLS) {
    last = queue.front();
    queue.pop_front();
    for (int k = 0; k < 4; ++k) {
      const int nx = last.x + DX[k];
      const int ny = last.y + DY[k];
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

/// Cells of `map` around the path, so the dumped image is not the whole 4000x4000 grid.
Costmap crop_around(const Costmap & map, const Path & path, MapIndex & lower_left)
{
  const MapGeometry & geometry = map.geometry();
  int min_x = map.size_x();
  int min_y = map.size_y();
  int max_x = 0;
  int max_y = 0;
  for (const Pose2D & pose : path) {
    const auto index = geometry.world_to_map(pose.position);
    if (!index.has_value()) {
      continue;
    }
    min_x = std::min(min_x, index->x);
    max_x = std::max(max_x, index->x);
    min_y = std::min(min_y, index->y);
    max_y = std::max(max_y, index->y);
  }
  min_x = std::max(0, min_x - CROP_MARGIN_CELLS);
  min_y = std::max(0, min_y - CROP_MARGIN_CELLS);
  max_x = std::min(map.size_x() - 1, max_x + CROP_MARGIN_CELLS);
  max_y = std::min(map.size_y() - 1, max_y + CROP_MARGIN_CELLS);

  const int size_x = max_x - min_x + 1;
  const int size_y = max_y - min_y + 1;
  const Eigen::Vector2d origin =
    geometry.origin() + Eigen::Vector2d{
                          static_cast<double>(min_x) * geometry.resolution(),
                          static_cast<double>(min_y) * geometry.resolution()};

  Costmap crop(MapGeometry(size_x, size_y, geometry.resolution(), origin));
  for (int my = 0; my < size_y; ++my) {
    for (int mx = 0; mx < size_x; ++mx) {
      crop(mx, my) = map(min_x + mx, min_y + my);
    }
  }
  lower_left = MapIndex{min_x, min_y};
  return crop;
}

bool write_path_csv(const std::filesystem::path & file, const Path & path)
{
  std::ofstream out(file);
  if (!out) {
    return false;
  }
  out << "x,y,yaw\n";
  for (const Pose2D & pose : path) {
    out << pose.position.x() << ',' << pose.position.y() << ',' << pose.yaw << '\n';
  }
  return static_cast<bool>(out);
}

/// Poses whose cell is classified Circumscribed; the planner must never produce any.
std::size_t count_non_free(
  const Path & path, const Costmap & map, const CostTraversabilityModel & model)
{
  std::size_t count = 0;
  for (const Pose2D & pose : path) {
    const auto index = map.geometry().world_to_map(pose.position);
    if (!index.has_value() || model.classify(map(index->x, index->y)) != Traversability::Free) {
      ++count;
    }
  }
  return count;
}

int usage(const char * program)
{
  std::cerr << "usage: " << program << " <map.yaml> <output_dir>"
            << " [start_x start_y goal_x goal_y]\n"
            << "  Without explicit poses, a reachable start/goal pair is picked automatically.\n"
            << "  Writes crop.pgm, raw.csv, smoothed.csv and meta.txt into <output_dir>.\n";
  return EXIT_FAILURE;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3 && argc != 7) {
    return usage(argv[0]);
  }
  const std::filesystem::path yaml = argv[1];
  const std::filesystem::path output_dir = argv[2];

  Costmap costmap;
  try {
    costmap = eltanin::map_io::load_map(yaml);
  } catch (const eltanin::map_io::MapIoError & error) {
    std::cerr << "failed to load " << yaml << ": " << error.what() << '\n';
    return EXIT_FAILURE;
  }

  const Polygon2D footprint = {
    Eigen::Vector2d{0.22, 0.15}, Eigen::Vector2d{-0.22, 0.15}, Eigen::Vector2d{-0.22, -0.15},
    Eigen::Vector2d{0.22, -0.15}};
  const auto radii = CollisionRadii::from_footprint(footprint, INFLATION_RADIUS);
  const auto inflation =
    radii.has_value() ? InflationCostModel::create(*radii, COST_SCALING_FACTOR) : std::nullopt;
  if (!inflation.has_value()) {
    std::cerr << "failed to build the inflation model\n";
    return EXIT_FAILURE;
  }
  InflationLayer(*inflation, false).update_costs(costmap);
  const CostTraversabilityModel model(inflation->circumscribed_cost(), false);
  const MapGeometry & geometry = costmap.geometry();

  Pose2D start;
  Pose2D goal;
  if (argc == 7) {
    try {
      start.position = Eigen::Vector2d{std::stod(argv[3]), std::stod(argv[4])};
      goal.position = Eigen::Vector2d{std::stod(argv[5]), std::stod(argv[6])};
    } catch (const std::exception &) {
      return usage(argv[0]);
    }
  } else {
    const auto start_cell = first_free_cell(costmap, model);
    if (!start_cell.has_value()) {
      std::cerr << "the inflated map has no traversable cell\n";
      return EXIT_FAILURE;
    }
    const auto goal_cell = farthest_reachable_cell(costmap, model, *start_cell);
    if (!goal_cell.has_value()) {
      std::cerr << "no cell is reachable from the automatic start\n";
      return EXIT_FAILURE;
    }
    start.position = geometry.map_to_world(start_cell->x, start_cell->y);
    goal.position = geometry.map_to_world(goal_cell->x, goal_cell->y);
  }

  const auto raw = eltanin::planner::plan(costmap, model, start, goal);
  if (!raw.has_value()) {
    std::cerr << "plan() found no path\n";
    return EXIT_FAILURE;
  }
  const Path smoothed = eltanin::planner::smooth(*raw, costmap, model);

  std::error_code directory_error;
  std::filesystem::create_directories(output_dir, directory_error);
  MapIndex lower_left{0, 0};
  const Costmap crop = crop_around(costmap, *raw, lower_left);
  try {
    eltanin::map_io::write_pgm(output_dir / "crop.pgm", crop);
  } catch (const eltanin::map_io::MapIoError & error) {
    std::cerr << "failed to write crop.pgm: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  if (!write_path_csv(output_dir / "raw.csv", *raw) ||
      !write_path_csv(output_dir / "smoothed.csv", smoothed)) {
    std::cerr << "failed to write the path CSV files\n";
    return EXIT_FAILURE;
  }

  const MapGeometry & crop_geometry = crop.geometry();
  std::ofstream meta(output_dir / "meta.txt");
  if (!meta) {
    std::cerr << "failed to write meta.txt\n";
    return EXIT_FAILURE;
  }
  meta << "resolution " << crop_geometry.resolution() << '\n'
       << "origin_x " << crop_geometry.origin().x() << '\n'
       << "origin_y " << crop_geometry.origin().y() << '\n'
       << "size_x " << crop_geometry.size_x() << '\n'
       << "size_y " << crop_geometry.size_y() << '\n'
       << "crop_offset_x " << lower_left.x << '\n'
       << "crop_offset_y " << lower_left.y << '\n'
       << "circumscribed_cost " << static_cast<int>(inflation->circumscribed_cost()) << '\n'
       << "inscribed_radius " << radii->inscribed_radius() << '\n'
       << "circumscribed_radius " << radii->circumscribed_radius() << '\n'
       << "inflation_radius " << radii->inflation_radius() << '\n';

  double max_displacement = 0.0;
  for (std::size_t i = 0; i < raw->size(); ++i) {
    max_displacement =
      std::max(max_displacement, (smoothed[i].position - (*raw)[i].position).norm());
  }

  std::cout << "crop " << crop_geometry.size_x() << " x " << crop_geometry.size_y() << " cells @ "
            << crop_geometry.resolution() << " m\n"
            << "poses " << raw->size() << "  raw length " << eltanin::path_length(*raw)
            << " m  smoothed length " << eltanin::path_length(smoothed) << " m\n"
            << "max smoother displacement " << max_displacement << " m = "
            << max_displacement / crop_geometry.resolution() << " cells\n"
            << "non-free poses: raw " << count_non_free(*raw, costmap, model) << ", smoothed "
            << count_non_free(smoothed, costmap, model) << '\n'
            << "wrote crop.pgm, raw.csv, smoothed.csv, meta.txt into " << output_dir << '\n';
  return EXIT_SUCCESS;
}
