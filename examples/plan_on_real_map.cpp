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

#include <real_map_fixture.hpp>

#include <eltanin/core/path.hpp>
#include <eltanin/planner/astar_planner.hpp>
#include <eltanin/planner/path_smoother.hpp>
#include <eltanin/map_io/pgm.hpp>

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

using eltanin::Path;
using eltanin::Pose2D;
using eltanin::map::Costmap;
using eltanin::map::MapGeometry;
using eltanin::map::MapIndex;
using eltanin_examples::count_non_free;
using eltanin_examples::crop_around;
using eltanin_examples::positions_of;
using eltanin_examples::write_meta;
using eltanin_examples::write_path_csv;

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

  const auto inflated = eltanin_examples::load_and_inflate(yaml);
  if (!inflated.has_value()) {
    return EXIT_FAILURE;
  }
  const Costmap & costmap = inflated->map;
  const auto & model = inflated->model;

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
    const auto pair = eltanin_examples::auto_start_goal(costmap, model);
    if (!pair.has_value()) {
      return EXIT_FAILURE;
    }
    start = pair->first;
    goal = pair->second;
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
  const Costmap crop = crop_around(costmap, positions_of(*raw), lower_left);
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
  write_meta(meta, crop, lower_left, *inflated);

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
