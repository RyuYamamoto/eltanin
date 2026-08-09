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

#include <eltanin/core/angle.hpp>
#include <eltanin/map_io/pgm.hpp>
#include <eltanin/planner/astar_planner.hpp>
#include <eltanin/planner/hybrid_astar_planner.hpp>

#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <string>
#include <iostream>

namespace
{

constexpr double TURNING_RADIUS = 0.4;

int usage(const char * program)
{
  std::cerr << "usage: " << program
            << " <map.yaml> <output_dir> <start_x> <start_y> <start_yaw>"
               " <goal_x> <goal_y> <goal_yaw> [allow_reverse 0|1] [allow_turn_in_place 0|1]\n";
  return EXIT_FAILURE;
}

bool write_path(const std::filesystem::path & file, const eltanin::Path & path)
{
  std::ofstream out(file);
  if (!out) {
    return false;
  }
  out << "index,x,y,yaw,s,curvature\n";
  double s = 0.0;
  out << "0," << path[0].position.x() << ',' << path[0].position.y() << ',' << path[0].yaw
      << ",0,0\n";
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double ds = (path[i].position - path[i - 1].position).norm();
    s += ds;
    const double delta_yaw =
      eltanin::shortest_angular_distance(path[i - 1].yaw, path[i].yaw);
    const double curvature = ds > 0.0 ? 2.0 * std::sin(0.5 * delta_yaw) / ds : 0.0;
    out << i << ',' << path[i].position.x() << ',' << path[i].position.y() << ',' << path[i].yaw
        << ',' << s << ',' << curvature << '\n';
  }
  return static_cast<bool>(out);
}

bool write_footprint(const std::filesystem::path & file, const eltanin::Polygon2D & footprint)
{
  std::ofstream out(file);
  if (!out) {
    return false;
  }
  out << "x,y\n";
  for (const Eigen::Vector2d & vertex : footprint.vertices()) {
    out << vertex.x() << ',' << vertex.y() << '\n';
  }
  return static_cast<bool>(out);
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 9 || argc > 11) {
    return usage(argv[0]);
  }

  const std::filesystem::path yaml = argv[1];
  const std::filesystem::path output_dir = argv[2];
  eltanin::Pose2D start;
  eltanin::Pose2D goal;
  try {
    start = eltanin::Pose2D{
      Eigen::Vector2d{std::stod(argv[3]), std::stod(argv[4])}, std::stod(argv[5])};
    goal = eltanin::Pose2D{
      Eigen::Vector2d{std::stod(argv[6]), std::stod(argv[7])}, std::stod(argv[8])};
  } catch (const std::exception &) {
    return usage(argv[0]);
  }
  if (!start.position.allFinite() || !goal.position.allFinite() || !std::isfinite(start.yaw) ||
      !std::isfinite(goal.yaw)) {
    return usage(argv[0]);
  }

  const auto inflated = eltanin_examples::load_and_inflate(yaml);
  if (!inflated.has_value()) {
    return EXIT_FAILURE;
  }

  eltanin::planner::AStarParams guide_params;
  guide_params.smoother.reset();
  const auto guide =
    eltanin::planner::plan_astar(inflated->map, inflated->model, start, goal, guide_params);
  if (!guide) {
    std::cerr << "A* found no corridor between start and goal: "
              << eltanin::planner::to_string(guide.error()) << '\n';
    return EXIT_FAILURE;
  }

  eltanin::map::MapIndex lower_left{0, 0};
  const eltanin::map::Costmap costmap = eltanin_examples::crop_around(
    inflated->map, eltanin_examples::positions_of(*guide), lower_left);
  eltanin::planner::HybridAStarParams params;
  params.motion_model.minimum_turning_radius = TURNING_RADIUS;
  params.common.footprint = eltanin_examples::robot_footprint();
  if (argc >= 10) {
    params.motion_model.reverse = std::string(argv[9]) == "1";
  }
  if (argc >= 11) {
    params.motion_model.turn_in_place = std::string(argv[10]) == "1";
  }

  const auto path =
    eltanin::planner::plan_hybrid_astar(costmap, inflated->model, start, goal, params);
  if (!path) {
    std::cerr << "Hybrid A* found no path in the cropped corridor: "
              << eltanin::planner::to_string(path.error()) << '\n';
    return EXIT_FAILURE;
  }

  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  if (error) {
    std::cerr << "failed to create " << output_dir << '\n';
    return EXIT_FAILURE;
  }
  try {
    eltanin::map_io::write_pgm(output_dir / "costmap.pgm", costmap);
  } catch (const eltanin::map_io::MapIoError & exception) {
    std::cerr << exception.what() << '\n';
    return EXIT_FAILURE;
  }

  const eltanin::Polygon2D footprint = eltanin_examples::robot_footprint();
  if (!write_path(output_dir / "path.csv", *path) ||
      !write_footprint(output_dir / "footprint.csv", footprint)) {
    std::cerr << "failed to write CSV output\n";
    return EXIT_FAILURE;
  }

  std::ofstream meta(output_dir / "meta.txt");
  if (!meta) {
    return EXIT_FAILURE;
  }
  eltanin_examples::write_meta(meta, costmap, lower_left, *inflated);
  meta << "turning_radius " << TURNING_RADIUS << '\n'
       << "start_x " << start.position.x() << '\n'
       << "start_y " << start.position.y() << '\n'
       << "start_yaw " << start.yaw << '\n'
       << "goal_x " << goal.position.x() << '\n'
       << "goal_y " << goal.position.y() << '\n'
       << "goal_yaw " << goal.yaw << '\n';

  const auto & geometry = costmap.geometry();
  std::cout << "map " << yaml << '\n'
            << "crop " << geometry.size_x() << " x " << geometry.size_y() << " cells @ "
            << geometry.resolution() << " m\n"
            << "poses " << path->size() << " length " << eltanin::path_length(*path) << " m\n"
            << "wrote costmap.pgm, path.csv, footprint.csv and meta.txt into " << output_dir
            << '\n';
  return EXIT_SUCCESS;
}
