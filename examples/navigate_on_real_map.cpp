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

#include <navigation_loop.hpp>

#include <eltanin/map/grid_map.hpp>

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <utility>

namespace
{

using eltanin::Pose2D;
using eltanin_examples::LegStats;
using eltanin_examples::NavigateConfig;
using eltanin_examples::NavigateResult;
using eltanin_examples::PlannerType;

#ifdef ELTANIN_TEST_MAP_DIR
constexpr const char * DEFAULT_MAP_DIR = ELTANIN_TEST_MAP_DIR;
#else
constexpr const char * DEFAULT_MAP_DIR = "";
#endif

int usage(const char * program)
{
  std::cerr
    << "usage: " << program << " <output_dir> [options]\n"
    << "  Runs the whole eltanin navigation stack in one process, without ROS: map_io ->\n"
    << "  LayeredCostmap -> selected global planner -> selected follower ->\n"
    << "  VelocityLimiter -> SimpleSimulator, replanning once the path is blocked.\n"
    << "options:\n"
    << "  --map <map.yaml>           default: " << DEFAULT_MAP_DIR << "/map.yaml\n"
    << "  --start <x> <y>            default: picked from the map, with --goal\n"
    << "  --goal <x> <y>             default: picked from the map, with --start\n"
    << "  --planner <name>           astar (default) or hybrid-astar\n"
    << "  --follower <name>          pure_pursuit (default) or mpc\n"
    << "  --velocity-profile         cap the speed by the path curvature\n"
    << "  --obstacle-fraction <f>    where the unknown obstacle sits on the path; 0 disables it\n"
    << "  --obstacle-half-width <n>  half width of that obstacle [cells]\n"
    << "  --dt <s>                   control period\n"
    << "  --sensor-decimation <n>    control cycles between two scans\n"
    << "  --replan-on-stop-only      do not replan when the observations block the path, only\n"
    << "                             once the limiter has brought the robot to a standstill\n"
    << "  Writes costmap.pgm, traversed.pgm, path.csv, trajectory.csv, obstacles.csv and\n"
    << "  meta.txt into <output_dir>.\n";
  return EXIT_FAILURE;
}

struct Arguments
{
  std::filesystem::path output_dir;
  std::filesystem::path map;
  NavigateConfig config;
};

/// True when `count` more arguments follow the option at `index`.
bool has_values(int argc, int index, int count) { return index + count < argc; }

/// nullopt when the command line is unusable; the caller then prints the usage.
std::optional<Arguments> parse(int argc, char ** argv)
{
  if (argc < 2 || argv[1][0] == '-') {
    return std::nullopt;
  }
  Arguments arguments;
  arguments.output_dir = argv[1];
  arguments.map = std::filesystem::path(DEFAULT_MAP_DIR) / "map.yaml";

  std::optional<Pose2D> start;
  std::optional<Pose2D> goal;
  try {
    for (int i = 2; i < argc; ++i) {
      const std::string option = argv[i];
      if (option == "--map" && has_values(argc, i, 1)) {
        arguments.map = argv[++i];
      } else if (option == "--start" && has_values(argc, i, 2)) {
        start = Pose2D{Eigen::Vector2d{std::stod(argv[i + 1]), std::stod(argv[i + 2])}, 0.0};
        i += 2;
      } else if (option == "--goal" && has_values(argc, i, 2)) {
        goal = Pose2D{Eigen::Vector2d{std::stod(argv[i + 1]), std::stod(argv[i + 2])}, 0.0};
        i += 2;
      } else if (option == "--planner" && has_values(argc, i, 1)) {
        const std::string name = argv[++i];
        if (name == "astar") {
          arguments.config.planner = PlannerType::AStar;
        } else if (name == "hybrid-astar") {
          arguments.config.planner = PlannerType::HybridAStar;
        } else {
          return std::nullopt;
        }
      } else if (option == "--follower" && has_values(argc, i, 1)) {
        const std::optional<eltanin::control::FollowerType> type =
          eltanin::control::to_follower_type(argv[++i]);
        if (!type.has_value()) {
          return std::nullopt;
        }
        arguments.config.follower.type = *type;
      } else if (option == "--velocity-profile") {
        const eltanin::control::VelocityProfileParams profile;
        arguments.config.follower.pure_pursuit.velocity_profile = profile;
#ifdef ELTANIN_WITH_MPC
        arguments.config.follower.mpc.velocity_profile = profile;
#endif
      } else if (option == "--obstacle-fraction" && has_values(argc, i, 1)) {
        arguments.config.obstacle_fraction = std::stod(argv[++i]);
      } else if (option == "--obstacle-half-width" && has_values(argc, i, 1)) {
        arguments.config.obstacle_half_width_cells = std::stoi(argv[++i]);
      } else if (option == "--dt" && has_values(argc, i, 1)) {
        arguments.config.control_dt = std::stod(argv[++i]);
      } else if (option == "--sensor-decimation" && has_values(argc, i, 1)) {
        arguments.config.sensor_decimation = std::stoi(argv[++i]);
      } else if (option == "--replan-on-stop-only") {
        arguments.config.replan_on_blocked_path = false;
      } else {
        return std::nullopt;
      }
    }
  } catch (const std::exception &) {
    return std::nullopt;
  }

  if (start.has_value() != goal.has_value()) {
    std::cerr << "--start and --goal have to be given together\n";
    return std::nullopt;
  }
  if (start.has_value()) {
    arguments.config.start_goal = std::pair{*start, *goal};
  }
  if (arguments.config.control_dt <= 0.0 || arguments.config.sensor_decimation < 1) {
    std::cerr << "--dt has to be positive and --sensor-decimation at least 1\n";
    return std::nullopt;
  }
  return arguments;
}

void print_summary(
  const std::filesystem::path & map, const eltanin::map::Costmap & static_map,
  const NavigateConfig & config, const NavigateResult & result)
{
  const eltanin::map::MapGeometry & geometry = static_map.geometry();
  std::cout << "planner          " << eltanin_examples::planner_name(config.planner) << '\n'
            << "map              " << map << "  " << geometry.size_x() << " x " << geometry.size_y()
            << "  res " << geometry.resolution() << "  origin (" << geometry.origin().x() << ", "
            << geometry.origin().y() << ")\n"
            << "start            (" << result.start.position.x() << ", "
            << result.start.position.y() << ")   goal (" << result.goal.position.x() << ", "
            << result.goal.position.y() << ")\n";
  if (result.obstacle_centre.has_value()) {
    std::cout << "obstacle         (" << result.obstacle_centre->x() << ", "
              << result.obstacle_centre->y() << ")  half width " << result.obstacle_half_width
              << " m\n";
  }
  for (std::size_t leg = 0; leg < result.legs.size(); ++leg) {
    const LegStats & stats = result.legs[leg];
    std::cout << "leg " << leg << "   path " << stats.path_poses << " poses, " << stats.path_length
              << " m | cycles " << stats.cycles << " | limited " << stats.limited_cycles
              << " | collision " << stats.collision_cycles << " | truncated "
              << stats.truncated_cycles << '\n'
              << "        min collision distance " << stats.min_collision_distance
              << " m | max speed loss " << stats.max_speed_loss << " m/s | max path deviation "
              << stats.max_path_deviation << " m\n";
  }
  std::cout << "total   cycles " << result.samples.size() << " | sim time " << result.sim_time
            << " s | replans " << result.replans << " (" << result.replans_on_blocked_path
            << " from observations) | global updates " << result.global_updates
            << " | observations " << result.observations.size() << '\n'
            << "        final position error " << result.final_position_error << " m (tolerance "
            << config.goal_tolerance << " m) -> " << eltanin_examples::outcome_name(result.outcome)
            << '\n'
            << "        traversed poses in collision " << result.colliding_poses << " / "
            << result.samples.size() << '\n'
            << "        window clamped cycles " << result.window_clamped_cycles << '\n';
  if (std::isfinite(result.stop_clearance)) {
    std::cout << "        stop clearance " << result.stop_clearance << " m to any obstacle";
    if (result.obstacle_centre.has_value()) {
      std::cout << ", " << result.stop_obstacle_clearance << " m to the injected one";
    }
    std::cout << " (collision margin " << config.limiter.collision_margin << " m)\n";
  } else {
    std::cout << "        the robot never came to a standstill\n";
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::optional<Arguments> arguments = parse(argc, argv);
  if (!arguments.has_value()) {
    return usage(argv[0]);
  }

  const std::optional<eltanin::map::Costmap> static_map =
    eltanin_examples::load_raw_map(arguments->map);
  const std::optional<eltanin_examples::RobotModel> robot = eltanin_examples::make_robot_model();
  if (!static_map.has_value() || !robot.has_value()) {
    return EXIT_FAILURE;
  }

  const NavigateResult result =
    eltanin_examples::navigate(*static_map, *robot, arguments->config);
  print_summary(arguments->map, *static_map, arguments->config, result);

  if (!eltanin_examples::write_output_files(
        arguments->output_dir, arguments->config, *robot, result)) {
    return EXIT_FAILURE;
  }
  std::cout << "wrote costmap.pgm, traversed.pgm, path.csv, trajectory.csv, obstacles.csv,"
            << " meta.txt into " << arguments->output_dir << '\n';

  if (!result.reached()) {
    std::cerr << eltanin_examples::outcome_name(result.outcome) << ": " << result.message << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
