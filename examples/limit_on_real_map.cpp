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

#include <eltanin/collision/velocity_limiter.hpp>
#include <eltanin/control/pure_pursuit.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/core/polygon.hpp>
#include <eltanin/map_io/pgm.hpp>
#include <eltanin/planner/astar_planner.hpp>
#include <eltanin/planner/path_smoother.hpp>
#include <eltanin/sim/simple_simulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace
{

using eltanin::Path;
using eltanin::Polygon2D;
using eltanin::Pose2D;
using eltanin::Twist2D;
using eltanin::collision::VelocityLimiter;
using eltanin::collision::VelocityLimiterParams;
using eltanin::control::PurePursuit;
using eltanin::control::PurePursuitParams;
using eltanin::map::Costmap;
using eltanin::map::MapIndex;
using eltanin::sim::SimpleSimulator;
using eltanin_examples::crop_around;
using eltanin_examples::positions_of;
using eltanin_examples::write_meta;
using eltanin_examples::write_path_csv;

/// Control period; matches examples/track_on_real_map.cpp so the runs stay comparable [s].
constexpr double CONTROL_DT = 0.01;

/// 1000 s of simulated time.
constexpr std::size_t MAX_STEPS = 100000;

/// Cycles between two dumps of the predicted poses; every cycle would be 11 rows per 10 ms.
constexpr std::size_t DUMP_EVERY = 25;

/// Half-width of the obstacle stamped onto the planned path [cells].
constexpr int OBSTACLE_HALF_WIDTH_CELLS = 4;

/// Fraction along the path where that obstacle appears; 0 leaves the map untouched.
constexpr double DEFAULT_OBSTACLE_FRACTION = 0.5;

/// Stamps lethal cells onto the path after planning, so the planner never saw the obstacle.
std::optional<Eigen::Vector2d> add_obstacle_on_path(
  eltanin_examples::InflatedMap & inflated, const Path & path, double fraction)
{
  if (fraction <= 0.0 || path.empty()) {
    return std::nullopt;
  }
  const std::size_t index = std::min(
    path.size() - 1, static_cast<std::size_t>(fraction * static_cast<double>(path.size())));
  const Eigen::Vector2d centre = path[index].position;
  const auto cell = inflated.map.geometry().world_to_map(centre);
  if (!cell.has_value()) {
    return std::nullopt;
  }
  for (int dy = -OBSTACLE_HALF_WIDTH_CELLS; dy <= OBSTACLE_HALF_WIDTH_CELLS; ++dy) {
    for (int dx = -OBSTACLE_HALF_WIDTH_CELLS; dx <= OBSTACLE_HALF_WIDTH_CELLS; ++dx) {
      inflated.map.set(cell->x + dx, cell->y + dy, eltanin::map::LETHAL_OBSTACLE);
    }
  }
  // Inflation takes the maximum per cell, so a second pass equals inflating the union map.
  eltanin::map::InflationLayer(inflated.inflation, false).update_costs(inflated.map);
  return centre;
}

struct Sample
{
  Pose2D pose{};
  Twist2D requested{};
  Twist2D limited{};
  double collision_distance{std::numeric_limits<double>::infinity()};
  bool has_collision{false};
  std::size_t predicted_count{0};
};

/// One tracking cycle: PurePursuit proposes, the limiter caps, SimpleSimulator applies.
std::vector<Sample> run(
  PurePursuit & tracker, const VelocityLimiter & limiter, const eltanin_examples::InflatedMap & map,
  const Path & path, const Pose2D & start, std::vector<VelocityLimiter::Result> & dumped,
  std::vector<std::size_t> & dumped_cycles)
{
  std::vector<Sample> samples;
  SimpleSimulator plant(start);
  for (std::size_t step = 0; step < MAX_STEPS; ++step) {
    const PurePursuit::Result tracking = tracker.compute(plant.pose(), path, CONTROL_DT);
    if (tracking.status != PurePursuit::Status::Tracking) {
      break;
    }
    const VelocityLimiter::Result limited =
      limiter.limit(map.map, map.model, plant.pose(), tracking.command);

    samples.push_back(Sample{
      plant.pose(), tracking.command, limited.command, limited.collision_distance,
      limited.has_collision, limited.predicted_poses.size()});
    if (step % DUMP_EVERY == 0) {
      dumped.push_back(limited);
      dumped_cycles.push_back(step);
    }

    if (limited.command.linear.x() == 0.0 && limited.command.angular == 0.0) {
      break;
    }
    plant.update(limited.command, CONTROL_DT);
  }
  return samples;
}

bool write_trajectory_csv(const std::filesystem::path & file, const std::vector<Sample> & samples)
{
  std::ofstream out(file);
  if (!out) {
    return false;
  }
  out << "t,x,y,yaw,v_in,w_in,v_out,w_out,collision_distance,has_collision,predicted_poses\n";
  double time = 0.0;
  for (const Sample & sample : samples) {
    out << time << ',' << sample.pose.position.x() << ',' << sample.pose.position.y() << ','
        << sample.pose.yaw << ',' << sample.requested.linear.x() << ','
        << sample.requested.angular << ',' << sample.limited.linear.x() << ','
        << sample.limited.angular << ',' << sample.collision_distance << ','
        << static_cast<int>(sample.has_collision) << ',' << sample.predicted_count << '\n';
    time += CONTROL_DT;
  }
  return static_cast<bool>(out);
}

/// The pose sequence the limiter predicted, which is what a visualizer draws ahead of the robot.
bool write_predicted_csv(
  const std::filesystem::path & file, const std::vector<VelocityLimiter::Result> & dumped,
  const std::vector<std::size_t> & cycles)
{
  std::ofstream out(file);
  if (!out) {
    return false;
  }
  out << "cycle,step,x,y,yaw,has_collision\n";
  for (std::size_t i = 0; i < dumped.size(); ++i) {
    for (std::size_t step = 0; step < dumped[i].predicted_poses.size(); ++step) {
      const Pose2D & pose = dumped[i].predicted_poses[step];
      out << cycles[i] << ',' << step << ',' << pose.position.x() << ',' << pose.position.y()
          << ',' << pose.yaw << ',' << static_cast<int>(dumped[i].has_collision) << '\n';
    }
  }
  return static_cast<bool>(out);
}

/// The footprint moved into the world frame, which the core deliberately never draws itself.
bool write_footprint_csv(
  const std::filesystem::path & file, const Polygon2D & footprint,
  const std::vector<VelocityLimiter::Result> & dumped, const std::vector<std::size_t> & cycles)
{
  std::ofstream out(file);
  if (!out) {
    return false;
  }
  out << "cycle,vertex,x,y\n";
  for (std::size_t i = 0; i < dumped.size(); ++i) {
    if (dumped[i].predicted_poses.empty()) {
      continue;
    }
    const Polygon2D world = eltanin::transform(footprint, dumped[i].predicted_poses.front());
    for (std::size_t v = 0; v < world.size(); ++v) {
      out << cycles[i] << ',' << v << ',' << world[v].x() << ',' << world[v].y() << '\n';
    }
  }
  return static_cast<bool>(out);
}

int usage(const char * program)
{
  std::cerr << "usage: " << program << " <map.yaml> <output_dir>"
            << " [start_x start_y goal_x goal_y] [obstacle_fraction]\n"
            << "  Plans a path, tracks it with PurePursuit, passes every command through\n"
            << "  VelocityLimiter and drives SimpleSimulator with the limited command.\n"
            << "  obstacle_fraction places a lethal blob that far along the planned path after\n"
            << "  planning, so the limiter meets an obstacle the planner never saw; 0 disables it.\n"
            << "  Writes crop.pgm, path.csv, trajectory.csv, predicted.csv, footprint.csv\n"
            << "  and meta.txt into <output_dir>.\n";
  return EXIT_FAILURE;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3 && argc != 4 && argc != 7 && argc != 8) {
    return usage(argv[0]);
  }
  const std::filesystem::path yaml = argv[1];
  const std::filesystem::path output_dir = argv[2];

  double obstacle_fraction = DEFAULT_OBSTACLE_FRACTION;
  try {
    if (argc == 4) {
      obstacle_fraction = std::stod(argv[3]);
    } else if (argc == 8) {
      obstacle_fraction = std::stod(argv[7]);
    }
  } catch (const std::exception &) {
    return usage(argv[0]);
  }

  auto inflated = eltanin_examples::load_and_inflate(yaml);
  if (!inflated.has_value()) {
    return EXIT_FAILURE;
  }
  const Costmap & costmap = inflated->map;

  Pose2D start;
  Pose2D goal;
  if (argc >= 7) {
    try {
      start.position = Eigen::Vector2d{std::stod(argv[3]), std::stod(argv[4])};
      goal.position = Eigen::Vector2d{std::stod(argv[5]), std::stod(argv[6])};
    } catch (const std::exception &) {
      return usage(argv[0]);
    }
  } else {
    const auto pair = eltanin_examples::auto_start_goal(costmap, inflated->model);
    if (!pair.has_value()) {
      return EXIT_FAILURE;
    }
    start = pair->first;
    goal = pair->second;
  }

  const auto planned = eltanin::planner::plan_astar(costmap, inflated->model, start, goal);
  if (!planned) {
    std::cerr << "plan_astar() found no path: " << eltanin::planner::to_string(planned.error())
              << '\n';
    return EXIT_FAILURE;
  }
  const Path path = *planned;
  if (path.size() < 2) {
    std::cerr << "the planned path is too short to track\n";
    return EXIT_FAILURE;
  }

  const std::optional<Eigen::Vector2d> obstacle =
    add_obstacle_on_path(*inflated, path, obstacle_fraction);

  auto tracker = PurePursuit::create(PurePursuitParams{});
  if (!tracker.has_value()) {
    std::cerr << "PurePursuit::create failed\n";
    return EXIT_FAILURE;
  }

  VelocityLimiterParams limiter_params;
  limiter_params.footprint = eltanin_examples::robot_footprint();
  const auto limiter = VelocityLimiter::create(limiter_params);
  if (!limiter.has_value()) {
    std::cerr << "VelocityLimiter::create failed\n";
    return EXIT_FAILURE;
  }

  std::vector<VelocityLimiter::Result> dumped;
  std::vector<std::size_t> dumped_cycles;
  const std::vector<Sample> samples =
    run(*tracker, *limiter, *inflated, path, path[0], dumped, dumped_cycles);

  std::error_code directory_error;
  std::filesystem::create_directories(output_dir, directory_error);

  std::vector<Eigen::Vector2d> drawn = positions_of(path);
  for (const Sample & sample : samples) {
    drawn.push_back(sample.pose.position);
  }
  MapIndex lower_left{0, 0};
  const Costmap crop = crop_around(costmap, drawn, lower_left);
  try {
    eltanin::map_io::write_pgm(output_dir / "crop.pgm", crop);
  } catch (const eltanin::map_io::MapIoError & error) {
    std::cerr << "failed to write crop.pgm: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  if (
    !write_path_csv(output_dir / "path.csv", path) ||
    !write_trajectory_csv(output_dir / "trajectory.csv", samples) ||
    !write_predicted_csv(output_dir / "predicted.csv", dumped, dumped_cycles) ||
    !write_footprint_csv(
      output_dir / "footprint.csv", limiter->footprint(), dumped, dumped_cycles)) {
    std::cerr << "failed to write the CSV files\n";
    return EXIT_FAILURE;
  }

  std::size_t limited_cycles = 0;
  std::size_t collision_cycles = 0;
  std::size_t truncated_cycles = 0;
  double min_collision_distance = std::numeric_limits<double>::infinity();
  double max_speed_loss = 0.0;
  const std::size_t full_prediction =
    static_cast<std::size_t>(limiter->params().prediction_steps) + 1;
  for (const Sample & sample : samples) {
    const double loss = std::abs(sample.requested.linear.x()) - std::abs(sample.limited.linear.x());
    if (loss > 1e-12) {
      ++limited_cycles;
      max_speed_loss = std::max(max_speed_loss, loss);
    }
    if (sample.has_collision) {
      ++collision_cycles;
      min_collision_distance = std::min(min_collision_distance, sample.collision_distance);
    } else if (sample.predicted_count < full_prediction) {
      ++truncated_cycles;
    }
  }

  std::ofstream meta(output_dir / "meta.txt");
  if (!meta) {
    std::cerr << "failed to write meta.txt\n";
    return EXIT_FAILURE;
  }
  write_meta(meta, crop, lower_left, *inflated);
  meta << "control_dt " << CONTROL_DT << '\n'
       << "dump_every " << DUMP_EVERY << '\n'
       << "prediction_steps " << limiter->params().prediction_steps << '\n'
       << "prediction_time " << limiter->params().prediction_time << '\n'
       << "prediction_dt " << limiter->prediction_dt() << '\n'
       << "collision_margin " << limiter->params().collision_margin << '\n'
       << "max_deceleration " << limiter->params().max_deceleration << '\n'
       << "path_poses " << path.size() << '\n'
       << "path_length " << eltanin::path_length(path) << '\n'
       << "cycles " << samples.size() << '\n'
       << "limited_cycles " << limited_cycles << '\n'
       << "collision_cycles " << collision_cycles << '\n'
       << "truncated_cycles " << truncated_cycles << '\n'
       << "max_speed_loss " << max_speed_loss << '\n'
       << "min_collision_distance " << min_collision_distance << '\n'
       << "obstacle_fraction " << obstacle_fraction << '\n';
  if (obstacle.has_value()) {
    meta << "obstacle_x " << obstacle->x() << '\n'
         << "obstacle_y " << obstacle->y() << '\n'
         << "obstacle_half_width " << OBSTACLE_HALF_WIDTH_CELLS * crop.geometry().resolution()
         << '\n'
         << "stop_distance_to_obstacle "
         << (samples.empty() ? 0.0 : (samples.back().pose.position - *obstacle).norm()) << '\n';
  }

  std::cout << "path " << path.size() << " poses, " << eltanin::path_length(path) << " m\n"
            << "cycles " << samples.size() << ", limited " << limited_cycles << ", collision "
            << collision_cycles << ", truncated " << truncated_cycles << '\n'
            << "max speed loss " << max_speed_loss << " m/s, min collision distance "
            << min_collision_distance << " m\n"
            << "wrote crop.pgm, path.csv, trajectory.csv, predicted.csv, footprint.csv, meta.txt"
            << " into " << output_dir << '\n';
  return EXIT_SUCCESS;
}
