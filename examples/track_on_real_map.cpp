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

#include <eltanin/control/follower_factory.hpp>
#include <eltanin/control/pure_pursuit.hpp>
#include <eltanin/core/angle.hpp>
#include <eltanin/core/geometry.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/map_io/pgm.hpp>
#include <eltanin/planner/astar_planner.hpp>
#include <eltanin/planner/path_smoother.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace
{

using eltanin::Path;
using eltanin::Pose2D;
using eltanin::Traversability;
using eltanin::Twist2D;
using eltanin::control::FollowerFactoryParams;
using eltanin::control::FollowerState;
using eltanin::control::FollowerType;
using eltanin::control::FollowResult;
using eltanin::control::FollowStatus;
using eltanin::control::PathFollower;
using eltanin::control::PurePursuit;
using eltanin::control::PurePursuitParams;
using eltanin::control::VelocityProfileParams;
using eltanin::map::Costmap;
using eltanin::map::CostTraversabilityModel;
using eltanin::map::MapGeometry;
using eltanin::map::MapIndex;
using eltanin_examples::crop_around;
using eltanin_examples::positions_of;
using eltanin_examples::write_meta;
using eltanin_examples::count_non_free;
using eltanin_examples::write_path_csv;

/// Control period; matches navyu's update_frequency of 100 Hz [s].
constexpr double CONTROL_DT = 0.01;

/// Travel distance after which the transient of the initial alignment is treated as over [m].
constexpr double TRANSIENT_GATE = 1.0;

/// 1000 s of simulated time; a 37 m path at 0.5 m/s needs about 7500 steps.
constexpr std::size_t MAX_STEPS = 100000;

struct Sample
{
  Pose2D pose{};
  Twist2D command{};
  double lateral_error{0.0};
  double travelled{0.0};
  std::size_t target_index{0};
  Eigen::Vector2d lookahead_point{Eigen::Vector2d::Zero()};
  double solve_time{0.0};
  int solver_iterations{0};
};

/// Minimum distance from `position` to the path polyline [m].
double lateral_error(const Path & path, const Eigen::Vector2d & position)
{
  if (path.size() < 2) {
    return path.empty() ? 0.0 : (position - path[0].position).norm();
  }
  double minimum = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < path.size(); ++i) {
    minimum = std::min(
      minimum, eltanin::distance_to_segment(position, path[i - 1].position, path[i].position));
  }
  return minimum;
}

/// Explicit Euler integration of a differential-drive robot driven by the selected follower.
std::vector<Sample> track(PathFollower & follower, const Path & path, const Pose2D & start)
{
  const auto * pursuit = dynamic_cast<const PurePursuit *>(&follower);
#ifdef ELTANIN_WITH_MPC
  const auto * mpc = dynamic_cast<const eltanin::control::MpcFollower *>(&follower);
#endif
  std::vector<Sample> samples;
  samples.reserve(MAX_STEPS / 10);
  Pose2D pose = start;
  double travelled = 0.0;
  std::optional<Twist2D> measured;
  for (std::size_t step = 0; step < MAX_STEPS; ++step) {
    const FollowResult result = follower.follow(FollowerState{pose, measured}, path, CONTROL_DT);
    if (result.status != FollowStatus::Tracking) {
      break;
    }
    measured = result.command;
    Sample sample{pose, result.command, lateral_error(path, pose.position), travelled};
    if (pursuit != nullptr) {
      sample.target_index = pursuit->lookahead().target_index;
      sample.lookahead_point = pursuit->lookahead().point;
    }
#ifdef ELTANIN_WITH_MPC
    if (mpc != nullptr) {
      sample.solve_time = mpc->solver_stats().solve_time;
      sample.solver_iterations = mpc->solver_stats().iterations;
    }
#endif
    samples.push_back(sample);

    const double v = result.command.linear.x();
    pose.position.x() += v * std::cos(pose.yaw) * CONTROL_DT;
    pose.position.y() += v * std::sin(pose.yaw) * CONTROL_DT;
    pose.yaw = eltanin::normalize_angle(pose.yaw + result.command.angular * CONTROL_DT);
    travelled += v * CONTROL_DT;
  }
  samples.push_back(Sample{pose, Twist2D{}, lateral_error(path, pose.position), travelled});
  return samples;
}

bool write_trajectory_csv(const std::filesystem::path & file, const std::vector<Sample> & samples)
{
  std::ofstream out(file);
  if (!out) {
    return false;
  }
  out << "t,x,y,yaw,v,w,lateral_error,travelled,target_index,lookahead_x,lookahead_y"
         ",solve_time,solver_iterations\n";
  double time = 0.0;
  for (const Sample & sample : samples) {
    out << time << ',' << sample.pose.position.x() << ',' << sample.pose.position.y() << ','
        << sample.pose.yaw << ',' << sample.command.linear.x() << ',' << sample.command.angular
        << ',' << sample.lateral_error << ',' << sample.travelled << ',' << sample.target_index
        << ',' << sample.lookahead_point.x() << ',' << sample.lookahead_point.y() << ','
        << sample.solve_time << ',' << sample.solver_iterations << '\n';
    time += CONTROL_DT;
  }
  return static_cast<bool>(out);
}

struct Intrusions
{
  std::size_t circumscribed{0};
  std::size_t inscribed{0};
  std::size_t outside{0};
};

/// How often the traced trajectory left the Free band; this is the margin question of §6.1.
Intrusions count_intrusions(
  const std::vector<Sample> & samples, const Costmap & map, const CostTraversabilityModel & model)
{
  Intrusions counts;
  for (const Sample & sample : samples) {
    const auto index = map.geometry().world_to_map(sample.pose.position);
    if (!index.has_value()) {
      ++counts.outside;
      continue;
    }
    switch (model.classify(map(index->x, index->y))) {
      case Traversability::Free:
        break;
      case Traversability::Circumscribed:
        ++counts.circumscribed;
        break;
      case Traversability::Inscribed:
        ++counts.inscribed;
        break;
    }
  }
  return counts;
}

int usage(const char * program)
{
  std::cerr << "usage: " << program << " [--follower <name>] [--velocity-profile] [--vmax <v>]"
            << " <map.yaml> <output_dir>"
            << " [start_x start_y goal_x goal_y] [lateral_offset]\n"
            << "  --follower is pure_pursuit (default) or mpc; --velocity-profile caps the speed\n"
            << "  by the path curvature. Without explicit poses a reachable pair is picked.\n"
            << "  lateral_offset [m] displaces the robot sideways from the path start, which\n"
            << "  exercises the transient instead of only the steady-state error.\n"
            << "  Writes crop.pgm, path.csv, trajectory.csv and meta.txt into <output_dir>.\n";
  return EXIT_FAILURE;
}

/// Pulls the named options out of the command line, leaving the positional arguments behind.
std::optional<FollowerFactoryParams> take_options(std::vector<std::string> & arguments)
{
  FollowerFactoryParams params;
  std::optional<VelocityProfileParams> profile;
  double max_linear_vel = params.pure_pursuit.desired_linear_vel;
  std::vector<std::string> rest;
  for (std::size_t i = 0; i < arguments.size(); ++i) {
    if (arguments[i] == "--follower" && i + 1 < arguments.size()) {
      const std::optional<FollowerType> type =
        eltanin::control::to_follower_type(arguments[++i]);
      if (!type.has_value()) {
        return std::nullopt;
      }
      params.type = *type;
    } else if (arguments[i] == "--velocity-profile") {
      profile = VelocityProfileParams{};
    } else if (arguments[i] == "--vmax" && i + 1 < arguments.size()) {
      max_linear_vel = std::stod(arguments[++i]);
    } else {
      rest.push_back(arguments[i]);
    }
  }
  arguments = rest;

  params.pure_pursuit.desired_linear_vel = max_linear_vel;
#ifdef ELTANIN_WITH_MPC
  params.mpc.max_linear_vel = max_linear_vel;
#endif
  if (profile.has_value()) {
    profile->max_linear_vel = max_linear_vel;
    params.pure_pursuit.velocity_profile = profile;
#ifdef ELTANIN_WITH_MPC
    params.mpc.velocity_profile = profile;
#endif
  }
  return params;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::vector<std::string> arguments(argv + 1, argv + argc);
  std::optional<FollowerFactoryParams> options;
  try {
    options = take_options(arguments);
  } catch (const std::exception &) {
    return usage(argv[0]);
  }
  if (!options.has_value()) {
    return usage(argv[0]);
  }
  const std::size_t count = arguments.size();
  if (count != 2 && count != 3 && count != 6 && count != 7) {
    return usage(argv[0]);
  }
  const std::filesystem::path yaml = arguments[0];
  const std::filesystem::path output_dir = arguments[1];

  const auto inflated = eltanin_examples::load_and_inflate(yaml);
  if (!inflated.has_value()) {
    return EXIT_FAILURE;
  }
  const Costmap & costmap = inflated->map;
  const CostTraversabilityModel & model = inflated->model;

  Pose2D start;
  Pose2D goal;
  double offset = 0.0;
  try {
    if (count >= 6) {
      start.position = Eigen::Vector2d{std::stod(arguments[2]), std::stod(arguments[3])};
      goal.position = Eigen::Vector2d{std::stod(arguments[4]), std::stod(arguments[5])};
    }
    if (count == 3) {
      offset = std::stod(arguments[2]);
    } else if (count == 7) {
      offset = std::stod(arguments[6]);
    }
  } catch (const std::exception &) {
    return usage(argv[0]);
  }
  if (count < 6) {
    const auto pair = eltanin_examples::auto_start_goal(costmap, model);
    if (!pair.has_value()) {
      return EXIT_FAILURE;
    }
    start = pair->first;
    goal = pair->second;
  }

  const auto planned = eltanin::planner::plan_astar(costmap, model, start, goal);
  if (!planned) {
    std::cerr << "plan_astar() found no path: " << eltanin::planner::to_string(planned.error())
              << '\n';
    return EXIT_FAILURE;
  }
  const Path & path = *planned;
  if (path.size() < 2) {
    std::cerr << "the planned path is too short to track\n";
    return EXIT_FAILURE;
  }

  eltanin::control::FollowerResult built = eltanin::control::make_path_follower(*options);
  if (!built.has_value()) {
    std::cerr << "failed to build the follower: " << eltanin::control::to_string(built.error())
              << '\n';
    return EXIT_FAILURE;
  }
  const std::unique_ptr<PathFollower> follower = built.take();
  const bool profiled = options->pure_pursuit.velocity_profile.has_value();
  double max_linear_vel_limit = options->pure_pursuit.desired_linear_vel;
  double max_angular_vel_limit = options->pure_pursuit.max_angular_vel;
#ifdef ELTANIN_WITH_MPC
  if (options->type == FollowerType::Mpc) {
    max_linear_vel_limit = options->mpc.max_linear_vel;
    max_angular_vel_limit = options->mpc.max_angular_vel;
  }
#endif

  // The robot starts on the path tangent; `offset` moves it sideways to excite the transient.
  Pose2D robot{path[0].position, path[0].yaw};
  robot.position += offset * Eigen::Vector2d{-std::sin(robot.yaw), std::cos(robot.yaw)};
  const std::vector<Sample> samples = track(*follower, path, robot);

  double max_error = 0.0;
  double max_error_after_gate = 0.0;
  double max_linear_vel = 0.0;
  double max_abs_angular_vel = 0.0;
  for (const Sample & sample : samples) {
    max_error = std::max(max_error, sample.lateral_error);
    if (sample.travelled >= TRANSIENT_GATE) {
      max_error_after_gate = std::max(max_error_after_gate, sample.lateral_error);
    }
    max_linear_vel = std::max(max_linear_vel, sample.command.linear.x());
    max_abs_angular_vel = std::max(max_abs_angular_vel, std::abs(sample.command.angular));
  }
  const Sample & last = samples.back();
  const double remaining = (path[path.size() - 1].position - last.pose.position).norm();
  const bool reached = samples.size() < MAX_STEPS;
  const Intrusions intrusions = count_intrusions(samples, costmap, model);

  std::vector<double> solve_times;
  int solver_iterations_max = 0;
  for (const Sample & sample : samples) {
    solve_times.push_back(sample.solve_time);
    solver_iterations_max = std::max(solver_iterations_max, sample.solver_iterations);
  }
  std::sort(solve_times.begin(), solve_times.end());
  const double solve_time_p50 = solve_times[solve_times.size() / 2];
  const double solve_time_p99 = solve_times[solve_times.size() * 99 / 100];
  const double solve_time_max = solve_times.back();

  std::error_code directory_error;
  std::filesystem::create_directories(output_dir, directory_error);
  MapIndex lower_left{0, 0};
  std::vector<Eigen::Vector2d> drawn = positions_of(path);
  for (const Sample & sample : samples) {
    drawn.push_back(sample.pose.position);
  }
  const Costmap crop = crop_around(costmap, drawn, lower_left);
  try {
    eltanin::map_io::write_pgm(output_dir / "crop.pgm", crop);
  } catch (const eltanin::map_io::MapIoError & error) {
    std::cerr << "failed to write crop.pgm: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  if (!write_path_csv(output_dir / "path.csv", path) ||
      !write_trajectory_csv(output_dir / "trajectory.csv", samples)) {
    std::cerr << "failed to write the CSV files\n";
    return EXIT_FAILURE;
  }

  const MapGeometry & crop_geometry = crop.geometry();
  std::ofstream meta(output_dir / "meta.txt");
  if (!meta) {
    std::cerr << "failed to write meta.txt\n";
    return EXIT_FAILURE;
  }
  write_meta(meta, crop, lower_left, *inflated);
  meta << "control_dt " << CONTROL_DT << '\n'
       << "follower " << eltanin::control::name_of(options->type) << '\n'
       << "velocity_profile " << static_cast<int>(profiled) << '\n'
       << "desired_linear_vel " << max_linear_vel_limit << '\n'
       << "max_angular_vel " << max_angular_vel_limit << '\n'
       << "initial_lateral_offset " << offset << '\n'
       << "path_poses " << path.size() << '\n'
       << "path_length " << eltanin::path_length(path) << '\n'
       << "path_non_free_poses " << count_non_free(path, costmap, model) << '\n'
       << "goal_reached " << static_cast<int>(reached) << '\n'
       << "steps " << samples.size() << '\n'
       << "travelled " << last.travelled << '\n'
       << "goal_remaining " << remaining << '\n'
       << "max_lateral_error " << max_error << '\n'
       << "max_lateral_error_after_gate " << max_error_after_gate << '\n'
       << "final_lateral_error " << last.lateral_error << '\n'
       << "max_linear_vel " << max_linear_vel << '\n'
       << "max_abs_angular_vel " << max_abs_angular_vel << '\n'
       << "trajectory_circumscribed " << intrusions.circumscribed << '\n'
       << "trajectory_inscribed " << intrusions.inscribed << '\n'
       << "trajectory_outside_map " << intrusions.outside << '\n'
       << "solve_time_p50 " << solve_time_p50 << '\n'
       << "solve_time_p99 " << solve_time_p99 << '\n'
       << "solve_time_max " << solve_time_max << '\n'
       << "solver_iterations_max " << solver_iterations_max << '\n';

  std::cout << "crop " << crop_geometry.size_x() << " x " << crop_geometry.size_y() << " cells @ "
            << crop_geometry.resolution() << " m\n"
            << "path " << path.size() << " poses, " << eltanin::path_length(path)
            << " m, non-free poses " << count_non_free(path, costmap, model) << '\n'
            << "tracked " << samples.size() << " steps, " << last.travelled
            << " m, goal reached " << static_cast<int>(reached) << ", remaining " << remaining
            << " m\n"
            << "lateral error: max " << max_error << " m, after " << TRANSIENT_GATE << " m "
            << max_error_after_gate << " m, final " << last.lateral_error << " m\n"
            << "command peaks: v " << max_linear_vel << " m/s (limit " << max_linear_vel_limit
            << "), |w| " << max_abs_angular_vel << " rad/s (limit " << max_angular_vel_limit
            << ")\n"
            << "solve time: p50 " << solve_time_p50 * 1e3 << " ms, p99 " << solve_time_p99 * 1e3
            << " ms, max " << solve_time_max * 1e3 << " ms, iterations max "
            << solver_iterations_max << '\n'
            << "trajectory left the Free band: circumscribed " << intrusions.circumscribed
            << ", inscribed " << intrusions.inscribed << ", outside map " << intrusions.outside
            << '\n'
            << "wrote crop.pgm, path.csv, trajectory.csv, meta.txt into " << output_dir << '\n';
  return EXIT_SUCCESS;
}
