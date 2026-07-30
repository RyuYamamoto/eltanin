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

#include <eltanin/control/pure_pursuit.hpp>
#include <eltanin/core/angle.hpp>
#include <eltanin/core/footprint.hpp>
#include <eltanin/core/geometry.hpp>
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
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
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
using eltanin::Twist2D;
using eltanin::control::PurePursuit;
using eltanin::control::PurePursuitParams;
using eltanin::map::Costmap;
using eltanin::map::CostTraversabilityModel;
using eltanin::map::InflationCostModel;
using eltanin::map::InflationLayer;
using eltanin::map::MapGeometry;
using eltanin::map::MapIndex;

/// Same robot as examples/plan_on_real_map.cpp, so the artifacts stay comparable.
constexpr double INFLATION_RADIUS = 0.55;
constexpr double COST_SCALING_FACTOR = 10.0;

/// Cells kept around the path when cropping, so the surrounding obstacles stay visible.
constexpr int CROP_MARGIN_CELLS = 30;

/// Bound on the flood fill used to pick an automatic goal.
constexpr std::size_t MAX_FLOOD_FILL_CELLS = 200000;

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
};

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

/// Explicit Euler integration of a differential-drive robot driven by PurePursuit.
std::vector<Sample> track(PurePursuit & tracker, const Path & path, const Pose2D & start)
{
  std::vector<Sample> samples;
  samples.reserve(MAX_STEPS / 10);
  Pose2D pose = start;
  double travelled = 0.0;
  for (std::size_t step = 0; step < MAX_STEPS; ++step) {
    const PurePursuit::Result result = tracker.compute(pose, path, CONTROL_DT);
    if (result.status != PurePursuit::Status::Tracking) {
      break;
    }
    samples.push_back(Sample{
      pose, result.command, lateral_error(path, pose.position), travelled, result.target_index,
      result.lookahead_point});

    const double v = result.command.linear.x();
    pose.position.x() += v * std::cos(pose.yaw) * CONTROL_DT;
    pose.position.y() += v * std::sin(pose.yaw) * CONTROL_DT;
    pose.yaw = eltanin::normalize_angle(pose.yaw + result.command.angular * CONTROL_DT);
    travelled += v * CONTROL_DT;
  }
  samples.push_back(Sample{pose, Twist2D{}, lateral_error(path, pose.position), travelled});
  return samples;
}

/// Cells of `map` covering both the path and the traced trajectory.
Costmap crop_around(
  const Costmap & map, const Path & path, const std::vector<Sample> & samples,
  MapIndex & lower_left)
{
  const MapGeometry & geometry = map.geometry();
  int min_x = map.size_x();
  int min_y = map.size_y();
  int max_x = 0;
  int max_y = 0;
  const auto extend = [&](const Eigen::Vector2d & position) {
    const auto index = geometry.world_to_map(position);
    if (!index.has_value()) {
      return;
    }
    min_x = std::min(min_x, index->x);
    max_x = std::max(max_x, index->x);
    min_y = std::min(min_y, index->y);
    max_y = std::max(max_y, index->y);
  };
  for (const Pose2D & pose : path) {
    extend(pose.position);
  }
  for (const Sample & sample : samples) {
    extend(sample.pose.position);
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

bool write_trajectory_csv(const std::filesystem::path & file, const std::vector<Sample> & samples)
{
  std::ofstream out(file);
  if (!out) {
    return false;
  }
  out << "t,x,y,yaw,v,w,lateral_error,travelled,target_index,lookahead_x,lookahead_y\n";
  double time = 0.0;
  for (const Sample & sample : samples) {
    out << time << ',' << sample.pose.position.x() << ',' << sample.pose.position.y() << ','
        << sample.pose.yaw << ',' << sample.command.linear.x() << ',' << sample.command.angular
        << ',' << sample.lateral_error << ',' << sample.travelled << ',' << sample.target_index
        << ',' << sample.lookahead_point.x() << ',' << sample.lookahead_point.y() << '\n';
    time += CONTROL_DT;
  }
  return static_cast<bool>(out);
}

/// Poses whose cell is not Free; the planner must produce none, the tracker may.
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
  std::cerr << "usage: " << program << " <map.yaml> <output_dir>"
            << " [start_x start_y goal_x goal_y] [lateral_offset]\n"
            << "  Without explicit poses, a reachable start/goal pair is picked automatically.\n"
            << "  lateral_offset [m] displaces the robot sideways from the path start, which\n"
            << "  exercises the transient instead of only the steady-state error.\n"
            << "  Writes crop.pgm, path.csv, trajectory.csv and meta.txt into <output_dir>.\n";
  return EXIT_FAILURE;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3 || argc == 5 || argc == 6 || argc > 8) {
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
  double offset = 0.0;
  try {
    if (argc >= 7) {
      start.position = Eigen::Vector2d{std::stod(argv[3]), std::stod(argv[4])};
      goal.position = Eigen::Vector2d{std::stod(argv[5]), std::stod(argv[6])};
    }
    if (argc == 4) {
      offset = std::stod(argv[3]);
    } else if (argc == 8) {
      offset = std::stod(argv[7]);
    }
  } catch (const std::exception &) {
    return usage(argv[0]);
  }
  if (argc < 7) {
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
  const Path path = eltanin::planner::smooth(*raw, costmap, model);
  if (path.size() < 2) {
    std::cerr << "the smoothed path is too short to track\n";
    return EXIT_FAILURE;
  }

  auto tracker = PurePursuit::create(PurePursuitParams{});
  if (!tracker.has_value()) {
    std::cerr << "failed to build the tracker\n";
    return EXIT_FAILURE;
  }
  const PurePursuitParams & params = tracker->params();

  // The robot starts on the path tangent; `offset` moves it sideways to excite the transient.
  Pose2D robot{path[0].position, path[0].yaw};
  robot.position += offset * Eigen::Vector2d{-std::sin(robot.yaw), std::cos(robot.yaw)};
  const std::vector<Sample> samples = track(*tracker, path, robot);

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

  std::error_code directory_error;
  std::filesystem::create_directories(output_dir, directory_error);
  MapIndex lower_left{0, 0};
  const Costmap crop = crop_around(costmap, path, samples, lower_left);
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
       << "inflation_radius " << radii->inflation_radius() << '\n'
       << "control_dt " << CONTROL_DT << '\n'
       << "desired_linear_vel " << params.desired_linear_vel << '\n'
       << "max_angular_vel " << params.max_angular_vel << '\n'
       << "yaw_tolerance " << params.yaw_tolerance << '\n'
       << "lookahead_time " << params.lookahead_time << '\n'
       << "min_lookahead_dist " << params.min_lookahead_dist << '\n'
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
       << "trajectory_outside_map " << intrusions.outside << '\n';

  std::cout << "crop " << crop_geometry.size_x() << " x " << crop_geometry.size_y() << " cells @ "
            << crop_geometry.resolution() << " m\n"
            << "path " << path.size() << " poses, " << eltanin::path_length(path)
            << " m, non-free poses " << count_non_free(path, costmap, model) << '\n'
            << "tracked " << samples.size() << " steps, " << last.travelled
            << " m, goal reached " << static_cast<int>(reached) << ", remaining " << remaining
            << " m\n"
            << "lateral error: max " << max_error << " m, after " << TRANSIENT_GATE << " m "
            << max_error_after_gate << " m, final " << last.lateral_error << " m\n"
            << "command peaks: v " << max_linear_vel << " m/s (limit "
            << params.desired_linear_vel << "), |w| " << max_abs_angular_vel << " rad/s (limit "
            << params.max_angular_vel << ")\n"
            << "trajectory left the Free band: circumscribed " << intrusions.circumscribed
            << ", inscribed " << intrusions.inscribed << ", outside map " << intrusions.outside
            << '\n'
            << "wrote crop.pgm, path.csv, trajectory.csv, meta.txt into " << output_dir << '\n';
  return EXIT_SUCCESS;
}
