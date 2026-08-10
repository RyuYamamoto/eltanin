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

#include <eltanin/collision/collision_checker.hpp>
#include <eltanin/collision/velocity_limiter.hpp>
#include <eltanin/core/footprint.hpp>
#include <eltanin/core/polygon.hpp>
#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/distance_map.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/map/layers/inflation_layer.hpp>
#include <eltanin/map_io/pgm.hpp>
#include <eltanin/sim/simple_simulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

namespace
{

using eltanin::DistanceTraversabilityModel;
using eltanin::Polygon2D;
using eltanin::Pose2D;
using eltanin::Traversability;
using eltanin::Twist2D;
using eltanin::collision::check_footprint;
using eltanin::collision::CollisionCheck;
using eltanin::collision::VelocityLimiter;
using eltanin::collision::VelocityLimiterParams;
using eltanin::map::Costmap;
using eltanin::map::CostTraversabilityModel;
using eltanin::map::InflationCostModel;
using eltanin::map::InflationLayer;
using eltanin::map::MapGeometry;
using eltanin::sim::SimpleSimulator;

constexpr double PI = std::numbers::pi;

/// 6 m x 6 m at 0.05 m; wide enough that the 1.0 m prediction horizon never leaves the map.
constexpr int MAP_CELLS = 120;
constexpr double RESOLUTION = 0.05;
constexpr double INFLATION_RADIUS = 0.55;
constexpr double COST_SCALING_FACTOR = 3.0;

/// Requested speed the limiter has to cut down [m/s].
constexpr double REQUESTED_SPEED = 0.5;

/// Control period of the closed-loop run [s].
constexpr double CONTROL_DT = 0.1;
constexpr int MAX_CYCLES = 200;

/// Headings sampled over the full turn for the polar plot.
constexpr int HEADING_SAMPLES = 360;

struct Scenario
{
  Costmap map;
  CostTraversabilityModel model;
};

Polygon2D default_footprint() { return VelocityLimiterParams{}.footprint; }

std::optional<InflationCostModel> inflation_model(const Polygon2D & footprint)
{
  const auto distance_model = DistanceTraversabilityModel::from_footprint(footprint, INFLATION_RADIUS);
  if (!distance_model.has_value()) {
    return std::nullopt;
  }
  return InflationCostModel::create(*distance_model, COST_SCALING_FACTOR);
}

Scenario inflate(Costmap map, const InflationCostModel & inflation)
{
  InflationLayer(inflation, false).update_costs(map);
  return Scenario{std::move(map), CostTraversabilityModel(inflation.circumscribed_cost(), false)};
}

/// A lethal wall column on the right edge; the robot drives towards or away from it along x.
Scenario wall_scenario(const InflationCostModel & inflation)
{
  Costmap map(
    MapGeometry(MAP_CELLS, MAP_CELLS, RESOLUTION, Eigen::Vector2d::Zero()),
    eltanin::map::FREE_SPACE);
  for (int my = 0; my < map.size_y(); ++my) {
    map(MAP_CELLS - 1, my) = eltanin::map::LETHAL_OBSTACLE;
  }
  return inflate(std::move(map), inflation);
}

/// The same wall with no inflation at all; the shape collision_predictor turns into a distance map.
Costmap raw_wall_map()
{
  Costmap map(
    MapGeometry(MAP_CELLS, MAP_CELLS, RESOLUTION, Eigen::Vector2d::Zero()),
    eltanin::map::FREE_SPACE);
  for (int my = 0; my < map.size_y(); ++my) {
    map(MAP_CELLS - 1, my) = eltanin::map::LETHAL_OBSTACLE;
  }
  return map;
}

/// A single lethal cell diagonally offset from the robot, which lands in the Circumscribed band.
Scenario single_obstacle_scenario(const InflationCostModel & inflation, int offset_x, int offset_y)
{
  Costmap map(
    MapGeometry(MAP_CELLS, MAP_CELLS, RESOLUTION, Eigen::Vector2d::Zero()),
    eltanin::map::FREE_SPACE);
  const int centre = MAP_CELLS / 2;
  map(centre + offset_x, centre + offset_y) = eltanin::map::LETHAL_OBSTACLE;
  return inflate(std::move(map), inflation);
}

Eigen::Vector2d cell_centre(int mx, int my)
{
  return Eigen::Vector2d{
    RESOLUTION * (static_cast<double>(mx) + 0.5), RESOLUTION * (static_cast<double>(my) + 0.5)};
}

/// The rule navyu used; kept here only so the CSV can show what it does to a reverse command.
double navyu_limit(double v_max, double v_in) { return std::min(v_max, v_in); }

/// Limited speed as a function of the gap to the wall, for a forward and a reverse command.
bool write_velocity_profile(
  const std::filesystem::path & file, const Scenario & scenario, const VelocityLimiter & limiter)
{
  std::ofstream out(file);
  if (!out) {
    return false;
  }
  const double wall_x = cell_centre(MAP_CELLS - 1, 0).x();
  out << "gap,collision_distance,v_out_forward,v_out_reverse,navyu_forward,navyu_reverse\n";

  const auto braking_limit = [&limiter](double collision_distance) {
    return std::sqrt(
      2.0 * limiter.params().max_deceleration *
      std::max(0.0, collision_distance - limiter.params().collision_margin));
  };

  for (int mx = 10; mx < MAP_CELLS - 1; ++mx) {
    const Eigen::Vector2d position = cell_centre(mx, MAP_CELLS / 2);
    // Facing away from the wall with a negative command traces the very same motion towards it.
    const VelocityLimiter::Result forward = limiter.limit(
      scenario.map, scenario.model, Pose2D{position, 0.0},
      Twist2D{Eigen::Vector2d{REQUESTED_SPEED, 0.0}, 0.0});
    const VelocityLimiter::Result reverse = limiter.limit(
      scenario.map, scenario.model, Pose2D{position, PI},
      Twist2D{Eigen::Vector2d{-REQUESTED_SPEED, 0.0}, 0.0});

    out << wall_x - position.x() << ',' << forward.collision_distance << ','
        << forward.command.linear.x() << ',' << reverse.command.linear.x() << ','
        << navyu_limit(braking_limit(forward.collision_distance), REQUESTED_SPEED) << ','
        << navyu_limit(braking_limit(reverse.collision_distance), -REQUESTED_SPEED) << '\n';
  }
  return static_cast<bool>(out);
}

/// Cross-section of the distance-map law: the clearance, the proximity ramp and the two caps.
bool write_distance_profile(
  const std::filesystem::path & file, const VelocityLimiter & limiter)
{
  const auto distances = eltanin::map::build_distance_map(
    raw_wall_map(),
    CostTraversabilityModel(eltanin::map::INSCRIBED_INFLATED_OBSTACLE, false),
    eltanin::map::DistanceMapParams{});
  const auto model =
    DistanceTraversabilityModel::from_footprint(limiter.footprint(), INFLATION_RADIUS);
  if (!distances.has_value() || !model.has_value()) {
    return false;
  }

  std::ofstream out(file);
  if (!out) {
    return false;
  }
  const double wall_x = cell_centre(MAP_CELLS - 1, 0).x();
  out << "gap,collision_distance,time_to_collision,clearance,proximity_scale,v_out\n";

  for (int mx = 10; mx < MAP_CELLS - 1; ++mx) {
    const Eigen::Vector2d position = cell_centre(mx, MAP_CELLS / 2);
    const VelocityLimiter::Result forward = limiter.limit(
      *distances, *model, Pose2D{position, 0.0},
      Twist2D{Eigen::Vector2d{REQUESTED_SPEED, 0.0}, 0.0});

    out << wall_x - position.x() << ',' << forward.collision_distance << ','
        << forward.time_to_collision << ','
        << forward.clearance.value_or(std::numeric_limits<double>::quiet_NaN()) << ','
        << forward.proximity_scale << ',' << forward.command.linear.x() << '\n';
  }
  return static_cast<bool>(out);
}

/// Closed-loop stopping run; the plant and the prediction share integrate_differential_drive().
bool write_closed_loop_csv(
  const std::filesystem::path & file, const Scenario & scenario, const VelocityLimiter & limiter,
  double requested_speed)
{
  std::ofstream out(file);
  if (!out) {
    return false;
  }
  const double wall_x = cell_centre(MAP_CELLS - 1, 0).x();
  out << "cycle,t,x,gap,v_out,collision_distance,has_collision,predicted_poses\n";

  SimpleSimulator plant(Pose2D{cell_centre(20, MAP_CELLS / 2), 0.0});
  for (int cycle = 0; cycle < MAX_CYCLES; ++cycle) {
    const VelocityLimiter::Result result = limiter.limit(
      scenario.map, scenario.model, plant.pose(),
      Twist2D{Eigen::Vector2d{requested_speed, 0.0}, 0.0});
    out << cycle << ',' << static_cast<double>(cycle) * CONTROL_DT << ','
        << plant.pose().position.x() << ',' << wall_x - plant.pose().position.x() << ','
        << result.command.linear.x() << ',' << result.collision_distance << ','
        << static_cast<int>(result.has_collision) << ',' << result.predicted_poses.size() << '\n';
    if (result.command.linear.x() == 0.0) {
      break;
    }
    plant.update(result.command, CONTROL_DT);
  }
  return static_cast<bool>(out);
}

/// Collision verdict over a full turn at one pose; only the Circumscribed band can vary.
bool write_heading_sweep(
  const std::filesystem::path & file, const Scenario & scenario, const Polygon2D & footprint,
  int offset_x, int offset_y)
{
  std::ofstream out(file);
  if (!out) {
    return false;
  }
  const int centre = MAP_CELLS / 2;
  const Eigen::Vector2d robot = cell_centre(centre, centre);
  const Eigen::Vector2d obstacle = cell_centre(centre + offset_x, centre + offset_y);
  const Traversability band = scenario.model.classify(scenario.map(centre, centre));

  out << "# obstacle_distance " << (obstacle - robot).norm() << '\n'
      << "# centre_cost " << static_cast<int>(scenario.map(centre, centre)) << '\n'
      << "# centre_band " << static_cast<int>(band) << '\n';
  out << "yaw,collision\n";
  for (int i = 0; i < HEADING_SAMPLES; ++i) {
    const double yaw = -PI + 2.0 * PI * static_cast<double>(i) / HEADING_SAMPLES;
    const CollisionCheck check =
      check_footprint(scenario.map, scenario.model, footprint, Pose2D{robot, yaw});
    out << yaw << ',' << static_cast<int>(check == CollisionCheck::Collision) << '\n';
  }
  return static_cast<bool>(out);
}

/// Per-cell band of the map, so a plot can show where the exact check is even reachable.
bool write_band_pgm(const std::filesystem::path & file, const Scenario & scenario)
{
  Costmap bands(scenario.map.geometry());
  for (int my = 0; my < scenario.map.size_y(); ++my) {
    for (int mx = 0; mx < scenario.map.size_x(); ++mx) {
      switch (scenario.model.classify(scenario.map(mx, my))) {
        case Traversability::Free:
          bands(mx, my) = 0;
          break;
        case Traversability::Circumscribed:
          bands(mx, my) = 128;
          break;
        case Traversability::Inscribed:
          bands(mx, my) = 254;
          break;
      }
    }
  }
  try {
    eltanin::map_io::write_pgm(file, bands);
  } catch (const eltanin::map_io::MapIoError & error) {
    std::cerr << "failed to write " << file << ": " << error.what() << '\n';
    return false;
  }
  return true;
}

int usage(const char * program)
{
  std::cerr << "usage: " << program << " <output_dir>\n"
            << "  Runs the velocity limiter on synthetic maps and writes CSV files for plotting.\n"
            << "  velocity_profile.csv  limited speed vs the gap to a wall, forward and reverse,\n"
            << "                        next to what navyu's std::min() would have produced.\n"
            << "  closed_loop_*.csv     the stopping run driven through SimpleSimulator.\n"
            << "  heading_sweep_*.csv   collision verdict over a full turn at one pose.\n"
            << "  bands.pgm             per-cell Free / Circumscribed / Inscribed classification.\n"
            << "  meta.txt              parameters and the derived distance_model.\n";
  return EXIT_FAILURE;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    return usage(argv[0]);
  }
  const std::filesystem::path output_dir = argv[1];

  const Polygon2D footprint = default_footprint();
  const auto inflation = inflation_model(footprint);
  if (!inflation.has_value()) {
    std::cerr << "failed to build the inflation model\n";
    return EXIT_FAILURE;
  }
  const auto limiter = VelocityLimiter::create(VelocityLimiterParams{});
  if (!limiter.has_value()) {
    std::cerr << "VelocityLimiter::create failed\n";
    return EXIT_FAILURE;
  }

  const Scenario wall = wall_scenario(*inflation);
  // (5, 5) cells away is 0.354 m, which falls between the inscribed and circumscribed distance_model.
  const Scenario diagonal = single_obstacle_scenario(*inflation, 5, 5);
  const Scenario head_on = single_obstacle_scenario(*inflation, 5, 0);

  std::error_code directory_error;
  std::filesystem::create_directories(output_dir, directory_error);

  if (
    !write_velocity_profile(output_dir / "velocity_profile.csv", wall, *limiter) ||
    !write_distance_profile(output_dir / "distance_profile.csv", *limiter) ||
    !write_closed_loop_csv(output_dir / "closed_loop_forward.csv", wall, *limiter, REQUESTED_SPEED) ||
    !write_heading_sweep(output_dir / "heading_sweep_diagonal.csv", diagonal, footprint, 5, 5) ||
    !write_heading_sweep(output_dir / "heading_sweep_head_on.csv", head_on, footprint, 5, 0) ||
    !write_band_pgm(output_dir / "bands.pgm", diagonal)) {
    std::cerr << "failed to write the output files\n";
    return EXIT_FAILURE;
  }

  const DistanceTraversabilityModel & distance_model = inflation->distance_model();
  std::ofstream meta(output_dir / "meta.txt");
  if (!meta) {
    std::cerr << "failed to write meta.txt\n";
    return EXIT_FAILURE;
  }
  meta << "resolution " << RESOLUTION << '\n'
       << "size_x " << MAP_CELLS << '\n'
       << "size_y " << MAP_CELLS << '\n'
       << "inscribed_radius " << distance_model.inscribed_radius() << '\n'
       << "circumscribed_radius " << distance_model.circumscribed_radius() << '\n'
       << "inflation_radius " << distance_model.inflation_radius() << '\n'
       << "circumscribed_cost " << static_cast<int>(inflation->circumscribed_cost()) << '\n'
       << "cost_scaling_factor " << COST_SCALING_FACTOR << '\n'
       << "requested_speed " << REQUESTED_SPEED << '\n'
       << "control_dt " << CONTROL_DT << '\n'
       << "prediction_dt "
       << limiter->prediction_dt(Twist2D{Eigen::Vector2d{REQUESTED_SPEED, 0.0}, 0.0}) << '\n'
       << "prediction_steps " << limiter->params().prediction_steps << '\n'
       << "reaction_time " << limiter->params().reaction_time << '\n'
       << "horizon "
       << limiter->horizon(Twist2D{Eigen::Vector2d{REQUESTED_SPEED, 0.0}, 0.0}) << '\n'
       << "collision_margin " << limiter->params().collision_margin << '\n'
       << "max_deceleration " << limiter->params().max_deceleration << '\n';

  std::cout << "inscribed " << distance_model.inscribed_radius() << " m, circumscribed "
            << distance_model.circumscribed_radius() << " m, circumscribed_cost "
            << static_cast<int>(inflation->circumscribed_cost()) << '\n'
            << "wrote velocity_profile.csv, distance_profile.csv, closed_loop_forward.csv,"
            << " heading_sweep_diagonal.csv,"
            << " heading_sweep_head_on.csv, bands.pgm, meta.txt into " << output_dir << '\n';
  return EXIT_SUCCESS;
}
