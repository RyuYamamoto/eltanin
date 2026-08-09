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

#ifndef ELTANIN_EXAMPLES__NAVIGATION_LOOP_HPP_
#define ELTANIN_EXAMPLES__NAVIGATION_LOOP_HPP_

#include <real_map_fixture.hpp>

#include <eltanin/collision/collision_checker.hpp>
#include <eltanin/collision/velocity_limiter.hpp>
#include <eltanin/control/follower_factory.hpp>
#include <eltanin/control/goal_approach.hpp>
#include <eltanin/control/path_follower.hpp>
#include <eltanin/core/geometry.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/core/polygon.hpp>
#include <eltanin/core/types.hpp>
#include <eltanin/map/cost_values.hpp>
#include <eltanin/map/layered_costmap.hpp>
#include <eltanin/map/layers/inflation_layer.hpp>
#include <eltanin/map/layers/obstacle_layer.hpp>
#include <eltanin/map/layers/static_layer.hpp>
#include <eltanin/map_io/pgm.hpp>
#include <eltanin/planner/astar_planner.hpp>
#include <eltanin/planner/hybrid_astar_planner.hpp>
#include <eltanin/planner/path_smoother.hpp>
#include <eltanin/sensor/scan_projection.hpp>
#include <eltanin/sim/simple_simulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <ostream>
#include <span>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace eltanin_examples
{

enum class PlannerType
{
  AStar,
  HybridAStar
};

inline const char * planner_name(PlannerType planner) noexcept
{
  return planner == PlannerType::HybridAStar ? "hybrid_astar" : "astar";
}

/// Every knob of the closed loop, so that the loop itself holds no magic number.
struct NavigateConfig
{
  PlannerType planner{PlannerType::AStar};
  eltanin::planner::AStarParams astar{};
  eltanin::planner::HybridAStarParams hybrid_astar{};
  /// Control period [s]; docs/control-design.md §5 measures the same tracking quality as 0.01.
  double control_dt{0.05};
  /// Cycles between two synthetic scans; 4 means 5 Hz at the default control period.
  int sensor_decimation{4};
  /// Side of the local window [m]; the derivation of 6.0 is in docs/integration-design.md §4.
  double local_window_size{6.0};
  /// Beams over the full circle; 180 puts them 0.105 m apart at range_max, about two cells.
  int lidar_beams{180};
  double lidar_range_min{0.1};
  /// Matches the local window half width; farther points would be dropped by the window anyway [m].
  double lidar_range_max{3.0};
  /// Ray marching step as a fraction of the map resolution.
  double raycast_step_scale{0.5};
  /// Final position error accepted as "reached" [m]; twice the 0.05 m path point spacing.
  double goal_tolerance{0.10};
  /// Simulated time bound [s]; divided by control_dt it gives the cycle limit.
  double max_sim_time{1000.0};
  int max_replans{3};
  /// Replan as soon as the observations block the path, instead of waiting for the limiter to stop.
  bool replan_on_blocked_path{true};
  /// How far along the path ahead of the robot that check looks [m].
  double path_check_distance{4.0};
  /// Consecutive cycles with an exactly zero command that trigger a replan; the fallback trigger.
  int stop_cycles_to_replan{5};
  /// Travel since the last replan below which another stop counts as a stall [m].
  double stall_min_progress{0.20};
  /// Where along the first planned path the unknown obstacle appears; 0 disables it.
  double obstacle_fraction{0.5};
  /// Half width of that obstacle [cells]; 4 gives a 9 x 9 cell = 0.45 m square.
  int obstacle_half_width_cells{4};
  bool verify_traversed{true};
  /// nullopt picks the start and the goal from the map, so no map coordinate is hard coded.
  std::optional<std::pair<eltanin::Pose2D, eltanin::Pose2D>> start_goal{};
  /// Footprint aside, these are the limiter defaults docs/collision-design.md was measured with.
  eltanin::collision::VelocityLimiterParams limiter{.footprint = robot_footprint()};
  eltanin::control::FollowerFactoryParams follower{};
  eltanin::control::GoalApproachParams goal_approach{};
};

/// Extra distance searched around the robot when measuring the clearance at a stop [m].
inline constexpr double CLEARANCE_SEARCH_MARGIN = 1.0;

/// Why the run ended; every value but Reached names the stage that failed.
enum class NavigateOutcome
{
  Reached,
  ModelFailed,
  StartGoalFailed,
  PlanFailed,
  PathTooShort,
  NoPath,
  GoalToleranceFailed,
  ReplanFailed,
  ReplanLimit,
  Stalled,
  StepLimit
};

inline const char * outcome_name(NavigateOutcome outcome) noexcept
{
  switch (outcome) {
    case NavigateOutcome::Reached:
      return "reached";
    case NavigateOutcome::ModelFailed:
      return "model_failed";
    case NavigateOutcome::StartGoalFailed:
      return "start_goal_failed";
    case NavigateOutcome::PlanFailed:
      return "plan_failed";
    case NavigateOutcome::PathTooShort:
      return "path_too_short";
    case NavigateOutcome::NoPath:
      return "no_path";
    case NavigateOutcome::GoalToleranceFailed:
      return "goal_tolerance_failed";
    case NavigateOutcome::ReplanFailed:
      return "replan_failed";
    case NavigateOutcome::ReplanLimit:
      return "replan_limit";
    case NavigateOutcome::Stalled:
      return "stalled";
    case NavigateOutcome::StepLimit:
      return "step_limit";
  }
  return "unknown";
}

/// One control cycle, recorded before the plant is stepped.
struct Sample
{
  std::size_t leg{0};
  double t{0.0};
  eltanin::Pose2D pose{};
  eltanin::Twist2D requested{};
  eltanin::Twist2D limited{};
  double collision_distance{std::numeric_limits<double>::infinity()};
  bool has_collision{false};
  std::size_t predicted_count{0};
};

/// Per-leg summary; a leg is one planned path, so every replan starts a new one.
struct LegStats
{
  std::size_t path_poses{0};
  double path_length{0.0};
  std::size_t cycles{0};
  std::size_t limited_cycles{0};
  std::size_t collision_cycles{0};
  std::size_t truncated_cycles{0};
  double min_collision_distance{std::numeric_limits<double>::infinity()};
  double max_speed_loss{0.0};
  double max_path_deviation{0.0};
};

/// A cell found occupied although the static map calls it free, plus the time it was found.
struct Observation
{
  double t{0.0};
  Eigen::Vector2d point{Eigen::Vector2d::Zero()};
};

struct NavigateResult
{
  NavigateOutcome outcome{NavigateOutcome::ModelFailed};
  /// Empty on success; otherwise it says which stage failed and with what numbers.
  std::string message;
  eltanin::Pose2D start{};
  eltanin::Pose2D goal{};
  std::vector<eltanin::Path> leg_paths;
  std::vector<LegStats> legs;
  std::vector<Sample> samples;
  std::vector<Observation> observations;
  std::size_t replans{0};
  /// Replans the observations asked for; the rest came from the limiter having stopped the robot.
  std::size_t replans_on_blocked_path{0};
  std::size_t global_updates{0};
  /// Cycles where the local window had to be clamped to stay inside the static map.
  std::size_t window_clamped_cycles{0};
  double final_position_error{0.0};
  double sim_time{0.0};
  std::size_t colliding_poses{0};
  /// Where the first collision happened; only meaningful when colliding_poses is not zero.
  eltanin::Pose2D first_colliding_pose{};
  std::optional<Eigen::Vector2d> obstacle_centre{};
  double obstacle_half_width{0.0};
  /// Footprint distance to the nearest lethal cell over the stopped cycles [m].
  double stop_clearance{std::numeric_limits<double>::infinity()};
  /// Same, restricted to the injected obstacle the robot stopped for [m].
  double stop_obstacle_clearance{std::numeric_limits<double>::infinity()};
  /// Belief costmap as it stood at the end of the run; its geometry is the static map's.
  eltanin::map::Costmap global_costmap;
  /// Inflated truth the synthetic sensor reads and the traversed poses are verified against.
  eltanin::map::Costmap ground_truth;

  bool reached() const noexcept { return outcome == NavigateOutcome::Reached; }
};

namespace detail
{

/// Lethal block stamped onto the ground truth only, so the planner cannot know about it.
struct StampedObstacle
{
  Eigen::Vector2d centre{Eigen::Vector2d::Zero()};
  eltanin::map::CellRect cells{};
};

/// Stamps the block at `fraction` along the path; nullopt when it is disabled or lands off the map.
inline std::optional<StampedObstacle> stamp_obstacle(
  eltanin::map::Costmap & truth, const eltanin::Path & path, double fraction, int half_width_cells)
{
  if (fraction <= 0.0 || path.empty()) {
    return std::nullopt;
  }
  const std::size_t index = std::min(
    path.size() - 1, static_cast<std::size_t>(fraction * static_cast<double>(path.size())));
  const Eigen::Vector2d centre = path[index].position;
  const std::optional<eltanin::map::MapIndex> cell = truth.geometry().world_to_map(centre);
  if (!cell.has_value()) {
    return std::nullopt;
  }
  for (int dy = -half_width_cells; dy <= half_width_cells; ++dy) {
    for (int dx = -half_width_cells; dx <= half_width_cells; ++dx) {
      truth.set(cell->x + dx, cell->y + dy, eltanin::map::LETHAL_OBSTACLE);
    }
  }
  const eltanin::map::MapGeometry & geometry = truth.geometry();
  const eltanin::map::CellRect cells{
    std::max(0, cell->x - half_width_cells), std::max(0, cell->y - half_width_cells),
    std::min(geometry.size_x() - 1, cell->x + half_width_cells),
    std::min(geometry.size_y() - 1, cell->y + half_width_cells)};
  return StampedObstacle{centre, cells};
}

/// Range of one beam [m]; +inf when it leaves the map or reaches range_max without a hit.
inline float cast_ray(
  const eltanin::map::Costmap & truth, const Eigen::Vector2d & sensor, double angle,
  double range_min, double range_max, double step)
{
  const Eigen::Vector2d direction{std::cos(angle), std::sin(angle)};
  const int steps = static_cast<int>(std::floor((range_max - range_min) / step)) + 1;
  for (int i = 0; i < steps; ++i) {
    const double range = range_min + static_cast<double>(i) * step;
    const std::optional<eltanin::map::MapIndex> cell =
      truth.geometry().world_to_map(sensor + range * direction);
    if (!cell.has_value()) {
      break;
    }
    if (truth(cell->x, cell->y) == eltanin::map::LETHAL_OBSTACLE) {
      return static_cast<float>(range);
    }
  }
  return std::numeric_limits<float>::infinity();
}

/// Fills `scan.ranges` for a full-circle sensor at the robot origin, in the sensor frame.
inline void cast_scan(
  const eltanin::map::Costmap & truth, const eltanin::Pose2D & sensor,
  const NavigateConfig & config, eltanin::sensor::ScanData & scan)
{
  const double step = config.raycast_step_scale * truth.geometry().resolution();
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    scan.ranges[i] = cast_ray(
      truth, sensor.position, sensor.yaw + angle, config.lidar_range_min, config.lidar_range_max,
      step);
  }
}

/// Drops points whose cell is not lethal in the truth, so no free cell is ever marked occupied.
inline void keep_points_on_obstacles(
  const eltanin::map::Costmap & truth, std::vector<Eigen::Vector2d> & points)
{
  const auto off_obstacle = [&truth](const Eigen::Vector2d & point) {
    const std::optional<eltanin::map::MapIndex> cell = truth.geometry().world_to_map(point);
    return !cell.has_value() || truth(cell->x, cell->y) != eltanin::map::LETHAL_OBSTACLE;
  };
  points.erase(std::remove_if(points.begin(), points.end(), off_obstacle), points.end());
}

/// Window origin snapped to the static grid and clamped inside it, so StaticLayer resamples 1:1.
inline Eigen::Vector2d snapped_window_origin(
  const eltanin::map::MapGeometry & static_geometry, int window_cells,
  const Eigen::Vector2d & robot, bool & clamped)
{
  const double resolution = static_geometry.resolution();
  const double half = 0.5 * static_cast<double>(window_cells) * resolution;
  const double limits[2] = {
    static_cast<double>(static_geometry.size_x() - window_cells),
    static_cast<double>(static_geometry.size_y() - window_cells)};
  Eigen::Vector2d origin = Eigen::Vector2d::Zero();
  clamped = false;
  for (int axis = 0; axis < 2; ++axis) {
    const double base = static_geometry.origin()[axis];
    const double raw = std::floor((robot[axis] - half - base) / resolution);
    const double index = std::clamp(raw, 0.0, std::max(0.0, limits[axis]));
    if (index != raw) {
      clamped = true;
    }
    origin[axis] = base + index * resolution;
  }
  return origin;
}

/// Exact distance from the world footprint to the nearest lethal cell centre in `rect`; 0 if inside.
inline double footprint_clearance(
  const eltanin::map::Costmap & truth, const eltanin::Polygon2D & world_footprint,
  const eltanin::map::CellRect & rect)
{
  const eltanin::map::MapGeometry & geometry = truth.geometry();
  double clearance = std::numeric_limits<double>::infinity();
  for (int my = rect.min_y; my <= rect.max_y; ++my) {
    for (int mx = rect.min_x; mx <= rect.max_x; ++mx) {
      if (truth(mx, my) != eltanin::map::LETHAL_OBSTACLE) {
        continue;
      }
      const Eigen::Vector2d centre = geometry.map_to_world(mx, my);
      if (eltanin::contains(world_footprint, centre)) {
        return 0.0;
      }
      for (std::size_t v = 0; v < world_footprint.size(); ++v) {
        const Eigen::Vector2d & a = world_footprint[v];
        const Eigen::Vector2d & b = world_footprint[(v + 1) % world_footprint.size()];
        clearance = std::min(clearance, eltanin::distance_to_segment(centre, a, b));
      }
    }
  }
  return clearance;
}

/// Same, over every lethal cell within `reach` of `position`.
inline double footprint_clearance_around(
  const eltanin::map::Costmap & truth, const eltanin::Polygon2D & world_footprint,
  const Eigen::Vector2d & position, double reach)
{
  const std::optional<eltanin::map::CellRect> rect = truth.geometry().world_rect_to_cells(
    position - Eigen::Vector2d::Constant(reach), position + Eigen::Vector2d::Constant(reach));
  if (!rect.has_value()) {
    return std::numeric_limits<double>::infinity();
  }
  return footprint_clearance(truth, world_footprint, *rect);
}

/// True when an observed point sits within `radius` of the path over the next `distance` metres.
inline bool path_blocked_ahead(
  const eltanin::Path & path, const Eigen::Vector2d & robot,
  std::span<const Eigen::Vector2d> points, double radius, double distance)
{
  if (path.size() < 2 || points.empty()) {
    return false;
  }
  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double squared = (path[i].position - robot).squaredNorm();
    if (squared < best) {
      best = squared;
      nearest = i;
    }
  }
  // The radius is the circumscribed one, so this asks exactly what A* asks of a cell: is it Free.
  const double squared_radius = radius * radius;
  double travelled = 0.0;
  for (std::size_t i = nearest; i + 1 < path.size() && travelled <= distance; ++i) {
    travelled += (path[i + 1].position - path[i].position).norm();
    for (const Eigen::Vector2d & point : points) {
      if ((path[i + 1].position - point).squaredNorm() <= squared_radius) {
        return true;
      }
    }
  }
  return false;
}

/// Minimum distance from `position` to the path polyline [m]; as in examples/track_on_real_map.cpp.
inline double lateral_error(const eltanin::Path & path, const Eigen::Vector2d & position)
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

/// The guide only fixes the corridor, so smoothing it would be wasted work.
inline eltanin::planner::AStarParams guide_params()
{
  eltanin::planner::AStarParams params;
  params.smoother.reset();
  return params;
}

/// Plans one leg on the belief costmap; the error says why nothing came back.
inline eltanin::planner::PlanResult plan_leg(
  const eltanin::map::Costmap & belief, const RobotModel & robot, const eltanin::Pose2D & from,
  const eltanin::Pose2D & to, const NavigateConfig & config)
{
  if (config.planner == PlannerType::AStar) {
    return eltanin::planner::AStarPlanner(config.astar).plan(belief, robot.model, from, to);
  }

  // Hybrid A* is bounded by memory, so it searches a corridor around a raw A* guide.
  const auto guide =
    eltanin::planner::AStarPlanner(guide_params()).plan(belief, robot.model, from, to);
  if (!guide) {
    return guide;
  }
  eltanin::map::MapIndex lower_left{0, 0};
  const eltanin::map::Costmap corridor = crop_around(belief, positions_of(*guide), lower_left);
  return eltanin::planner::HybridAStarPlanner(config.hybrid_astar)
    .plan(corridor, robot.model, from, to);
}

inline LegStats leg_stats_for(const eltanin::Path & path)
{
  LegStats stats;
  stats.path_poses = path.size();
  stats.path_length = eltanin::path_length(path);
  return stats;
}

/// Folds one cycle into the statistics of its leg; `full_prediction` is prediction_steps + 1.
inline void accumulate_sample(
  LegStats & stats, const Sample & sample, const eltanin::Path & path,
  std::size_t full_prediction)
{
  ++stats.cycles;
  const double speed_loss =
    std::abs(sample.requested.linear.x()) - std::abs(sample.limited.linear.x());
  if (speed_loss > 1e-12) {
    ++stats.limited_cycles;
    stats.max_speed_loss = std::max(stats.max_speed_loss, speed_loss);
  }
  if (sample.has_collision) {
    ++stats.collision_cycles;
    stats.min_collision_distance =
      std::min(stats.min_collision_distance, sample.collision_distance);
  } else if (sample.predicted_count < full_prediction) {
    ++stats.truncated_cycles;
  }
  stats.max_path_deviation =
    std::max(stats.max_path_deviation, lateral_error(path, sample.pose.position));
}

inline std::string pose_text(const eltanin::Pose2D & pose)
{
  return "(" + std::to_string(pose.position.x()) + ", " + std::to_string(pose.position.y()) + ")";
}

}  // namespace detail

/// Runs the whole stack until the goal or a failure; `static_map` is read only and copied inside.
inline NavigateResult navigate(
  const eltanin::map::Costmap & static_map, const RobotModel & robot,
  const NavigateConfig & config)
{
  using eltanin::Pose2D;
  using eltanin::collision::CollisionCheck;
  using eltanin::collision::VelocityLimiter;
  using eltanin::control::GoalApproach;
  using eltanin::control::PathFollower;
  using eltanin::map::Costmap;
  using eltanin::map::InflationLayer;
  using eltanin::map::LayeredCostmap;
  using eltanin::map::MapGeometry;
  using eltanin::map::MapIndex;
  using eltanin::map::ObstacleLayer;
  using eltanin::map::StaticLayer;

  NavigateResult result;
  const MapGeometry & static_geometry = static_map.geometry();
  const double resolution = static_geometry.resolution();
  const bool valid_config =
    std::isfinite(config.control_dt) && config.control_dt > 0.0 &&
    config.sensor_decimation >= 1 && std::isfinite(config.local_window_size) &&
    config.local_window_size > 0.0 && config.lidar_beams >= 1 &&
    std::isfinite(config.lidar_range_min) && config.lidar_range_min >= 0.0 &&
    std::isfinite(config.lidar_range_max) &&
    config.lidar_range_max >= config.lidar_range_min &&
    std::isfinite(config.raycast_step_scale) && config.raycast_step_scale > 0.0 &&
    std::isfinite(config.goal_tolerance) && config.goal_tolerance > 0.0 &&
    std::isfinite(config.max_sim_time) && config.max_sim_time > 0.0 &&
    config.max_replans >= 0 && std::isfinite(config.path_check_distance) &&
    config.path_check_distance >= 0.0 && config.stop_cycles_to_replan >= 1 &&
    std::isfinite(config.stall_min_progress) && config.stall_min_progress >= 0.0 &&
    std::isfinite(config.obstacle_fraction) && config.obstacle_fraction >= 0.0 &&
    config.obstacle_fraction <= 1.0 && config.obstacle_half_width_cells >= 0 &&
    resolution > 0.0 && static_geometry.cell_count() > 0;
  if (!valid_config) {
    result.outcome = NavigateOutcome::ModelFailed;
    result.message = "invalid navigation configuration or empty static map";
    return result;
  }
  if (
    config.start_goal.has_value() &&
    (!config.start_goal->first.position.allFinite() ||
     !std::isfinite(config.start_goal->first.yaw) ||
     !config.start_goal->second.position.allFinite() ||
     !std::isfinite(config.start_goal->second.yaw))) {
    result.outcome = NavigateOutcome::StartGoalFailed;
    result.message = "start and goal poses must be finite";
    return result;
  }

  eltanin::control::GoalApproachParams goal_approach_params = config.goal_approach;
  goal_approach_params.xy_goal_tolerance = config.goal_tolerance;
  eltanin::control::FollowerResult built = eltanin::control::make_path_follower(config.follower);
  std::optional<GoalApproach> goal_approach = GoalApproach::create(goal_approach_params);
  std::optional<VelocityLimiter> limiter = VelocityLimiter::create(config.limiter);
  if (!built.has_value() || !goal_approach.has_value() || !limiter.has_value()) {
    result.outcome = NavigateOutcome::ModelFailed;
    result.message = built.has_value()
                       ? "GoalApproach::create or VelocityLimiter::create rejected its parameters"
                       : std::string("make_path_follower failed: ") +
                           eltanin::control::to_string(built.error());
    return result;
  }
  const std::unique_ptr<PathFollower> follower = built.take();
  const eltanin::Polygon2D & footprint = limiter->footprint();

  LayeredCostmap global(static_geometry, eltanin::map::NO_INFORMATION);
  global.add_layer<StaticLayer>(static_map);
  ObstacleLayer & global_obstacles = global.add_layer<ObstacleLayer>();
  global.add_layer<InflationLayer>(robot.inflation, false);
  global.update();
  ++result.global_updates;

  Pose2D start;
  Pose2D goal;
  if (config.start_goal.has_value()) {
    start = config.start_goal->first;
    goal = config.start_goal->second;
  } else {
    const auto pair = auto_start_goal(global.costmap(), robot.model);
    if (!pair.has_value()) {
      result.outcome = NavigateOutcome::StartGoalFailed;
      result.message = "auto_start_goal found no reachable pair of cells on the inflated map";
      return result;
    }
    start = pair->first;
    goal = pair->second;
  }
  result.start = start;
  result.goal = goal;

  std::optional<eltanin::Path> path;
  try {
    const eltanin::planner::PlanResult planned =
      detail::plan_leg(global.costmap(), robot, start, goal, config);
    if (!planned) {
      result.outcome = NavigateOutcome::PlanFailed;
      result.message = std::string(planner_name(config.planner)) + " found no path from " +
                       detail::pose_text(start) + " to " + detail::pose_text(goal) + ": " +
                       eltanin::planner::to_string(planned.error());
      return result;
    }
    path = planned.path();
  } catch (const std::invalid_argument & error) {
    result.outcome = NavigateOutcome::ModelFailed;
    result.message = error.what();
    return result;
  }
  if (path->size() < 2) {
    result.outcome = NavigateOutcome::PathTooShort;
    result.message = "the initial path has fewer than two poses";
    return result;
  }

  Costmap ground_truth = static_map;
  const std::optional<detail::StampedObstacle> obstacle = detail::stamp_obstacle(
    ground_truth, *path, config.obstacle_fraction, config.obstacle_half_width_cells);
  // The truth is inflated too, or check_footprint would never reach its exact stage.
  InflationLayer(robot.inflation, false).update_costs(ground_truth);
  if (obstacle.has_value()) {
    result.obstacle_centre = obstacle->centre;
    result.obstacle_half_width =
      (static_cast<double>(config.obstacle_half_width_cells) + 0.5) * resolution;
  }

  // A window wider than the map would leave NO_INFORMATION cells the limiter reads as obstacles.
  const double max_window_size =
    static_cast<double>(std::min(static_geometry.size_x(), static_geometry.size_y())) * resolution;
  if (config.local_window_size < resolution || config.local_window_size > max_window_size) {
    result.outcome = NavigateOutcome::ModelFailed;
    result.message = "local window must fit inside the static map";
    return result;
  }
  const int window_cells = static_cast<int>(std::lround(config.local_window_size / resolution));
  LayeredCostmap local(
    MapGeometry(window_cells, window_cells, resolution, static_geometry.origin()),
    eltanin::map::NO_INFORMATION);
  local.add_layer<StaticLayer>(static_map);
  ObstacleLayer & local_obstacles = local.add_layer<ObstacleLayer>();
  local.add_layer<InflationLayer>(robot.inflation, false);

  eltanin::sensor::ScanData scan;
  scan.angle_min = -std::numbers::pi;
  scan.angle_increment = 2.0 * std::numbers::pi / static_cast<double>(config.lidar_beams);
  scan.range_min = config.lidar_range_min;
  scan.range_max = config.lidar_range_max;
  scan.ranges.assign(static_cast<std::size_t>(config.lidar_beams), 0.0F);
  const eltanin::sensor::ScanFilter filter{};
  std::vector<Eigen::Vector2d> points;

  std::vector<Eigen::Vector2d> observed_points;
  std::unordered_set<std::size_t> observed_cells;
  std::size_t observed_at_last_update = 0;

  result.leg_paths.push_back(*path);
  result.legs.push_back(detail::leg_stats_for(*path));

  eltanin::sim::SimpleSimulator plant(start);
  const std::size_t max_steps = static_cast<std::size_t>(config.max_sim_time / config.control_dt);
  const std::size_t full_prediction =
    static_cast<std::size_t>(limiter->params().prediction_steps) + 1;
  const double clearance_reach = robot.distance_model.circumscribed_radius() + CLEARANCE_SEARCH_MARGIN;
  std::size_t leg = 0;
  int zero_cycles = 0;
  /// What the plant was last told to do; the follower reads it instead of guessing its own state.
  std::optional<eltanin::Twist2D> measured{};
  double progress_since_replan = 0.0;
  bool path_blocked = false;
  bool at_goal = false;

  for (std::size_t step = 0; step < max_steps; ++step) {
    const double t = static_cast<double>(step) * config.control_dt;
    result.sim_time = t;

    if (step % static_cast<std::size_t>(config.sensor_decimation) == 0) {
      detail::cast_scan(ground_truth, plant.pose(), config, scan);
      eltanin::sensor::project_scan(
        scan, filter, eltanin::Transform2D::from_pose(plant.pose()), points);
      detail::keep_points_on_obstacles(ground_truth, points);
      for (const Eigen::Vector2d & point : points) {
        const std::optional<MapIndex> cell = static_geometry.world_to_map(point);
        if (!cell.has_value() || static_map(cell->x, cell->y) == eltanin::map::LETHAL_OBSTACLE) {
          continue;
        }
        if (observed_cells.insert(static_geometry.index(cell->x, cell->y)).second) {
          const Eigen::Vector2d centre = static_geometry.map_to_world(cell->x, cell->y);
          observed_points.push_back(centre);
          result.observations.push_back(Observation{t, centre});
        }
      }
      if (config.replan_on_blocked_path) {
        path_blocked = detail::path_blocked_ahead(
          *path, plant.pose().position, observed_points, robot.distance_model.circumscribed_radius(),
          config.path_check_distance);
      }
      // The origin and the cells have to move together: set_origin() alone relabels stale cells.
      bool clamped = false;
      local.set_origin(detail::snapped_window_origin(
        static_geometry, window_cells, plant.pose().position, clamped));
      result.window_clamped_cycles += clamped ? 1 : 0;
      local_obstacles.set_points(points);
      local.update();
    }

    const GoalApproach::Result approach =
      goal_approach->compute(plant.pose(), *path, config.control_dt);
    if (approach.state == GoalApproach::State::Reached) {
      at_goal = true;
      break;
    }
    if (approach.state == GoalApproach::State::AlignmentTimeout) {
      result.outcome = NavigateOutcome::GoalToleranceFailed;
      result.message = "goal yaw alignment timed out on leg " + std::to_string(leg);
      break;
    }

    eltanin::Twist2D requested;
    if (approach.state == GoalApproach::State::Aligning) {
      requested = approach.command;
    } else {
      const eltanin::control::FollowResult tracking = follower->follow(
        eltanin::control::FollowerState{plant.pose(), measured}, *path, config.control_dt);
      if (tracking.status == eltanin::control::FollowStatus::NoPath) {
        result.outcome = NavigateOutcome::NoPath;
        result.message = "the follower reported NoPath on leg " + std::to_string(leg);
        break;
      }
      if (tracking.status == eltanin::control::FollowStatus::GoalReached) {
        result.outcome = NavigateOutcome::GoalToleranceFailed;
        result.message =
          "the follower reached the last path pose before GoalApproach accepted the goal";
        break;
      }
      requested = eltanin::control::detail::apply_linear_limit(
        tracking.command, approach.linear_vel_limit);
    }

    const VelocityLimiter::Result limited =
      limiter->limit(local.costmap(), robot.model, plant.pose(), requested);

    result.samples.push_back(Sample{
      leg, t, plant.pose(), requested, limited.command, limited.collision_distance,
      limited.has_collision, limited.predicted_poses.size()});
    detail::accumulate_sample(result.legs[leg], result.samples.back(), *path, full_prediction);

    if (limited.command.linear.x() == 0.0 && limited.command.angular == 0.0) {
      ++zero_cycles;
      const eltanin::Polygon2D world_footprint = eltanin::transform(footprint, plant.pose());
      result.stop_clearance = std::min(
        result.stop_clearance,
        detail::footprint_clearance_around(
          ground_truth, world_footprint, plant.pose().position, clearance_reach));
      if (obstacle.has_value()) {
        result.stop_obstacle_clearance = std::min(
          result.stop_obstacle_clearance,
          detail::footprint_clearance(ground_truth, world_footprint, obstacle->cells));
      }
    } else {
      zero_cycles = 0;
    }

    const bool stopped_too_long = zero_cycles >= config.stop_cycles_to_replan;
    if (path_blocked || stopped_too_long) {
      // A stall means replanning did not help, so the first stop always gets one replan.
      if (stopped_too_long && result.replans > 0 &&
          progress_since_replan < config.stall_min_progress) {
        result.outcome = NavigateOutcome::Stalled;
        result.message = "stalled at " + detail::pose_text(plant.pose()) + " after moving only " +
                         std::to_string(progress_since_replan) + " m since the last plan";
        break;
      }
      if (result.replans >= static_cast<std::size_t>(config.max_replans)) {
        result.outcome = NavigateOutcome::ReplanLimit;
        result.message = "the replan limit of " + std::to_string(config.max_replans) +
                         " was reached at " + detail::pose_text(plant.pose());
        break;
      }
      global_obstacles.set_points(observed_points);
      global.update();
      ++result.global_updates;
      observed_at_last_update = observed_points.size();
      const eltanin::planner::PlanResult replanned =
        detail::plan_leg(global.costmap(), robot, plant.pose(), goal, config);
      if (!replanned) {
        result.outcome = NavigateOutcome::ReplanFailed;
        result.message = "replan " + std::to_string(result.replans + 1) +
                         " failed: " + planner_name(config.planner) + " found no path from " +
                         detail::pose_text(plant.pose()) + " to " + detail::pose_text(goal) + ": " +
                         eltanin::planner::to_string(replanned.error());
        break;
      }
      if (replanned->size() < 2) {
        result.outcome = NavigateOutcome::PathTooShort;
        result.message = "the path of replan " + std::to_string(result.replans + 1) +
                         " has fewer than two poses";
        break;
      }
      path = replanned.path();
      goal_approach->reset();
      measured.reset();
      // A follower owns progress and heading state for one path; a replacement must reset both.
      follower->reset();
      if (!stopped_too_long) {
        ++result.replans_on_blocked_path;
      }
      ++result.replans;
      ++leg;
      result.leg_paths.push_back(*path);
      result.legs.push_back(detail::leg_stats_for(*path));
      progress_since_replan = 0.0;
      zero_cycles = 0;
      path_blocked = false;
    }

    const Eigen::Vector2d before = plant.pose().position;
    plant.update(limited.command, config.control_dt);
    measured = limited.command;
    progress_since_replan += (plant.pose().position - before).norm();

    // Verifying after the step checks the pose the limited command actually produced.
    if (config.verify_traversed) {
      const CollisionCheck check =
        eltanin::collision::check_footprint(ground_truth, robot.model, footprint, plant.pose());
      if (check == CollisionCheck::Collision) {
        if (result.colliding_poses == 0) {
          result.first_colliding_pose = plant.pose();
        }
        ++result.colliding_poses;
      }
    }
  }

  result.final_position_error = (plant.pose().position - goal.position).norm();
  if (at_goal) {
    if (result.final_position_error <= config.goal_tolerance) {
      result.outcome = NavigateOutcome::Reached;
    } else {
      result.outcome = NavigateOutcome::GoalToleranceFailed;
      result.message = "the follower reported GoalReached but the final position error is " +
                       std::to_string(result.final_position_error) + " m";
    }
  } else if (result.message.empty()) {
    result.outcome = NavigateOutcome::StepLimit;
    result.message = "the cycle limit of " + std::to_string(max_steps) + " was reached";
  }

  // The master is only regenerated when replanning, so the dump would otherwise miss late points.
  if (observed_points.size() > observed_at_last_update) {
    global_obstacles.set_points(observed_points);
    global.update();
    ++result.global_updates;
  }
  result.global_costmap = global.costmap();
  result.ground_truth = std::move(ground_truth);
  return result;
}

/// Planned paths of every leg; the existing write_path_csv has no leg column.
inline bool write_leg_paths_csv(
  const std::filesystem::path & file, const std::vector<eltanin::Path> & legs)
{
  std::ofstream out(file);
  if (!out) {
    return false;
  }
  out << "leg,index,x,y,yaw\n";
  for (std::size_t leg = 0; leg < legs.size(); ++leg) {
    for (std::size_t i = 0; i < legs[leg].size(); ++i) {
      const eltanin::Pose2D & pose = legs[leg][i];
      out << leg << ',' << i << ',' << pose.position.x() << ',' << pose.position.y() << ','
          << pose.yaw << '\n';
    }
  }
  return static_cast<bool>(out);
}

inline bool write_trajectory_csv(
  const std::filesystem::path & file, const std::vector<Sample> & samples)
{
  std::ofstream out(file);
  if (!out) {
    return false;
  }
  out << "t,leg,x,y,yaw,v_in,w_in,v_out,w_out,collision_distance,has_collision,predicted_poses\n";
  for (const Sample & sample : samples) {
    out << sample.t << ',' << sample.leg << ',' << sample.pose.position.x() << ','
        << sample.pose.position.y() << ',' << sample.pose.yaw << ',' << sample.requested.linear.x()
        << ',' << sample.requested.angular << ',' << sample.limited.linear.x() << ','
        << sample.limited.angular << ',' << sample.collision_distance << ','
        << static_cast<int>(sample.has_collision) << ',' << sample.predicted_count << '\n';
  }
  return static_cast<bool>(out);
}

/// The observations, in discovery order.
inline bool write_obstacles_csv(
  const std::filesystem::path & file, const std::vector<Observation> & observations)
{
  std::ofstream out(file);
  if (!out) {
    return false;
  }
  out << "t,x,y\n";
  for (const Observation & observation : observations) {
    out << observation.t << ',' << observation.point.x() << ',' << observation.point.y() << '\n';
  }
  return static_cast<bool>(out);
}

/// Writes costmap.pgm, traversed.pgm, path.csv, trajectory.csv, obstacles.csv and meta.txt.
inline bool write_output_files(
  const std::filesystem::path & directory, const NavigateConfig & config, const RobotModel & robot,
  const NavigateResult & result)
{
  std::error_code error;
  std::filesystem::create_directories(directory, error);
  if (!std::filesystem::is_directory(directory)) {
    std::cerr << "failed to create " << directory << ": " << error.message() << '\n';
    return false;
  }

  std::vector<Eigen::Vector2d> drawn;
  for (const eltanin::Path & leg : result.leg_paths) {
    for (const eltanin::Pose2D & pose : leg) {
      drawn.push_back(pose.position);
    }
  }
  for (const Sample & sample : result.samples) {
    drawn.push_back(sample.pose.position);
  }
  if (drawn.empty()) {
    std::cerr << "there is nothing to crop around: neither a path nor a trajectory\n";
    return false;
  }

  eltanin::map::MapIndex lower_left{0, 0};
  const eltanin::map::Costmap crop = crop_around(result.global_costmap, drawn, lower_left);
  eltanin::map::Costmap traversed(crop.geometry(), eltanin::map::FREE_SPACE);
  for (const Sample & sample : result.samples) {
    const auto cell = crop.geometry().world_to_map(sample.pose.position);
    if (cell.has_value()) {
      traversed(cell->x, cell->y) = eltanin::map::LETHAL_OBSTACLE;
    }
  }
  try {
    eltanin::map_io::write_pgm(directory / "costmap.pgm", crop);
    eltanin::map_io::write_pgm(directory / "traversed.pgm", traversed);
  } catch (const eltanin::map_io::MapIoError & io_error) {
    std::cerr << "failed to write the PGM files: " << io_error.what() << '\n';
    return false;
  }

  if (
    !write_leg_paths_csv(directory / "path.csv", result.leg_paths) ||
    !write_trajectory_csv(directory / "trajectory.csv", result.samples) ||
    !write_obstacles_csv(directory / "obstacles.csv", result.observations)) {
    std::cerr << "failed to write the CSV files into " << directory << '\n';
    return false;
  }

  std::ofstream meta(directory / "meta.txt");
  if (!meta) {
    std::cerr << "failed to write meta.txt into " << directory << '\n';
    return false;
  }
  write_meta(meta, crop, lower_left, robot.inflation, robot.distance_model);
  const eltanin::collision::VelocityLimiterParams & limits = config.limiter;
  meta << "planner " << planner_name(config.planner) << '\n'
       << "follower " << eltanin::control::name_of(config.follower.type) << '\n'
       << "control_dt " << config.control_dt << '\n'
       << "sensor_decimation " << config.sensor_decimation << '\n'
       << "local_window_size " << config.local_window_size << '\n'
       << "lidar_beams " << config.lidar_beams << '\n'
       << "lidar_range_min " << config.lidar_range_min << '\n'
       << "lidar_range_max " << config.lidar_range_max << '\n'
       << "raycast_step " << config.raycast_step_scale * crop.geometry().resolution() << '\n'
       << "prediction_steps " << limits.prediction_steps << '\n'
       << "prediction_time " << limits.prediction_time << '\n'
       << "prediction_dt " << limits.prediction_time / static_cast<double>(limits.prediction_steps)
       << '\n'
       << "collision_margin " << limits.collision_margin << '\n'
       << "max_deceleration " << limits.max_deceleration << '\n'
       << "goal_tolerance " << config.goal_tolerance << '\n'
       << "max_replans " << config.max_replans << '\n'
       << "replan_on_blocked_path " << static_cast<int>(config.replan_on_blocked_path) << '\n'
       << "path_check_distance " << config.path_check_distance << '\n'
       << "stop_cycles_to_replan " << config.stop_cycles_to_replan << '\n'
       << "stall_min_progress " << config.stall_min_progress << '\n'
       << "start_x " << result.start.position.x() << '\n'
       << "start_y " << result.start.position.y() << '\n'
       << "goal_x " << result.goal.position.x() << '\n'
       << "goal_y " << result.goal.position.y() << '\n'
       << "legs " << result.legs.size() << '\n';
  for (std::size_t leg = 0; leg < result.legs.size(); ++leg) {
    const LegStats & stats = result.legs[leg];
    meta << "leg " << leg << " path_poses " << stats.path_poses << " path_length "
         << stats.path_length << " cycles " << stats.cycles << " limited_cycles "
         << stats.limited_cycles << " collision_cycles " << stats.collision_cycles
         << " truncated_cycles " << stats.truncated_cycles << " min_collision_distance "
         << stats.min_collision_distance << " max_speed_loss " << stats.max_speed_loss
         << " max_path_deviation " << stats.max_path_deviation << '\n';
  }
  meta << "cycles " << result.samples.size() << '\n'
       << "sim_time " << result.sim_time << '\n'
       << "replans " << result.replans << '\n'
       << "replans_on_blocked_path " << result.replans_on_blocked_path << '\n'
       << "global_updates " << result.global_updates << '\n'
       << "window_clamped_cycles " << result.window_clamped_cycles << '\n'
       << "observations " << result.observations.size() << '\n'
       << "final_position_error " << result.final_position_error << '\n'
       << "colliding_poses " << result.colliding_poses << '\n'
       << "outcome " << outcome_name(result.outcome) << '\n'
       << "obstacle_fraction " << config.obstacle_fraction << '\n';
  if (result.colliding_poses > 0) {
    meta << "first_colliding_x " << result.first_colliding_pose.position.x() << '\n'
         << "first_colliding_y " << result.first_colliding_pose.position.y() << '\n'
         << "first_colliding_yaw " << result.first_colliding_pose.yaw << '\n';
  }
  if (std::isfinite(result.stop_clearance)) {
    meta << "stop_clearance " << result.stop_clearance << '\n';
  }
  if (result.obstacle_centre.has_value()) {
    meta << "obstacle_x " << result.obstacle_centre->x() << '\n'
         << "obstacle_y " << result.obstacle_centre->y() << '\n'
         << "obstacle_half_width " << result.obstacle_half_width << '\n'
         << "stop_obstacle_clearance " << result.stop_obstacle_clearance << '\n';
  }
  return static_cast<bool>(meta);
}

}  // namespace eltanin_examples

#endif  // ELTANIN_EXAMPLES__NAVIGATION_LOOP_HPP_
