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

#ifndef ELTANIN__COLLISION__VELOCITY_LIMITER_HPP_
#define ELTANIN__COLLISION__VELOCITY_LIMITER_HPP_

#include <eltanin/core/differential_drive.hpp>
#include <eltanin/core/polygon.hpp>
#include <eltanin/core/traversability.hpp>
#include <eltanin/core/types.hpp>
#include <eltanin/map/cell_map.hpp>
#include <eltanin/collision/collision_checker.hpp>

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace eltanin::collision
{

struct VelocityLimiterParams
{
  /// Robot footprint in the base frame; create() stores it normalized to counter-clockwise.
  Polygon2D footprint{
    Eigen::Vector2d{-0.3, 0.3}, Eigen::Vector2d{-0.3, -0.3}, Eigen::Vector2d{0.3, -0.3},
    Eigen::Vector2d{0.3, 0.3}};
  /// Number of prediction steps spread over the derived horizon; the only resolution knob.
  int prediction_steps{10};
  /// Latency budget [s]; the horizon floor at rest and the closing-speed cap near an obstacle.
  double reaction_time{0.3};
  /// Distance kept in front of the predicted collision [m].
  double collision_margin{0.2};
  /// Deceleration used by the braking-distance law [m/s^2].
  double max_deceleration{0.5};
  /// Drop the Free short-circuit of the centre cell; required for maps that carry no inflation.
  bool exact_footprint_check{true};
  /// Clearance [m] at or below which the proximity ramp sits at min_proximity_scale.
  double stop_clearance{0.10};
  /// Clearance [m] at or above which the proximity ramp does not slow the command down at all.
  double slow_down_clearance{0.50};
  /// Floor of the proximity ramp; strictly positive, or a narrow corridor becomes impassable.
  double min_proximity_scale{0.25};
};

namespace detail
{

/// Linear velocities below this magnitude [m/s] carry no ratio.
inline constexpr double MIN_LINEAR_VEL = 1e-9;

/// Bisections inside the colliding step; fixed, because the resolution of a stop is not a setting.
inline constexpr int COLLISION_REFINEMENT_STEPS = 4;

/// Time [s] the requested command needs to close the gap down to the margin; +inf when it never does.
double time_to_collision(
  const VelocityLimiterParams & params, const Twist2D & cmd_in, double collision_distance);

/// Braking-distance and reaction-time laws on the magnitude of the command; the sign is preserved.
Twist2D limit_command(
  const VelocityLimiterParams & params, const Twist2D & cmd_in, bool has_collision,
  double collision_distance, double proximity_scale);

/// Ramp from min_proximity_scale at stop_clearance to 1.0 at slow_down_clearance.
double proximity_scale(const VelocityLimiterParams & params, double clearance);

/// Dispatches to the exact or the two-stage footprint check according to the parameters.
template <map::CellMap Map, class Model>
  requires TraversabilityModel<Model, typename Map::value_type> &&
           ObstacleModel<Model, typename Map::value_type>
CollisionCheck check_pose(
  const VelocityLimiterParams & params, const Map & map, const Model & model, const Pose2D & pose)
{
  return params.exact_footprint_check ? check_footprint_exact(map, model, params.footprint, pose)
                                      : check_footprint(map, model, params.footprint, pose);
}

/// Largest sub-step time in [0, dt] that is still free; leaving the map counts as colliding.
template <map::CellMap Map, class Model>
  requires TraversabilityModel<Model, typename Map::value_type> &&
           ObstacleModel<Model, typename Map::value_type>
double last_free_time(
  const VelocityLimiterParams & params, const Map & map, const Model & model, const Pose2D & from,
  const Twist2D & cmd_in, double dt)
{
  double free_time = 0.0;
  double blocked_time = dt;
  for (int step = 0; step < COLLISION_REFINEMENT_STEPS; ++step) {
    const double middle = 0.5 * (free_time + blocked_time);
    const Pose2D pose = integrate_differential_drive(from, cmd_in, middle);
    if (check_pose(params, map, model, pose) == CollisionCheck::Free) {
      free_time = middle;
    } else {
      blocked_time = middle;
    }
  }
  return free_time;
}

/// Smallest clearance outside the inscribed circle along the braking sweep of this command.
template <map::CellMap Map, class Model>
  requires ClearanceModel<Model, typename Map::value_type>
double swept_clearance(
  const VelocityLimiterParams & params, double clearance_radius, const Map & map,
  const Model & model, const Pose2D & robot, const Twist2D & cmd_in)
{
  const map::MapGeometry & geometry = map.geometry();
  const double speed = std::abs(cmd_in.linear.x());
  const double braking_distance = speed * speed / (2.0 * params.max_deceleration);
  const double direction = (cmd_in.linear.x() < 0.0) ? -1.0 : 1.0;
  const Eigen::Vector2d heading{std::cos(robot.yaw), std::sin(robot.yaw)};
  const int samples =
    static_cast<int>(std::ceil(braking_distance / geometry.resolution()));

  double clearance = std::numeric_limits<double>::infinity();
  for (int sample = 0; sample <= samples; ++sample) {
    const double arc =
      std::min(static_cast<double>(sample) * geometry.resolution(), braking_distance);
    const std::optional<map::MapIndex> index =
      geometry.world_to_map(robot.position + direction * arc * heading);
    // Leaving the map ends the sweep; truncation is reported through predicted_poses instead.
    if (!index.has_value()) {
      break;
    }
    clearance = std::min(clearance, model.clearance(map(index->x, index->y)) - clearance_radius);
  }
  return clearance;
}

}  // namespace detail

/// Immutable command limiter; limit() is const, deterministic and holds no state between calls.
class VelocityLimiter
{
public:
  struct Result
  {
    Twist2D command{};
    bool has_collision{false};
    /// Arc length to the last free point, bisected inside the colliding step; +inf when free.
    double collision_distance{std::numeric_limits<double>::infinity()};
    /// Current pose first, the colliding pose last; shorter than prediction_steps + 1 if truncated.
    std::vector<Pose2D> predicted_poses{};
    /// Prediction horizon [s] this command was rolled out over; see horizon().
    double horizon{0.0};
    /// Seconds the requested command would still take to reach the margin; +inf when free or at rest.
    double time_to_collision{std::numeric_limits<double>::infinity()};
    /// Swept clearance [m] outside the inscribed circle; nullopt when the map carries no distance.
    std::optional<double> clearance{};
    /// Factor the proximity ramp applied to the command; 1.0 when nothing slowed it down.
    double proximity_scale{1.0};
  };

  /// nullopt when the footprint is degenerate, non-convex, excludes the origin, or a value is bad.
  static std::optional<VelocityLimiter> create(const VelocityLimiterParams & params);

  /// Predicts ahead and caps the command magnitude; throws std::invalid_argument for an unusable map.
  template <map::CellMap Map, class Model>
    requires TraversabilityModel<Model, typename Map::value_type> &&
             ObstacleModel<Model, typename Map::value_type>
  Result limit(
    const Map & map, const Model & model, const Pose2D & robot, const Twist2D & cmd_in) const;

  /// The stored parameters; the footprint may differ in vertex order from the one passed in.
  const VelocityLimiterParams & params() const noexcept { return params_; }

  /// Counter-clockwise footprint in the base frame; transform() it for visualization.
  const Polygon2D & footprint() const noexcept { return params_.footprint; }

  /// Radius [m] of the circle inside the footprint; what the proximity ramp measures clearance from.
  double clearance_radius() const noexcept { return clearance_radius_; }

  /// Horizon [s] derived from physics: the latency budget plus the time this command needs to stop.
  double horizon(const Twist2D & cmd) const noexcept
  {
    return params_.reaction_time + std::abs(cmd.linear.x()) / params_.max_deceleration;
  }

  double prediction_dt(const Twist2D & cmd) const noexcept
  {
    return horizon(cmd) / static_cast<double>(params_.prediction_steps);
  }

private:
  VelocityLimiter(const VelocityLimiterParams & params, double clearance_radius)
  : params_(params), clearance_radius_(clearance_radius)
  {
  }

  VelocityLimiterParams params_{};
  double clearance_radius_{0.0};
};

template <map::CellMap Map, class Model>
  requires TraversabilityModel<Model, typename Map::value_type> &&
           ObstacleModel<Model, typename Map::value_type>
VelocityLimiter::Result VelocityLimiter::limit(
  const Map & map, const Model & model, const Pose2D & robot, const Twist2D & cmd_in) const
{
  if (map.geometry().resolution() <= 0.0 || map.geometry().cell_count() == 0) {
    throw std::invalid_argument("VelocityLimiter requires a usable map");
  }

  Result result;
  result.predicted_poses.reserve(static_cast<std::size_t>(params_.prediction_steps) + 1);
  result.predicted_poses.push_back(robot);
  result.horizon = horizon(cmd_in);

  const double dt = prediction_dt(cmd_in);
  const double speed = std::abs(cmd_in.linear.x());
  const double step_distance = speed * dt;
  Pose2D pose = robot;
  double travelled = 0.0;
  for (int step = 0; step < params_.prediction_steps; ++step) {
    const Pose2D next = integrate_differential_drive(pose, cmd_in, dt);
    const CollisionCheck check = detail::check_pose(params_, map, model, next);
    // Leaving the map truncates the prediction instead of limiting; see docs/collision-design.md.
    if (check == CollisionCheck::OutsideMap) {
      break;
    }
    result.predicted_poses.push_back(next);
    if (check == CollisionCheck::Collision) {
      result.has_collision = true;
      result.collision_distance =
        travelled + speed * detail::last_free_time(params_, map, model, pose, cmd_in, dt);
      break;
    }
    travelled += step_distance;
    pose = next;
  }

  // Only a distance-valued map carries a clearance, so the cost-map path keeps the law it had.
  if constexpr (ClearanceModel<Model, typename Map::value_type>) {
    result.clearance =
      detail::swept_clearance(params_, clearance_radius_, map, model, robot, cmd_in);
    result.proximity_scale = detail::proximity_scale(params_, *result.clearance);
  }

  result.time_to_collision =
    detail::time_to_collision(params_, cmd_in, result.collision_distance);
  result.command = detail::limit_command(
    params_, cmd_in, result.has_collision, result.collision_distance, result.proximity_scale);
  return result;
}

}  // namespace eltanin::collision

#endif  // ELTANIN__COLLISION__VELOCITY_LIMITER_HPP_
