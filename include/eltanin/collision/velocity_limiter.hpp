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
};

namespace detail
{

/// Linear velocities below this magnitude [m/s] carry no ratio.
inline constexpr double MIN_LINEAR_VEL = 1e-9;

/// Bisections inside the colliding step; fixed, because the resolution of a stop is not a setting.
inline constexpr int COLLISION_REFINEMENT_STEPS = 4;

/// Braking-distance law applied to the magnitude of the command; the input sign is preserved.
Twist2D limit_command(
  const VelocityLimiterParams & params, const Twist2D & cmd_in, bool has_collision,
  double collision_distance);

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
  explicit VelocityLimiter(const VelocityLimiterParams & params) : params_(params) {}

  VelocityLimiterParams params_{};
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

  result.command =
    detail::limit_command(params_, cmd_in, result.has_collision, result.collision_distance);
  return result;
}

}  // namespace eltanin::collision

#endif  // ELTANIN__COLLISION__VELOCITY_LIMITER_HPP_
