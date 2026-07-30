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

#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace eltanin::collision
{

struct VelocityLimiterParams
{
  /// Robot footprint in the base frame; create() stores it normalized to counter-clockwise.
  Polygon2D footprint{
    Eigen::Vector2d{-0.3, 0.3}, Eigen::Vector2d{-0.3, -0.3}, Eigen::Vector2d{0.3, -0.3},
    Eigen::Vector2d{0.3, 0.3}};
  /// Number of prediction steps spread over prediction_time.
  int prediction_steps{10};
  /// Prediction horizon [s].
  double prediction_time{2.0};
  /// Distance kept in front of the predicted collision [m].
  double collision_margin{0.2};
  /// Deceleration used by the braking-distance law [m/s^2].
  double max_deceleration{0.5};
};

namespace detail
{

/// Linear velocities below this magnitude [m/s] carry no ratio.
inline constexpr double MIN_LINEAR_VEL = 1e-9;

/// Braking-distance law applied to the magnitude of the command; the input sign is preserved.
Twist2D limit_command(
  const VelocityLimiterParams & params, const Twist2D & cmd_in, bool has_collision,
  double collision_distance);

}  // namespace detail

/// Immutable command limiter; limit() is const, deterministic and holds no state between calls.
class VelocityLimiter
{
public:
  struct Result
  {
    Twist2D command{};
    bool has_collision{false};
    /// Arc length to the last collision-free predicted pose; +inf when there is no collision.
    double collision_distance{std::numeric_limits<double>::infinity()};
    /// Current pose first, the colliding pose last; shorter than prediction_steps + 1 if truncated.
    std::vector<Pose2D> predicted_poses{};
  };

  /// nullopt when the footprint is degenerate, non-convex, excludes the origin, or a value is bad.
  static std::optional<VelocityLimiter> create(const VelocityLimiterParams & params);

  /// Predicts prediction_steps ahead and caps the command magnitude. Preconditions: a usable map.
  template <map::CellMap Map, class Model>
    requires TraversabilityModel<Model, typename Map::value_type> &&
             ObstacleModel<Model, typename Map::value_type>
  Result limit(
    const Map & map, const Model & model, const Pose2D & robot, const Twist2D & cmd_in) const;

  /// The stored parameters; the footprint may differ in vertex order from the one passed in.
  const VelocityLimiterParams & params() const noexcept { return params_; }

  /// Counter-clockwise footprint in the base frame; transform() it for visualization.
  const Polygon2D & footprint() const noexcept { return params_.footprint; }

  double prediction_dt() const noexcept
  {
    return params_.prediction_time / static_cast<double>(params_.prediction_steps);
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
  assert(map.geometry().resolution() > 0.0);
  assert(map.geometry().cell_count() > 0);

  Result result;
  result.predicted_poses.reserve(static_cast<std::size_t>(params_.prediction_steps) + 1);
  result.predicted_poses.push_back(robot);

  const double dt = prediction_dt();
  const double step_distance = std::abs(cmd_in.linear.x()) * dt;
  Pose2D pose = robot;
  double travelled = 0.0;
  for (int step = 0; step < params_.prediction_steps; ++step) {
    pose = integrate_differential_drive(pose, cmd_in, dt);
    const CollisionCheck check = check_footprint(map, model, params_.footprint, pose);
    // Leaving the map truncates the prediction instead of limiting; see docs/safety-design.md.
    if (check == CollisionCheck::OutsideMap) {
      break;
    }
    result.predicted_poses.push_back(pose);
    if (check == CollisionCheck::Collision) {
      result.has_collision = true;
      result.collision_distance = travelled;
      break;
    }
    travelled += step_distance;
  }

  result.command =
    detail::limit_command(params_, cmd_in, result.has_collision, result.collision_distance);
  return result;
}

}  // namespace eltanin::collision

#endif  // ELTANIN__COLLISION__VELOCITY_LIMITER_HPP_
