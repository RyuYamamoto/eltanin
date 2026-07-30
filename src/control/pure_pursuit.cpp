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

#include <algorithm>
#include <cassert>
#include <cmath>
#include <numbers>

namespace eltanin::control
{

namespace
{

/// First-order gain of the forward velocity ramp [1/s].
constexpr double LINEAR_VEL_GAIN = 1.0;

/// Fraction of max_angular_vel used while turning in place to face the path.
constexpr double ALIGNMENT_ANGULAR_VEL_RATIO = 0.5;

/// Below this the bearing to the target pose is undefined, so the heading error is taken as 0 [m].
constexpr double MIN_TARGET_DISTANCE = 1e-9;

/// Global nearest pose; ties resolve to the smallest index so the result is deterministic.
std::size_t nearest_index(const Path & path, const Eigen::Vector2d & position)
{
  std::size_t nearest = 0;
  double min_distance = (path[0].position - position).squaredNorm();
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double distance = (path[i].position - position).squaredNorm();
    if (distance < min_distance) {
      min_distance = distance;
      nearest = i;
    }
  }
  return nearest;
}

/// First pose at or beyond `lookahead` when walking forward from `from`; the last pose otherwise.
std::size_t select_target_index(
  const Path & path, const Eigen::Vector2d & position, std::size_t from, double lookahead)
{
  for (std::size_t i = from; i < path.size(); ++i) {
    if ((path[i].position - position).norm() >= lookahead) {
      return i;
    }
  }
  return path.size() - 1;
}

}  // namespace

std::optional<PurePursuit> PurePursuit::create(const PurePursuitParams & params)
{
  // Checked first: NaN makes every range comparison below false.
  const bool all_finite = std::isfinite(params.desired_linear_vel) &&
                          std::isfinite(params.max_angular_vel) &&
                          std::isfinite(params.yaw_tolerance) &&
                          std::isfinite(params.lookahead_time) &&
                          std::isfinite(params.min_lookahead_dist);
  if (!all_finite) {
    return std::nullopt;
  }
  if (params.desired_linear_vel <= 0.0 || params.max_angular_vel <= 0.0) {
    return std::nullopt;
  }
  if (params.yaw_tolerance <= 0.0 || params.yaw_tolerance >= std::numbers::pi) {
    return std::nullopt;
  }
  if (params.lookahead_time < 0.0 || params.min_lookahead_dist <= 0.0) {
    return std::nullopt;
  }
  return PurePursuit(params);
}

PurePursuit::Result PurePursuit::compute(const Pose2D & robot, const Path & path, double dt)
{
  assert(std::isfinite(dt) && dt > 0.0);
  assert(robot.position.allFinite() && std::isfinite(robot.yaw));

  if (path.empty()) {
    reset();
    return Result{Twist2D{}, Status::NoPath, 0, Eigen::Vector2d::Zero()};
  }

  const std::size_t nearest = nearest_index(path, robot.position);
  if (nearest + 1 == path.size()) {
    reset();
    return Result{Twist2D{}, Status::GoalReached, 0, Eigen::Vector2d::Zero()};
  }

  const double lookahead = params_.lookahead_time * linear_vel_ + params_.min_lookahead_dist;
  const std::size_t target = select_target_index(path, robot.position, nearest, lookahead);
  const Eigen::Vector2d delta = path[target].position - robot.position;
  const double distance = delta.norm();
  const double alpha = (distance < MIN_TARGET_DISTANCE)
                         ? 0.0
                         : normalize_angle(std::atan2(delta.y(), delta.x()) - robot.yaw);

  if (!yaw_aligned_) {
    if (std::abs(alpha) < params_.yaw_tolerance) {
      // Latched inside the tolerance; drive in the same cycle instead of turning once more.
      yaw_aligned_ = true;
    } else {
      const double sign = (alpha > 0.0) ? 1.0 : -1.0;
      const Twist2D command{
        Eigen::Vector2d::Zero(),
        sign * params_.max_angular_vel * ALIGNMENT_ANGULAR_VEL_RATIO};
      return Result{command, Status::Tracking, target, path[target].position};
    }
  }

  linear_vel_ += LINEAR_VEL_GAIN * (params_.desired_linear_vel - linear_vel_) * dt;
  linear_vel_ = std::clamp(linear_vel_, 0.0, params_.desired_linear_vel);

  const double curvature_radius = std::max(distance, params_.min_lookahead_dist);
  const double angular_vel = std::clamp(
    2.0 * linear_vel_ * std::sin(alpha) / curvature_radius, -params_.max_angular_vel,
    params_.max_angular_vel);

  const Twist2D command{Eigen::Vector2d{linear_vel_, 0.0}, angular_vel};
  return Result{command, Status::Tracking, target, path[target].position};
}

void PurePursuit::reset() noexcept
{
  linear_vel_ = 0.0;
  yaw_aligned_ = false;
}

}  // namespace eltanin::control
