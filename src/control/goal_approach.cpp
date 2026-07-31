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

#include <eltanin/control/goal_approach.hpp>

#include <eltanin/core/angle.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <numbers>

namespace eltanin::control
{

namespace
{

/// Proportional gain of the final in-place alignment [1/s]; see docs/control-design.md 12.4.
constexpr double YAW_ALIGN_GAIN = 2.0;

/// Linear velocities below this magnitude [m/s] carry no ratio.
constexpr double MIN_LINEAR_VEL = 1e-9;

/// Twin of nearest_index() in pure_pursuit.cpp; merging them is left to E-2 / E-3 (S8).
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

/// Polyline length from `from` to the last pose; 0 when `from` is already the last one.
double remaining_arc_from(const Path & path, std::size_t from)
{
  double arc = 0.0;
  for (std::size_t i = from + 1; i < path.size(); ++i) {
    arc += (path[i].position - path[i - 1].position).norm();
  }
  return arc;
}

}  // namespace

namespace detail
{

Twist2D apply_linear_limit(const Twist2D & cmd_in, double limit)
{
  // NaN fails this too; +inf passes and makes the clamp an identity.
  assert(limit >= 0.0);

  const double v_in = cmd_in.linear.x();
  // clamp() caps the magnitude while keeping the sign, which std::min() fails to do when v_in < 0.
  const double v_out = std::clamp(v_in, -limit, limit);
  const double ratio = (std::abs(v_in) > MIN_LINEAR_VEL) ? v_out / v_in : 1.0;
  return Twist2D{Eigen::Vector2d{v_out, 0.0}, cmd_in.angular * ratio};
}

}  // namespace detail

std::optional<GoalApproach> GoalApproach::create(const GoalApproachParams & params)
{
  // Checked first: NaN makes every range comparison below false.
  const bool all_finite =
    std::isfinite(params.xy_goal_tolerance) && std::isfinite(params.yaw_goal_tolerance) &&
    std::isfinite(params.approach_distance) && std::isfinite(params.approach_decel) &&
    std::isfinite(params.yaw_align_timeout) && std::isfinite(params.max_angular_vel);
  if (!all_finite) {
    return std::nullopt;
  }
  if (params.xy_goal_tolerance <= 0.0 || params.approach_distance <= 0.0) {
    return std::nullopt;
  }
  if (params.approach_decel <= 0.0 || params.yaw_align_timeout <= 0.0) {
    return std::nullopt;
  }
  if (params.max_angular_vel <= 0.0) {
    return std::nullopt;
  }
  if (params.yaw_goal_tolerance <= 0.0 || params.yaw_goal_tolerance >= std::numbers::pi) {
    return std::nullopt;
  }
  // An approach band inside the arrival band could never be left; see docs/control-design.md 12.3.
  if (params.approach_distance < params.xy_goal_tolerance) {
    return std::nullopt;
  }
  return GoalApproach(params);
}

GoalApproach::Result GoalApproach::compute(const Pose2D & robot, const Path & path, double dt)
{
  assert(std::isfinite(dt) && dt > 0.0);
  assert(robot.position.allFinite() && std::isfinite(robot.yaw));

  Result result;
  if (!path.empty()) {
    const Pose2D & goal = path[path.size() - 1];
    result.remaining_arc = remaining_arc_from(path, nearest_index(path, robot.position));
    result.position_error = (goal.position - robot.position).norm();
    result.yaw_error = normalize_angle(goal.yaw - robot.yaw);
  }

  // The terminal latch outranks everything, including a path that went away.
  if (latched_.has_value()) {
    result.state = *latched_;
    result.linear_vel_limit = 0.0;
    result.align_elapsed = align_elapsed_;
    return result;
  }

  // An empty path lands here too: +inf never fits inside the band.
  if (result.remaining_arc > params_.approach_distance) {
    align_elapsed_ = 0.0;
    return result;
  }

  if (result.position_error > params_.xy_goal_tolerance) {
    align_elapsed_ = 0.0;
    result.state = State::Approaching;
    result.linear_vel_limit = std::sqrt(2.0 * params_.approach_decel * result.remaining_arc);
    return result;
  }

  result.linear_vel_limit = 0.0;
  if (std::abs(result.yaw_error) <= params_.yaw_goal_tolerance) {
    align_elapsed_ = 0.0;
    latched_ = State::Reached;
    result.state = State::Reached;
    return result;
  }

  align_elapsed_ += dt;
  result.align_elapsed = align_elapsed_;
  if (align_elapsed_ > params_.yaw_align_timeout) {
    latched_ = State::AlignmentTimeout;
    result.state = State::AlignmentTimeout;
    return result;
  }

  result.state = State::Aligning;
  result.command.angular = std::clamp(
    YAW_ALIGN_GAIN * result.yaw_error, -params_.max_angular_vel, params_.max_angular_vel);
  return result;
}

void GoalApproach::reset() noexcept
{
  latched_.reset();
  align_elapsed_ = 0.0;
}

}  // namespace eltanin::control
