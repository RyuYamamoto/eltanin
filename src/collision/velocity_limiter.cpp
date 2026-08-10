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

#include <eltanin/collision/velocity_limiter.hpp>

#include <eltanin/core/footprint.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace eltanin::collision
{

std::optional<VelocityLimiter> VelocityLimiter::create(const VelocityLimiterParams & params)
{
  for (const Eigen::Vector2d & vertex : params.footprint) {
    if (!vertex.allFinite()) {
      return std::nullopt;
    }
  }
  // inscribed_radius() rejects fewer than three vertices, a near-zero area and an outside origin.
  if (!inscribed_radius(params.footprint).has_value()) {
    return std::nullopt;
  }
  if (!is_convex(params.footprint)) {
    return std::nullopt;
  }
  if (params.prediction_steps < 1) {
    return std::nullopt;
  }
  if (!std::isfinite(params.reaction_time) || params.reaction_time <= 0.0) {
    return std::nullopt;
  }
  if (!std::isfinite(params.collision_margin) || params.collision_margin < 0.0) {
    return std::nullopt;
  }
  if (!std::isfinite(params.max_deceleration) || params.max_deceleration <= 0.0) {
    return std::nullopt;
  }
  if (!std::isfinite(params.stop_clearance) || params.stop_clearance < 0.0) {
    return std::nullopt;
  }
  if (
    !std::isfinite(params.slow_down_clearance) ||
    params.slow_down_clearance <= params.stop_clearance) {
    return std::nullopt;
  }
  // A zero floor would let the ramp stop the robot, which only the two longitudinal laws may do.
  if (
    !std::isfinite(params.min_proximity_scale) || params.min_proximity_scale <= 0.0 ||
    params.min_proximity_scale > 1.0) {
    return std::nullopt;
  }

  const std::optional<double> circumscribed = eltanin::circumscribed_radius(params.footprint);
  if (!circumscribed.has_value()) {
    return std::nullopt;
  }

  VelocityLimiterParams normalized = params;
  normalized.footprint = to_counter_clockwise(params.footprint);
  return VelocityLimiter(normalized, *circumscribed);
}

namespace detail
{

double time_to_collision(
  const VelocityLimiterParams & params, const Twist2D & cmd_in, double collision_distance)
{
  const double speed = std::abs(cmd_in.linear.x());
  if (speed <= MIN_LINEAR_VEL) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(0.0, collision_distance - params.collision_margin) / speed;
}

double proximity_scale(const VelocityLimiterParams & params, double clearance)
{
  if (!(clearance < params.slow_down_clearance)) {
    return 1.0;
  }
  if (clearance <= params.stop_clearance) {
    return params.min_proximity_scale;
  }
  const double travelled = (clearance - params.stop_clearance) /
                           (params.slow_down_clearance - params.stop_clearance);
  return params.min_proximity_scale + (1.0 - params.min_proximity_scale) * travelled;
}

Twist2D limit_command(
  const VelocityLimiterParams & params, const Twist2D & cmd_in, bool has_collision,
  double collision_distance, double proximity_scale)
{
  const double d_col = std::max(0.0, collision_distance - params.collision_margin);
  // Two caps: one so the robot can still brake, one so the latency budget cannot be spent closing.
  const double v_max = std::min(
    std::sqrt(2.0 * params.max_deceleration * d_col), d_col / params.reaction_time);
  const double v_in = cmd_in.linear.x();
  // clamp() caps the magnitude while keeping the sign, which std::min() fails to do when v_in < 0.
  const double v_out = std::clamp(v_in * proximity_scale, -v_max, v_max);
  const double ratio = (std::abs(v_in) > MIN_LINEAR_VEL) ? v_out / v_in : proximity_scale;
  double w_out = cmd_in.angular * ratio;
  // A ratio cannot limit in-place rotation, so a predicted collision zeroes it outright.
  if (has_collision && std::abs(v_in) <= MIN_LINEAR_VEL) {
    w_out = 0.0;
  }
  return Twist2D{Eigen::Vector2d{v_out, 0.0}, w_out};
}

}  // namespace detail

}  // namespace eltanin::collision
