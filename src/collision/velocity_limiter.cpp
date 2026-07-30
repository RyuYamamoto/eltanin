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
  if (!std::isfinite(params.prediction_time) || params.prediction_time <= 0.0) {
    return std::nullopt;
  }
  if (!std::isfinite(params.collision_margin) || params.collision_margin < 0.0) {
    return std::nullopt;
  }
  if (!std::isfinite(params.max_deceleration) || params.max_deceleration <= 0.0) {
    return std::nullopt;
  }

  VelocityLimiterParams normalized = params;
  normalized.footprint = to_counter_clockwise(params.footprint);
  return VelocityLimiter(normalized);
}

namespace detail
{

Twist2D limit_command(
  const VelocityLimiterParams & params, const Twist2D & cmd_in, bool has_collision,
  double collision_distance)
{
  const double d_col = std::max(0.0, collision_distance - params.collision_margin);
  const double v_max = std::sqrt(2.0 * params.max_deceleration * d_col);
  const double v_in = cmd_in.linear.x();
  // clamp() caps the magnitude while keeping the sign, which std::min() fails to do when v_in < 0.
  const double v_out = std::clamp(v_in, -v_max, v_max);
  const double ratio = (std::abs(v_in) > MIN_LINEAR_VEL) ? v_out / v_in : 1.0;
  double w_out = cmd_in.angular * ratio;
  // A ratio cannot limit in-place rotation, so a predicted collision zeroes it outright.
  if (has_collision && std::abs(v_in) <= MIN_LINEAR_VEL) {
    w_out = 0.0;
  }
  return Twist2D{Eigen::Vector2d{v_out, 0.0}, w_out};
}

}  // namespace detail

}  // namespace eltanin::collision
