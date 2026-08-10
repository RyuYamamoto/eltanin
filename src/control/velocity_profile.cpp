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

#include <eltanin/control/velocity_profile.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace eltanin::control
{

namespace
{

constexpr double NO_LIMIT = std::numeric_limits<double>::infinity();

}  // namespace

std::optional<VelocityProfile> VelocityProfile::create(const VelocityProfileParams & params)
{
  // Checked first: NaN makes every range comparison below false.
  const bool all_finite =
    std::isfinite(params.max_linear_vel) && std::isfinite(params.max_angular_vel) &&
    std::isfinite(params.max_lateral_accel) && std::isfinite(params.max_decel) &&
    std::isfinite(params.curvature_window) && std::isfinite(params.min_speed) &&
    std::isfinite(params.terminal_speed);
  if (!all_finite) {
    return std::nullopt;
  }
  if (params.max_linear_vel <= 0.0 || params.max_angular_vel <= 0.0) {
    return std::nullopt;
  }
  if (params.max_lateral_accel <= 0.0 || params.max_decel <= 0.0) {
    return std::nullopt;
  }
  if (params.curvature_window < 0.0) {
    return std::nullopt;
  }
  if (params.min_speed < 0.0 || params.min_speed > params.max_linear_vel) {
    return std::nullopt;
  }
  if (params.terminal_speed < 0.0 || params.terminal_speed > params.max_linear_vel) {
    return std::nullopt;
  }
  return VelocityProfile(params);
}

void VelocityProfile::build(const Path & path)
{
  clear();
  if (path.empty()) {
    return;
  }

  arc_lengths_ = cumulative_arc_length(path);
  const std::vector<double> curvature = path_curvature(path, params_.curvature_window);

  limits_.resize(path.size());
  for (std::size_t i = 0; i < path.size(); ++i) {
    // A straight pose divides by zero on purpose: both bounds become +inf and min() drops them.
    const double magnitude = std::abs(curvature[i]);
    const double angular_bound = params_.max_angular_vel / magnitude;
    const double lateral_bound = std::sqrt(params_.max_lateral_accel / magnitude);
    limits_[i] = std::max(
      params_.min_speed,
      std::min({params_.max_linear_vel, angular_bound, lateral_bound}));
  }

  // A cusp is a full stop, so neither pass may carry speed across one.
  limits_.back() = std::min(limits_.back(), params_.terminal_speed);
  for (std::size_t i = path.size() - 1; i > 0; --i) {
    if (path.is_cusp(i)) {
      limits_[i] = 0.0;
    }
    const double span = arc_lengths_[i] - arc_lengths_[i - 1];
    const double reachable =
      std::sqrt(limits_[i] * limits_[i] + 2.0 * params_.max_decel * span);
    limits_[i - 1] = std::min(limits_[i - 1], reachable);
  }

  // The forward pass starts at cusps and nowhere else, so a path without one keeps its old profile.
  bool leaving_a_cusp = false;
  for (std::size_t i = 1; i < path.size(); ++i) {
    leaving_a_cusp = leaving_a_cusp || path.is_cusp(i - 1);
    if (!leaving_a_cusp) {
      continue;
    }
    const double span = arc_lengths_[i] - arc_lengths_[i - 1];
    const double reachable =
      std::sqrt(limits_[i - 1] * limits_[i - 1] + 2.0 * params_.max_decel * span);
    limits_[i] = std::min(limits_[i], reachable);
  }
}

void VelocityProfile::clear() noexcept
{
  arc_lengths_.clear();
  limits_.clear();
}

double VelocityProfile::at_index(std::size_t index) const noexcept
{
  if (limits_.empty()) {
    return NO_LIMIT;
  }
  return limits_[std::min(index, limits_.size() - 1)];
}

double VelocityProfile::at_arc(double arc_length) const noexcept
{
  if (limits_.empty()) {
    return NO_LIMIT;
  }
  if (!(arc_length > arc_lengths_.front())) {
    return limits_.front();
  }
  if (arc_length >= arc_lengths_.back()) {
    return limits_.back();
  }

  const auto upper =
    std::lower_bound(arc_lengths_.begin(), arc_lengths_.end(), arc_length);
  const std::size_t high = static_cast<std::size_t>(upper - arc_lengths_.begin());
  const std::size_t low = high - 1;
  const double span = arc_lengths_[high] - arc_lengths_[low];
  if (span <= 0.0) {
    return limits_[high];
  }
  const double ratio = (arc_length - arc_lengths_[low]) / span;
  return limits_[low] + ratio * (limits_[high] - limits_[low]);
}

}  // namespace eltanin::control
