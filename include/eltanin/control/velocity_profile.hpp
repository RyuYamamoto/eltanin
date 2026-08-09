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

#ifndef ELTANIN__CONTROL__VELOCITY_PROFILE_HPP_
#define ELTANIN__CONTROL__VELOCITY_PROFILE_HPP_

#include <eltanin/core/path.hpp>

#include <cstddef>
#include <optional>
#include <vector>

namespace eltanin::control
{

/// Body limits the curvature-derived speed bound has to respect; the path itself is not a parameter.
struct VelocityProfileParams
{
  /// Upper bound the profile never exceeds [m/s]; matches PurePursuitParams::desired_linear_vel.
  double max_linear_vel{0.5};
  /// Symmetric angular velocity bound the profile must stay feasible under [rad/s].
  double max_angular_vel{1.0};
  /// Lateral acceleration budget [m/s^2]; unmeasured on hardware, so the default is conservative.
  double max_lateral_accel{0.5};
  /// Deceleration used by the backward pass [m/s^2]; same value as GoalApproachParams.
  double max_decel{0.5};
  /// Chord arc length of the curvature estimator [m]; below v_max / w_max it measures sampling.
  double curvature_window{0.3};
  /// Floor applied to the curvature-derived bound only, so a corner can never mean "stop" [m/s].
  double min_linear_vel{0.05};
  /// Terminal speed of the backward pass [m/s]; 0 makes the profile stop on the last pose.
  double terminal_linear_vel{0.0};
};

/// Speed upper bound along one path, built from its curvature and the body limits.
class VelocityProfile
{
public:
  /// nullopt when a parameter is non-finite or outside its admissible range.
  static std::optional<VelocityProfile> create(const VelocityProfileParams & params);

  /// Rebuilds from `path` in two passes; call it when the path changes, not every cycle.
  void build(const Path & path);

  /// Drops the built bound, so at_index() and at_arc() report no limit again.
  void clear() noexcept;

  [[nodiscard]] bool built() const noexcept { return !limits_.empty(); }

  /// Upper bound at that pose [m/s]; +inf before the first build(), clamped past the last pose.
  [[nodiscard]] double at_index(std::size_t index) const noexcept;

  /// Same bound, linearly interpolated in arc length and clamped to both ends [m/s].
  [[nodiscard]] double at_arc(double arc_length) const noexcept;

  [[nodiscard]] const std::vector<double> & arc_lengths() const noexcept { return arc_lengths_; }

  [[nodiscard]] const std::vector<double> & limits() const noexcept { return limits_; }

  [[nodiscard]] const VelocityProfileParams & params() const noexcept { return params_; }

private:
  explicit VelocityProfile(const VelocityProfileParams & params) : params_(params) {}

  VelocityProfileParams params_{};
  std::vector<double> arc_lengths_;
  std::vector<double> limits_;
};

}  // namespace eltanin::control

#endif  // ELTANIN__CONTROL__VELOCITY_PROFILE_HPP_
