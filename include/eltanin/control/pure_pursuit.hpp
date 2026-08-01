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

#ifndef ELTANIN__CONTROL__PURE_PURSUIT_HPP_
#define ELTANIN__CONTROL__PURE_PURSUIT_HPP_

#include <eltanin/core/path.hpp>
#include <eltanin/core/types.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <optional>

namespace eltanin::control
{

struct PurePursuitParams
{
  /// Forward velocity the first-order ramp converges to [m/s].
  double desired_linear_vel{0.5};
  /// Symmetric bound on the commanded angular velocity [rad/s].
  double max_angular_vel{1.0};
  /// Heading error below which the initial in-place alignment ends [rad].
  double yaw_tolerance{0.07};
  /// Velocity-proportional part of the lookahead distance [s].
  double lookahead_time{0.1};
  /// Constant part of the lookahead distance; also the curvature denominator floor [m].
  double min_lookahead_dist{0.3};
};

class PurePursuit
{
public:
  /// NoPath and GoalReached both come with a zero command, so the caller always has one to send.
  enum class Status { NoPath, Tracking, GoalReached };

  struct Result
  {
    Twist2D command{};
    Status status{Status::NoPath};
    /// Index of the lookahead pose in the path; 0 unless status is Tracking.
    std::size_t target_index{0};
    /// World position of the lookahead pose; zero unless status is Tracking.
    Eigen::Vector2d lookahead_point{Eigen::Vector2d::Zero()};
  };

  /// nullopt when a parameter is non-finite or outside its admissible range.
  static std::optional<PurePursuit> create(const PurePursuitParams & params);

  /// Reads `path` only; goal arrival is reported through Status, never by clearing the path.
  /// Throws std::invalid_argument when the robot pose or dt is invalid.
  Result compute(const Pose2D & robot, const Path & path, double dt);

  /// Clears path progress, the velocity ramp, and the alignment latch; call for every new path.
  void reset() noexcept;

  const PurePursuitParams & params() const noexcept { return params_; }

private:
  explicit PurePursuit(const PurePursuitParams & params) : params_(params) {}

  PurePursuitParams params_{};
  /// Search progress on the current path; reset() is required when the path changes.
  std::size_t nearest_index_{0};
  double linear_vel_{0.0};
  bool yaw_aligned_{false};
};

}  // namespace eltanin::control

#endif  // ELTANIN__CONTROL__PURE_PURSUIT_HPP_
