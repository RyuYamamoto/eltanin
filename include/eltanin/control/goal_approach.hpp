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

#ifndef ELTANIN__CONTROL__GOAL_APPROACH_HPP_
#define ELTANIN__CONTROL__GOAL_APPROACH_HPP_

#include <eltanin/core/path.hpp>
#include <eltanin/core/types.hpp>

#include <cstddef>
#include <limits>
#include <optional>

namespace eltanin::control
{

struct GoalApproachParams
{
  /// Position error that counts as arrival [m].
  double xy_goal_tolerance{0.10};
  /// Heading error that counts as arrival [rad].
  double yaw_goal_tolerance{0.10};
  /// Remaining arc length at which the deceleration law starts to apply [m].
  double approach_distance{0.5};
  /// Deceleration used by the goal-approach law; unrelated to obstacle braking [m/s^2].
  double approach_decel{0.5};
  /// Progress check on the final in-place alignment [s].
  double yaw_align_timeout{5.0};
  /// Symmetric bound on the alignment angular velocity [rad/s].
  double max_angular_vel{1.0};
};

namespace detail
{

/// Scales the whole command so the curvature is preserved; `limit` may be +inf (identity).
Twist2D apply_linear_limit(const Twist2D & cmd_in, double limit);

/// Nearest pose at or after `from`; a path that doubles back cannot pull the progress backwards.
std::size_t nearest_index_from(
  const Path & path, const Eigen::Vector2d & position, std::size_t from);

}  // namespace detail

/// Goal-approach deceleration and final yaw alignment; the caller composes it with PurePursuit.
class GoalApproach
{
public:
  /// Approaching decelerates; Aligning turns in place; the last two latch until reset().
  enum class State { Inactive, Approaching, Aligning, Reached, AlignmentTimeout };

  struct Result
  {
    /// Meaningful only when state is Aligning; zero otherwise.
    Twist2D command{};
    /// Upper bound to compose with apply_linear_limit(); +inf when there is nothing to limit.
    double linear_vel_limit{std::numeric_limits<double>::infinity()};
    State state{State::Inactive};
    /// Greater of the remaining path arc and the straight-line goal distance [m].
    /// +inf for an empty path.
    double remaining_arc{std::numeric_limits<double>::infinity()};
    /// Distance from the robot to the last path pose [m]; +inf for an empty path.
    double position_error{std::numeric_limits<double>::infinity()};
    /// Signed shortest rotation to the goal yaw [rad]; 0 for an empty path.
    double yaw_error{0.0};
    /// dt accumulated while Aligning [s]; 0 in every other non-latched state.
    double align_elapsed{0.0};
  };

  /// nullopt when a parameter is non-finite, out of range, or approach_distance < xy_goal_tolerance.
  static std::optional<GoalApproach> create(const GoalApproachParams & params);

  /// Reads `path` only, including the goal yaw from the last pose (the exception to C-4).
  /// Throws std::invalid_argument when the robot pose or dt is invalid.
  Result compute(const Pose2D & robot, const Path & path, double dt);

  /// Clears the terminal latch and the alignment timer; call it when a new path is handed in.
  void reset() noexcept;

  const GoalApproachParams & params() const noexcept { return params_; }

private:
  explicit GoalApproach(const GoalApproachParams & params) : params_(params) {}

  GoalApproachParams params_{};
  /// Set once Reached or AlignmentTimeout is entered; nothing but reset() clears it.
  std::optional<State> latched_{};
  double align_elapsed_{0.0};
  /// Search progress on the current path; reset() is required when the path changes.
  std::size_t progress_{0};
};

}  // namespace eltanin::control

#endif  // ELTANIN__CONTROL__GOAL_APPROACH_HPP_
