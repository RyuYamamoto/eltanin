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

#ifndef ELTANIN__CONTROL__PATH_FOLLOWER_HPP_
#define ELTANIN__CONTROL__PATH_FOLLOWER_HPP_

#include <eltanin/control/velocity_profile.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/core/types.hpp>

#include <cstddef>
#include <optional>
#include <string_view>
#include <utility>

namespace eltanin::control
{

/// Every status but Tracking comes with a zero command, so the caller always has one to send.
enum class FollowStatus
{
  NoPath,            ///< the path is empty
  Tracking,          ///< the command steers the robot along the path
  GoalReached,       ///< the last pose is the nearest one and the robot is on it
  SolverFailed,      ///< the follower could not compute a command and is decelerating
  PathNotSupported   ///< this follower cannot execute this path and has not tried
};

constexpr const char * to_string(FollowStatus status) noexcept
{
  switch (status) {
    case FollowStatus::NoPath:
      return "no path";
    case FollowStatus::Tracking:
      return "tracking";
    case FollowStatus::GoalReached:
      return "goal reached";
    case FollowStatus::SolverFailed:
      return "solver failed";
    case FollowStatus::PathNotSupported:
      return "path not supported";
  }
  return "unknown";
}

/// Linear speed [m/s] below which the body counts as stopped and may take up the other direction.
inline constexpr double STOPPED_SPEED = 1e-6;

/// What the follower is told about the robot; an absent twist means "use your own last command".
struct FollowerState
{
  Pose2D pose{};
  std::optional<Twist2D> twist{};
};

struct FollowResult
{
  Twist2D command{};
  FollowStatus status{FollowStatus::NoPath};
};

/// Runtime-polymorphic path follower; the base owns input validation and the degenerate paths.
class PathFollower
{
public:
  virtual ~PathFollower() = default;

  /// Throws std::invalid_argument when the pose, the twist, or dt is not finite and usable.
  [[nodiscard]] FollowResult follow(const FollowerState & state, const Path & path, double dt);

  /// Clears path progress, the remembered command, and the cached profile; call it per new path.
  void reset() noexcept;

  /// nullopt when the profile is disabled or not built yet; the arc runs from the first pose.
  [[nodiscard]] std::optional<double> velocity_limit_at(double arc_length) const noexcept;

  /// Whether this follower can execute `path` at all; a follower that cannot is not asked to try.
  [[nodiscard]] virtual bool supports(const Path & path) const noexcept
  {
    (void)path;
    return true;
  }

protected:
  PathFollower() = default;

  /// A profile makes every derived follower obey the same curvature-derived speed bound.
  explicit PathFollower(std::optional<VelocityProfile> profile) : profile_(std::move(profile)) {}

  PathFollower(const PathFollower &) = default;
  PathFollower & operator=(const PathFollower &) = default;
  PathFollower(PathFollower &&) = default;
  PathFollower & operator=(PathFollower &&) = default;

  [[nodiscard]] virtual FollowResult follow_on_path(
    const FollowerState & state, const Path & path, double dt) = 0;

  virtual void reset_derived() noexcept = 0;

  /// The command returned last cycle; the substitute for an absent FollowerState::twist.
  [[nodiscard]] const Twist2D & last_command() const noexcept { return last_command_; }

  /// The measured twist when there is one, the command returned last cycle otherwise.
  [[nodiscard]] const Twist2D & twist_of(const FollowerState & state) const noexcept
  {
    return state.twist.has_value() ? *state.twist : last_command_;
  }

  /// +inf when the profile is disabled, so composing it with a command is the identity.
  [[nodiscard]] double limit_at_index(std::size_t index) const noexcept;

  /// Same bound read by arc length instead of by pose index.
  [[nodiscard]] double limit_at_arc(double arc_length) const noexcept;

  /// nullptr when the profile is disabled, for followers that read the whole bound at once.
  [[nodiscard]] const VelocityProfile * profile() const noexcept
  {
    return profile_.has_value() ? &*profile_ : nullptr;
  }

private:
  Twist2D last_command_{};
  std::optional<VelocityProfile> profile_{};
  /// Pose count the profile was built from; a mismatch means reset() was skipped for a new path.
  std::size_t profile_path_size_{0};
};

/// Which follower to build; the ROS side switches on the name exactly as it does for planners.
enum class FollowerType
{
  PurePursuit,
  Mpc
};

constexpr std::optional<FollowerType> to_follower_type(std::string_view name) noexcept
{
  if (name == "pure_pursuit") {
    return FollowerType::PurePursuit;
  }
  if (name == "mpc") {
    return FollowerType::Mpc;
  }
  return std::nullopt;
}

constexpr const char * name_of(FollowerType type) noexcept
{
  switch (type) {
    case FollowerType::PurePursuit:
      return "pure_pursuit";
    case FollowerType::Mpc:
      return "mpc";
  }
  return "unknown";
}

}  // namespace eltanin::control

#endif  // ELTANIN__CONTROL__PATH_FOLLOWER_HPP_
