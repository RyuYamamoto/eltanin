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

#include <eltanin/control/path_follower.hpp>
#include <eltanin/control/velocity_profile.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/core/types.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <optional>
#include <utility>

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
  /// nullopt leaves the geometry alone; a value caps the command by the path curvature.
  std::optional<VelocityProfileParams> velocity_profile{};
};

class PurePursuit : public PathFollower
{
public:
  /// The pose the geometry aimed at last cycle; zeroed whenever the follower is not Tracking.
  struct Lookahead
  {
    std::size_t target_index{0};
    Eigen::Vector2d point{Eigen::Vector2d::Zero()};
  };

  /// nullopt when a parameter is non-finite or outside its admissible range.
  static std::optional<PurePursuit> create(const PurePursuitParams & params);

  /// Valid until the next follow(); FollowerState::twist is not read at all.
  const Lookahead & lookahead() const noexcept { return lookahead_; }

  const PurePursuitParams & params() const noexcept { return params_; }

protected:
  [[nodiscard]] FollowResult follow_on_path(
    const FollowerState & state, const Path & path, double dt) override;

  void reset_derived() noexcept override;

private:
  PurePursuit(const PurePursuitParams & params, std::optional<VelocityProfile> profile)
  : PathFollower(std::move(profile)), params_(params)
  {
  }

  /// The geometry alone, before the profile bound is composed with its command.
  [[nodiscard]] FollowResult pursue(const FollowerState & state, const Path & path, double dt);

  PurePursuitParams params_{};
  /// Search progress on the current path; reset() is required when the path changes.
  std::size_t nearest_index_{0};
  double linear_vel_{0.0};
  bool yaw_aligned_{false};
  Lookahead lookahead_{};
};

}  // namespace eltanin::control

#endif  // ELTANIN__CONTROL__PURE_PURSUIT_HPP_
