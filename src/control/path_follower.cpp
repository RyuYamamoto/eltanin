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

#include <eltanin/control/path_follower.hpp>

#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace eltanin::control
{

FollowResult PathFollower::follow(const FollowerState & state, const Path & path, double dt)
{
  const bool valid_twist =
    !state.twist.has_value() ||
    (state.twist->linear.allFinite() && std::isfinite(state.twist->angular));
  if (
    !std::isfinite(dt) || dt <= 0.0 || !state.pose.position.allFinite() ||
    !std::isfinite(state.pose.yaw) || !valid_twist) {
    throw std::invalid_argument(
      "PathFollower requires a finite state and positive finite dt");
  }

  if (path.empty()) {
    reset();
    return FollowResult{Twist2D{}, FollowStatus::NoPath};
  }
  if (path.size() == 1) {
    reset();
    return FollowResult{Twist2D{}, FollowStatus::GoalReached};
  }

  if (profile_.has_value()) {
    if (!profile_->built()) {
      profile_->build(path);
      profile_path_size_ = path.size();
    }
    assert(profile_path_size_ == path.size() && "a new path needs reset() before follow()");
  }

  const FollowResult result = follow_on_path(state, path, dt);
  last_command_ = result.command;
  return result;
}

void PathFollower::reset() noexcept
{
  last_command_ = Twist2D{};
  if (profile_.has_value()) {
    profile_->clear();
  }
  profile_path_size_ = 0;
  reset_derived();
}

std::optional<double> PathFollower::velocity_limit_at(double arc_length) const noexcept
{
  if (!profile_.has_value() || !profile_->built()) {
    return std::nullopt;
  }
  return profile_->at_arc(arc_length);
}

double PathFollower::limit_at_index(std::size_t index) const noexcept
{
  return profile_.has_value() ? profile_->at_index(index)
                              : std::numeric_limits<double>::infinity();
}

double PathFollower::limit_at_arc(double arc_length) const noexcept
{
  return profile_.has_value() ? profile_->at_arc(arc_length)
                              : std::numeric_limits<double>::infinity();
}

}  // namespace eltanin::control
