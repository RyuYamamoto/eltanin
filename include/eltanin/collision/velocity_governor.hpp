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

#ifndef ELTANIN__COLLISION__VELOCITY_GOVERNOR_HPP_
#define ELTANIN__COLLISION__VELOCITY_GOVERNOR_HPP_

#include <eltanin/collision/velocity_limiter.hpp>

#include <algorithm>
#include <optional>

namespace eltanin::collision
{

struct VelocityGovernorParams
{
  VelocityLimiterParams limiter{};
  /// Time [s] the proximity scale takes to climb back to 1.0; dropping is always immediate.
  double release_time{0.5};
};

/// Geometry and law live in VelocityLimiter, time lives here; a node holds the governor.
class VelocityGovernor
{
public:
  /// nullopt when the limiter parameters are unusable or release_time is not finite and positive.
  static std::optional<VelocityGovernor> create(const VelocityGovernorParams & params);

  /// dt <= 0 holds the current, stricter scale; only the release side ever consumes time.
  template <map::CellMap Map, class Model>
    requires TraversabilityModel<Model, typename Map::value_type> &&
             ObstacleModel<Model, typename Map::value_type>
  VelocityLimiter::Result update(
    const Map & map, const Model & model, const Pose2D & robot, const Twist2D & cmd_in, double dt);

  /// Forgets the held scale; safe at any time because the drop side is immediate.
  void reset() noexcept { held_scale_ = 1.0; }

  const VelocityLimiter & limiter() const noexcept { return limiter_; }

  const VelocityGovernorParams & params() const noexcept { return params_; }

  double held_scale() const noexcept { return held_scale_; }

private:
  VelocityGovernor(const VelocityGovernorParams & params, const VelocityLimiter & limiter)
  : params_(params), limiter_(limiter)
  {
  }

  VelocityGovernorParams params_{};
  VelocityLimiter limiter_;
  double held_scale_{1.0};
};

template <map::CellMap Map, class Model>
  requires TraversabilityModel<Model, typename Map::value_type> &&
           ObstacleModel<Model, typename Map::value_type>
VelocityLimiter::Result VelocityGovernor::update(
  const Map & map, const Model & model, const Pose2D & robot, const Twist2D & cmd_in, double dt)
{
  VelocityLimiter::Result result = limiter_.limit(map, model, robot, cmd_in);
  const double released = held_scale_ + std::max(0.0, dt) / params_.release_time;
  held_scale_ = std::min(result.proximity_scale, released);
  result.proximity_scale = held_scale_;
  result.command = detail::limit_command(
    limiter_.params(), cmd_in, result.has_collision, result.collision_distance, held_scale_);
  return result;
}

}  // namespace eltanin::collision

#endif  // ELTANIN__COLLISION__VELOCITY_GOVERNOR_HPP_
