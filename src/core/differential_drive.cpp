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

#include <eltanin/core/differential_drive.hpp>

#include <cassert>
#include <cmath>

namespace eltanin
{

Pose2D integrate_differential_drive(const Pose2D & pose, const Twist2D & twist, double dt)
{
  assert(dt > 0.0);

  const double v = twist.linear.x();
  const double w = twist.angular;
  const double yaw = pose.yaw;
  Eigen::Vector2d position = pose.position;

  // Below the threshold v / w overflows towards infinity, so integrate along the heading instead.
  if (std::abs(w) < ANGULAR_VEL_EPSILON) {
    position.x() += v * std::cos(yaw) * dt;
    position.y() += v * std::sin(yaw) * dt;
  } else {
    const double radius = v / w;
    position.x() += radius * (std::sin(yaw + w * dt) - std::sin(yaw));
    position.y() += radius * (std::cos(yaw) - std::cos(yaw + w * dt));
  }
  return Pose2D{position, normalize_angle(yaw + w * dt)};
}

}  // namespace eltanin
