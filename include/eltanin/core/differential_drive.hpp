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

#ifndef ELTANIN__CORE__DIFFERENTIAL_DRIVE_HPP_
#define ELTANIN__CORE__DIFFERENTIAL_DRIVE_HPP_

#include <eltanin/core/types.hpp>

namespace eltanin
{

/// Below this angular velocity [rad/s] the straight-line approximation replaces the arc integral.
inline constexpr double ANGULAR_VEL_EPSILON = 1e-6;

/// One differential-drive step; linear.y() is ignored. Throws std::invalid_argument on bad input.
Pose2D integrate_differential_drive(const Pose2D & pose, const Twist2D & twist, double dt);

}  // namespace eltanin

#endif  // ELTANIN__CORE__DIFFERENTIAL_DRIVE_HPP_
