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

#ifndef ELTANIN__CORE__ANGLE_HPP_
#define ELTANIN__CORE__ANGLE_HPP_

#include <cmath>

namespace eltanin
{

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;

/// Normalize to (-pi, pi]. Correct in one call for any |angle|; NaN for non-finite input.
inline double normalize_angle(double angle)
{
  const double r = std::remainder(angle, kTwoPi);
  if (r <= -kPi) {
    return r + kTwoPi;
  }
  return r;
}

/// Normalize to [0, 2*pi). NaN for non-finite input.
inline double normalize_angle_positive(double angle)
{
  double r = std::fmod(angle, kTwoPi);
  if (r < 0.0) {
    r += kTwoPi;
  }
  if (r >= kTwoPi) {
    r = 0.0;
  }
  return r;
}

/// Shortest signed rotation from `from` to `to`, in (-pi, pi].
inline double shortest_angular_distance(double from, double to)
{
  return normalize_angle(to - from);
}

}  // namespace eltanin

#endif  // ELTANIN__CORE__ANGLE_HPP_
