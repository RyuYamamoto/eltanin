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
#include <numbers>

namespace eltanin
{

/// Normalize to (-pi, pi]. Correct in one call for any |angle|; NaN for non-finite input.
inline double normalize_angle(double angle)
{
  constexpr double two_pi = 2.0 * std::numbers::pi;
  const double r = std::remainder(angle, two_pi);
  if (r <= -std::numbers::pi) {
    return r + two_pi;
  }
  return r;
}

/// Normalize to [0, 2*pi). NaN for non-finite input.
inline double normalize_angle_positive(double angle)
{
  constexpr double two_pi = 2.0 * std::numbers::pi;
  double r = std::fmod(angle, two_pi);
  if (r < 0.0) {
    r += two_pi;
  }
  if (r >= two_pi) {
    r = 0.0;
  }
  return r;
}

/// Shortest signed rotation from `from` to `to`, in (-pi, pi].
inline double shortest_angular_distance(double from, double to)
{
  return normalize_angle(to - from);
}

/// A closed arc traced counter-clockwise from `from` to `to`; a full turn is not representable.
struct AngleRange
{
  double from{0.0};
  double to{0.0};
};

/// True when `angle` lies on the CCW arc from `from` to `to`, both ends included.
inline bool angle_in_range(double angle, double from, double to)
{
  return normalize_angle_positive(angle - from) <= normalize_angle_positive(to - from);
}

}  // namespace eltanin

#endif  // ELTANIN__CORE__ANGLE_HPP_
