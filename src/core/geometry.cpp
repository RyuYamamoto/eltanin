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

#include <eltanin/core/geometry.hpp>

#include <algorithm>

namespace eltanin
{

Eigen::Vector2d closest_point_on_segment(
  const Eigen::Vector2d & p, const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  const Eigen::Vector2d ab = b - a;
  const double length_squared = ab.squaredNorm();
  if (length_squared <= 0.0) {
    return a;
  }
  const double t = std::clamp((p - a).dot(ab) / length_squared, 0.0, 1.0);
  return a + t * ab;
}

double distance_to_segment(
  const Eigen::Vector2d & p, const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  return (p - closest_point_on_segment(p, a, b)).norm();
}

}  // namespace eltanin
