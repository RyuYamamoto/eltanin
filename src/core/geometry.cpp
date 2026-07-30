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
#include <cmath>

namespace eltanin
{

namespace
{

/// Relative bound: the raw cross product scales with the product of the segment lengths.
constexpr double SEGMENT_PARALLEL_EPS = 1e-12;

}  // namespace

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

std::optional<Eigen::Vector2d> segment_intersection(
  const Eigen::Vector2d & a1, const Eigen::Vector2d & a2, const Eigen::Vector2d & b1,
  const Eigen::Vector2d & b2)
{
  const Eigen::Vector2d r = a2 - a1;
  const Eigen::Vector2d s = b2 - b1;
  const double cross_rs = r.x() * s.y() - r.y() * s.x();
  // A degenerate segment gives cross_rs == 0 with a zero bound, so it takes this branch too.
  if (std::abs(cross_rs) <= SEGMENT_PARALLEL_EPS * r.norm() * s.norm()) {
    return std::nullopt;
  }
  const Eigen::Vector2d q = b1 - a1;
  const double t = (q.x() * s.y() - q.y() * s.x()) / cross_rs;
  const double u = (q.x() * r.y() - q.y() * r.x()) / cross_rs;
  if (t < 0.0 || t > 1.0 || u < 0.0 || u > 1.0) {
    return std::nullopt;
  }
  return a1 + t * r;
}

}  // namespace eltanin
