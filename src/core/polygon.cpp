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

#include <eltanin/core/polygon.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>

namespace eltanin
{
namespace
{

/// Relative tolerance for the convexity cross product; an absolute one would be scale dependent.
constexpr double kConvexityRelativeTolerance = 1e-12;

}  // namespace

bool contains(const Polygon2D & polygon, const Eigen::Vector2d & point, double edge_tolerance)
{
  const std::vector<Eigen::Vector2d> & v = polygon.vertices();
  const std::size_t n = v.size();
  if (n < 3) {
    return false;
  }

  // Boundary is resolved first because a crossing-number test cannot decide edge points stably.
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    if (distance_to_segment(point, v[j], v[i]) <= edge_tolerance) {
      return true;
    }
  }

  bool inside = false;
  const double px = point.x();
  const double py = point.y();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    // Half-open comparison on y keeps vertices from being counted twice.
    if ((v[i].y() > py) != (v[j].y() > py)) {
      const double x_intersect =
        v[j].x() + (py - v[j].y()) * (v[i].x() - v[j].x()) / (v[i].y() - v[j].y());
      if (px < x_intersect) {
        inside = !inside;
      }
    }
  }
  return inside;
}

double signed_area(const Polygon2D & polygon)
{
  const std::vector<Eigen::Vector2d> & v = polygon.vertices();
  const std::size_t n = v.size();
  if (n < 3) {
    return 0.0;
  }
  double sum = 0.0;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    sum += v[j].x() * v[i].y() - v[i].x() * v[j].y();
  }
  return 0.5 * sum;
}

Winding winding(const Polygon2D & polygon, double area_tolerance)
{
  const double area = signed_area(polygon);
  if (polygon.size() < 3 || std::abs(area) <= area_tolerance) {
    return Winding::Degenerate;
  }
  return area > 0.0 ? Winding::CounterClockwise : Winding::Clockwise;
}

Polygon2D to_counter_clockwise(const Polygon2D & polygon)
{
  if (winding(polygon) != Winding::Clockwise) {
    return polygon;
  }
  std::vector<Eigen::Vector2d> reversed(polygon.begin(), polygon.end());
  std::reverse(reversed.begin(), reversed.end());
  return Polygon2D(std::move(reversed));
}

bool is_convex(const Polygon2D & polygon)
{
  const std::vector<Eigen::Vector2d> & v = polygon.vertices();
  const std::size_t n = v.size();
  if (n < 3) {
    return false;
  }

  int sign = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Vector2d e1 = v[(i + 1) % n] - v[i];
    const Eigen::Vector2d e2 = v[(i + 2) % n] - v[(i + 1) % n];
    const double cross = e1.x() * e2.y() - e1.y() * e2.x();
    // Collinear vertices carry no turn direction, so they are skipped rather than rejected.
    if (std::abs(cross) <= kConvexityRelativeTolerance * e1.norm() * e2.norm()) {
      continue;
    }
    const int current = cross > 0.0 ? 1 : -1;
    if (sign == 0) {
      sign = current;
    } else if (sign != current) {
      return false;
    }
  }
  return true;
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> bounding_box(const Polygon2D & polygon)
{
  assert(!polygon.empty());
  Eigen::Vector2d min = polygon[0];
  Eigen::Vector2d max = polygon[0];
  for (const Eigen::Vector2d & vertex : polygon) {
    min = min.cwiseMin(vertex);
    max = max.cwiseMax(vertex);
  }
  return {min, max};
}

Polygon2D transform(const Polygon2D & polygon, const Transform2D & tf)
{
  std::vector<Eigen::Vector2d> transformed;
  transformed.reserve(polygon.size());
  for (const Eigen::Vector2d & vertex : polygon) {
    transformed.push_back(tf * vertex);
  }
  return Polygon2D(std::move(transformed));
}

Polygon2D transform(const Polygon2D & polygon, const Pose2D & pose)
{
  return transform(polygon, Transform2D::from_pose(pose));
}

}  // namespace eltanin
