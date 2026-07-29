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

#include <cstddef>

namespace eltanin
{

bool contains(const Polygon2D & polygon, const Vec2 & point, double edge_tolerance)
{
  const std::vector<Vec2> & v = polygon.vertices();
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
  const std::vector<Vec2> & v = polygon.vertices();
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

Polygon2D transform(const Polygon2D & polygon, const Transform2D & tf)
{
  std::vector<Vec2> transformed;
  transformed.reserve(polygon.size());
  for (const Vec2 & vertex : polygon) {
    transformed.push_back(tf * vertex);
  }
  return Polygon2D(std::move(transformed));
}

Polygon2D transform(const Polygon2D & polygon, const Pose2D & pose)
{
  return transform(polygon, Transform2D::from_pose(pose));
}

}  // namespace eltanin
