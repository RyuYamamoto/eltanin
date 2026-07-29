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

#ifndef ELTANIN__CORE__POLYGON_HPP_
#define ELTANIN__CORE__POLYGON_HPP_

#include <eltanin/core/geometry.hpp>
#include <eltanin/core/types.hpp>

#include <cstddef>
#include <initializer_list>
#include <utility>
#include <vector>

namespace eltanin
{

/// Implicitly closed polygon. Self-intersecting polygons, holes and offsetting are unsupported.
class Polygon2D
{
public:
  Polygon2D() = default;

  explicit Polygon2D(std::vector<Vec2> vertices) : vertices_(std::move(vertices)) {}

  Polygon2D(std::initializer_list<Vec2> vertices) : vertices_(vertices) {}

  const std::vector<Vec2> & vertices() const noexcept { return vertices_; }

  std::size_t size() const noexcept { return vertices_.size(); }

  bool empty() const noexcept { return vertices_.empty(); }

  const Vec2 & operator[](std::size_t i) const { return vertices_[i]; }

  auto begin() const noexcept { return vertices_.begin(); }

  auto end() const noexcept { return vertices_.end(); }

  void push_back(const Vec2 & vertex) { vertices_.push_back(vertex); }

  void clear() noexcept { vertices_.clear(); }

private:
  std::vector<Vec2> vertices_;
};

/// Crossing-number containment, valid for convex and concave polygons and independent of winding.
bool contains(const Polygon2D & polygon, const Vec2 & point, double edge_tolerance = 1e-9);

/// Shoelace signed area; positive for counter-clockwise winding.
double signed_area(const Polygon2D & polygon);

Polygon2D transform(const Polygon2D & polygon, const Transform2D & tf);

Polygon2D transform(const Polygon2D & polygon, const Pose2D & pose);

}  // namespace eltanin

#endif  // ELTANIN__CORE__POLYGON_HPP_
