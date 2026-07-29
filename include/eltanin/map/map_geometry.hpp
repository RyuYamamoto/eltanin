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

#ifndef ELTANIN__MAP__MAP_GEOMETRY_HPP_
#define ELTANIN__MAP__MAP_GEOMETRY_HPP_

#include <eltanin/core/types.hpp>

#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>

namespace eltanin::map
{

struct MapIndex
{
  int x{0};
  int y{0};
};

inline bool operator==(const MapIndex & lhs, const MapIndex & rhs) noexcept
{
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

inline bool operator!=(const MapIndex & lhs, const MapIndex & rhs) noexcept
{
  return !(lhs == rhs);
}

/// The single place in this library that converts between world and map coordinates.
class MapGeometry
{
public:
  MapGeometry() = default;

  MapGeometry(int size_x, int size_y, double resolution, const Vec2 & origin)
  : size_x_(size_x), size_y_(size_y), resolution_(resolution), origin_(origin)
  {
  }

  int size_x() const noexcept { return size_x_; }

  int size_y() const noexcept { return size_y_; }

  double resolution() const noexcept { return resolution_; }

  const Vec2 & origin() const noexcept { return origin_; }

  std::size_t cell_count() const noexcept
  {
    if (size_x_ <= 0 || size_y_ <= 0) {
      return 0;
    }
    return static_cast<std::size_t>(size_x_) * static_cast<std::size_t>(size_y_);
  }

  bool in_bounds(int mx, int my) const noexcept
  {
    return mx >= 0 && my >= 0 && mx < size_x_ && my < size_y_;
  }

  /// Row-major linear index. Precondition: in_bounds(mx, my).
  std::size_t index(int mx, int my) const noexcept
  {
    assert(in_bounds(mx, my));
    return static_cast<std::size_t>(mx) +
           static_cast<std::size_t>(size_x_) * static_cast<std::size_t>(my);
  }

  /// Center of the cell, not its corner. Precondition: in_bounds(mx, my).
  Vec2 map_to_world(int mx, int my) const noexcept
  {
    assert(in_bounds(mx, my));
    return map_to_world_unbounded(mx, my);
  }

  Vec2 map_to_world_unbounded(int mx, int my) const noexcept
  {
    return origin_ + Vec2{
                       (static_cast<double>(mx) + 0.5) * resolution_,
                       (static_cast<double>(my) + 0.5) * resolution_};
  }

  /// nullopt when the point lies outside the map.
  std::optional<MapIndex> world_to_map(const Vec2 & world) const noexcept
  {
    const MapIndex index = world_to_map_unbounded(world);
    if (!in_bounds(index.x, index.y)) {
      return std::nullopt;
    }
    return index;
  }

  /// Floor-based conversion without a bounds check; used to derive clamped region borders.
  MapIndex world_to_map_unbounded(const Vec2 & world) const noexcept
  {
    const double fx = std::floor((world.x() - origin_.x()) / resolution_);
    const double fy = std::floor((world.y() - origin_.y()) / resolution_);
    return MapIndex{to_int_saturating(fx), to_int_saturating(fy)};
  }

  bool operator==(const MapGeometry & rhs) const noexcept
  {
    return size_x_ == rhs.size_x_ && size_y_ == rhs.size_y_ && resolution_ == rhs.resolution_ &&
           origin_.x() == rhs.origin_.x() && origin_.y() == rhs.origin_.y();
  }

  bool operator!=(const MapGeometry & rhs) const noexcept { return !(*this == rhs); }

private:
  /// Guards against undefined behaviour when casting an out-of-int-range or NaN double.
  static int to_int_saturating(double value) noexcept
  {
    constexpr double lowest = static_cast<double>(std::numeric_limits<int>::min());
    constexpr double highest = static_cast<double>(std::numeric_limits<int>::max());
    if (!(value >= lowest)) {
      return std::numeric_limits<int>::min();
    }
    if (value > highest) {
      return std::numeric_limits<int>::max();
    }
    return static_cast<int>(value);
  }

  int size_x_{0};
  int size_y_{0};
  double resolution_{0.0};
  Vec2 origin_{Vec2::Zero()};
};

}  // namespace eltanin::map

#endif  // ELTANIN__MAP__MAP_GEOMETRY_HPP_
