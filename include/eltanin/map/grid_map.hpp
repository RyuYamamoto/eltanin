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

#ifndef ELTANIN__MAP__GRID_MAP_HPP_
#define ELTANIN__MAP__GRID_MAP_HPP_

#include <eltanin/map/map_geometry.hpp>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <vector>

namespace eltanin::map
{

/// Owns its cells. Pass by reference; copying a 4000x4000 map moves 16 MB.
template <class T>
class GridMap
{
  static_assert(std::is_default_constructible_v<T>, "cell type must be default constructible");
  static_assert(std::is_copy_constructible_v<T>, "cell type must be copy constructible");

public:
  using value_type = T;

  GridMap() = default;

  explicit GridMap(const MapGeometry & geometry, const T & initial = T{})
  : geometry_(geometry), cells_(geometry.cell_count(), initial)
  {
  }

  const MapGeometry & geometry() const noexcept { return geometry_; }

  /// Moves the window without touching the cells; nothing is reallocated or reinitialized.
  void set_origin(const Eigen::Vector2d & origin) noexcept { geometry_.set_origin(origin); }

  int size_x() const noexcept { return geometry_.size_x(); }

  int size_y() const noexcept { return geometry_.size_y(); }

  std::size_t cell_count() const noexcept { return cells_.size(); }

  /// Precondition: in_bounds(mx, my). Checked by assert only, for hot loops.
  T & operator()(int mx, int my) noexcept { return cells_[geometry_.index(mx, my)]; }

  const T & operator()(int mx, int my) const noexcept { return cells_[geometry_.index(mx, my)]; }

  /// Precondition: index < cell_count(). Checked by assert only, for hot loops.
  T & operator[](std::size_t index) noexcept
  {
    assert(index < cells_.size());
    return cells_[index];
  }

  const T & operator[](std::size_t index) const noexcept
  {
    assert(index < cells_.size());
    return cells_[index];
  }

  std::optional<T> get(int mx, int my) const
  {
    if (!geometry_.in_bounds(mx, my)) {
      return std::nullopt;
    }
    return cells_[geometry_.index(mx, my)];
  }

  /// Returns false when out of bounds; the map is left untouched.
  bool set(int mx, int my, const T & value)
  {
    if (!geometry_.in_bounds(mx, my)) {
      return false;
    }
    cells_[geometry_.index(mx, my)] = value;
    return true;
  }

  void fill(const T & value) { std::fill(cells_.begin(), cells_.end(), value); }

  std::vector<T> & data() noexcept { return cells_; }

  const std::vector<T> & data() const noexcept { return cells_; }

private:
  MapGeometry geometry_;
  std::vector<T> cells_;
};

using Costmap = GridMap<std::uint8_t>;

/// Reserved for a future Euclidean distance transform; no producer exists yet.
using DistanceMap = GridMap<float>;

}  // namespace eltanin::map

#endif  // ELTANIN__MAP__GRID_MAP_HPP_
