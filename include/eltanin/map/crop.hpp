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

#ifndef ELTANIN__MAP__CROP_HPP_
#define ELTANIN__MAP__CROP_HPP_

#include <eltanin/core/types.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/map/map_geometry.hpp>

#include <algorithm>
#include <optional>
#include <span>

namespace eltanin::map
{

/// Cells covering every in-map position, grown by margin_cells; nullopt when none are inside.
inline std::optional<CellRect> bounding_cells(
  const MapGeometry & geometry, std::span<const Eigen::Vector2d> positions,
  int margin_cells) noexcept
{
  if (margin_cells < 0 || geometry.cell_count() == 0) {
    return std::nullopt;
  }
  bool seen = false;
  int min_x = geometry.size_x() - 1;
  int min_y = geometry.size_y() - 1;
  int max_x = 0;
  int max_y = 0;
  for (const Eigen::Vector2d & position : positions) {
    const std::optional<MapIndex> index = geometry.world_to_map(position);
    if (!index.has_value()) {
      continue;
    }
    seen = true;
    min_x = std::min(min_x, index->x);
    max_x = std::max(max_x, index->x);
    min_y = std::min(min_y, index->y);
    max_y = std::max(max_y, index->y);
  }
  if (!seen) {
    return std::nullopt;
  }
  return CellRect{
    std::max(0, min_x - margin_cells), std::max(0, min_y - margin_cells),
    std::min(geometry.size_x() - 1, max_x + margin_cells),
    std::min(geometry.size_y() - 1, max_y + margin_cells)};
}

/// Copy of the rectangle, carrying the world origin of its lower-left cell.
template <class T>
GridMap<T> crop(const GridMap<T> & map, const CellRect & rect)
{
  const MapGeometry & geometry = map.geometry();
  const int size_x = rect.max_x - rect.min_x + 1;
  const int size_y = rect.max_y - rect.min_y + 1;
  const Eigen::Vector2d origin =
    geometry.origin() + Eigen::Vector2d{
                          static_cast<double>(rect.min_x) * geometry.resolution(),
                          static_cast<double>(rect.min_y) * geometry.resolution()};

  GridMap<T> cropped(MapGeometry(size_x, size_y, geometry.resolution(), origin));
  for (int my = 0; my < size_y; ++my) {
    for (int mx = 0; mx < size_x; ++mx) {
      cropped(mx, my) = map(rect.min_x + mx, rect.min_y + my);
    }
  }
  return cropped;
}

/// The sub-map a search only needs to see; world coordinates keep working through its origin.
template <class T>
std::optional<GridMap<T>> crop_around(
  const GridMap<T> & map, std::span<const Eigen::Vector2d> positions, int margin_cells)
{
  const std::optional<CellRect> rect = bounding_cells(map.geometry(), positions, margin_cells);
  if (!rect.has_value()) {
    return std::nullopt;
  }
  return crop(map, *rect);
}

}  // namespace eltanin::map

#endif  // ELTANIN__MAP__CROP_HPP_
