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

#include <eltanin/planner/obstacle_field.hpp>

#include <eltanin/map/crop.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace eltanin::planner::detail
{

namespace
{

/// Stands in for infinity so the parabola intersections below never produce a NaN.
constexpr float UNREACHED = 1e18F;

/// Felzenszwalb and Huttenlocher's exact squared distance transform of one row or column.
void transform_line(
  std::span<const float> source, std::span<float> destination, std::vector<int> & hull,
  std::vector<float> & boundary)
{
  const auto count = static_cast<int>(source.size());
  const auto squared = [](int value) { return static_cast<float>(value) * static_cast<float>(value); };

  const auto intersect = [&](int q, int p) {
    return ((source[static_cast<std::size_t>(q)] + squared(q)) -
            (source[static_cast<std::size_t>(p)] + squared(p))) /
           static_cast<float>(2 * q - 2 * p);
  };

  int top = 0;
  hull[0] = 0;
  boundary[0] = -UNREACHED;
  boundary[1] = UNREACHED;
  for (int q = 1; q < count; ++q) {
    // Popping parabolas that the new one has covered is what keeps the whole pass linear.
    float intersection = intersect(q, hull[top]);
    while (intersection <= boundary[top]) {
      --top;
      intersection = intersect(q, hull[top]);
    }
    ++top;
    hull[top] = q;
    boundary[top] = intersection;
    boundary[top + 1] = UNREACHED;
  }

  int current = 0;
  for (int q = 0; q < count; ++q) {
    while (boundary[current + 1] < static_cast<float>(q)) {
      ++current;
    }
    const int p = hull[current];
    destination[static_cast<std::size_t>(q)] =
      squared(q - p) + source[static_cast<std::size_t>(p)];
  }
}

/// Square tile side; walking a column directly would miss the cache on every single cell.
constexpr int TILE = 32;

/// Writes the `rows` by `cols` row-major `source` into `destination` as `cols` by `rows`.
void transpose(std::span<const float> source, std::span<float> destination, int rows, int cols)
{
  for (int row_base = 0; row_base < rows; row_base += TILE) {
    for (int column_base = 0; column_base < cols; column_base += TILE) {
      const int row_end = std::min(row_base + TILE, rows);
      const int column_end = std::min(column_base + TILE, cols);
      for (int row = row_base; row < row_end; ++row) {
        for (int column = column_base; column < column_end; ++column) {
          destination[static_cast<std::size_t>(column) * static_cast<std::size_t>(rows) +
                      static_cast<std::size_t>(row)] =
            source[static_cast<std::size_t>(row) * static_cast<std::size_t>(cols) +
                   static_cast<std::size_t>(column)];
        }
      }
    }
  }
}

/// Runs the one-dimensional transform along every row of a row-major buffer, in place.
void transform_rows(std::span<float> values, int rows, int cols)
{
  std::vector<float> line(static_cast<std::size_t>(cols), 0.0F);
  std::vector<int> hull(static_cast<std::size_t>(cols) + 1, 0);
  std::vector<float> boundary(static_cast<std::size_t>(cols) + 2, 0.0F);
  for (int row = 0; row < rows; ++row) {
    const std::span<float> source =
      values.subspan(static_cast<std::size_t>(row) * static_cast<std::size_t>(cols),
                     static_cast<std::size_t>(cols));
    transform_line(source, std::span<float>{line}, hull, boundary);
    std::copy(line.begin(), line.end(), source.begin());
  }
}

/// Seeds one rectangle of the classified grid, transforms it, and converts it to metres.
std::vector<float> distance_in_rect(
  const TraversabilityView & grid, int min_x, int min_y, int size_x, int size_y, float cap)
{
  const map::MapGeometry & geometry = grid.geometry();
  const auto cells = static_cast<std::size_t>(size_x) * static_cast<std::size_t>(size_y);
  const auto at = [size_x](int x, int y) {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(size_x) +
           static_cast<std::size_t>(x);
  };

  std::vector<float> squared(cells, 0.0F);
  for (int y = 0; y < size_y; ++y) {
    for (int x = 0; x < size_x; ++x) {
      squared[at(x, y)] = grid.blocked(min_x + x, min_y + y) ? 0.0F : UNREACHED;
    }
  }

  transform_rows(squared, size_y, size_x);
  // Transposing costs two blocked passes and buys contiguous access for the second direction.
  std::vector<float> transposed(cells, 0.0F);
  transpose(squared, transposed, size_y, size_x);
  transform_rows(transposed, size_x, size_y);
  transpose(transposed, squared, size_x, size_y);

  const auto resolution = static_cast<float>(geometry.resolution());
  // The edge of a map is not a wall: it is usually just where a corridor was cropped out.
  const auto span = static_cast<float>(std::hypot(geometry.size_x(), geometry.size_y()));
  for (int y = 0; y < size_y; ++y) {
    for (int x = 0; x < size_x; ++x) {
      const float metres = std::min(std::sqrt(squared[at(x, y)]), span) * resolution;
      // A window only knows what is inside it, so anything past its reach reads as the reach.
      squared[at(x, y)] = cap > 0.0F ? std::min(metres, cap) : metres;
    }
  }
  return squared;
}

}  // namespace

void validate_clearance_cost(const ClearanceCost & cost)
{
  const bool valid = std::isfinite(cost.penalty) && cost.penalty >= 0.0 &&
                     std::isfinite(cost.distance) && cost.distance > 0.0;
  if (!valid) {
    throw std::invalid_argument("invalid ClearanceCost");
  }
}

std::vector<float> build_obstacle_distance(const TraversabilityView & grid)
{
  const map::MapGeometry & geometry = grid.geometry();
  return distance_in_rect(grid, 0, 0, geometry.size_x(), geometry.size_y(), 0.0F);
}

std::optional<ObstacleWindow> build_obstacle_window(
  const TraversabilityView & grid, std::span<const Eigen::Vector2d> positions, double reach)
{
  const map::MapGeometry & geometry = grid.geometry();
  if (!std::isfinite(reach) || reach <= 0.0) {
    return std::nullopt;
  }
  // One cell of slack so the falloff is still exact for a point sitting on the window edge.
  const int margin = static_cast<int>(std::ceil(reach / geometry.resolution())) + 1;
  const std::optional<map::CellRect> rect = map::bounding_cells(geometry, positions, margin);
  if (!rect.has_value()) {
    return std::nullopt;
  }

  const int size_x = rect->max_x - rect->min_x + 1;
  const int size_y = rect->max_y - rect->min_y + 1;
  std::vector<float> distance =
    distance_in_rect(grid, rect->min_x, rect->min_y, size_x, size_y, static_cast<float>(reach));
  const Eigen::Vector2d origin =
    geometry.origin() + Eigen::Vector2d{
                          static_cast<double>(rect->min_x) * geometry.resolution(),
                          static_cast<double>(rect->min_y) * geometry.resolution()};
  return ObstacleWindow{
    map::MapGeometry(size_x, size_y, geometry.resolution(), origin), std::move(distance), reach};
}

}  // namespace eltanin::planner::detail
