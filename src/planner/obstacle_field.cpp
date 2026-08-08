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

#include <algorithm>
#include <cmath>
#include <cstddef>
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

}  // namespace

std::vector<float> build_obstacle_distance(const TraversabilityView & grid)
{
  const map::MapGeometry & geometry = grid.geometry();
  const int size_x = geometry.size_x();
  const int size_y = geometry.size_y();
  const auto cells = static_cast<std::size_t>(size_x) * static_cast<std::size_t>(size_y);

  std::vector<float> squared(cells, 0.0F);
  for (int my = 0; my < size_y; ++my) {
    for (int mx = 0; mx < size_x; ++mx) {
      squared[geometry.index(mx, my)] = grid.free(mx, my) ? UNREACHED : 0.0F;
    }
  }

  const auto longest = static_cast<std::size_t>(std::max(size_x, size_y));
  std::vector<float> line(longest, 0.0F);
  std::vector<float> result(longest, 0.0F);
  std::vector<int> hull(longest + 1, 0);
  std::vector<float> boundary(longest + 2, 0.0F);

  for (int mx = 0; mx < size_x; ++mx) {
    for (int my = 0; my < size_y; ++my) {
      line[static_cast<std::size_t>(my)] = squared[geometry.index(mx, my)];
    }
    transform_line(
      std::span<const float>{line.data(), static_cast<std::size_t>(size_y)},
      std::span<float>{result.data(), static_cast<std::size_t>(size_y)}, hull, boundary);
    for (int my = 0; my < size_y; ++my) {
      squared[geometry.index(mx, my)] = result[static_cast<std::size_t>(my)];
    }
  }

  for (int my = 0; my < size_y; ++my) {
    for (int mx = 0; mx < size_x; ++mx) {
      line[static_cast<std::size_t>(mx)] = squared[geometry.index(mx, my)];
    }
    transform_line(
      std::span<const float>{line.data(), static_cast<std::size_t>(size_x)},
      std::span<float>{result.data(), static_cast<std::size_t>(size_x)}, hull, boundary);
    for (int mx = 0; mx < size_x; ++mx) {
      squared[geometry.index(mx, my)] = result[static_cast<std::size_t>(mx)];
    }
  }

  const auto resolution = static_cast<float>(geometry.resolution());
  for (int my = 0; my < size_y; ++my) {
    for (int mx = 0; mx < size_x; ++mx) {
      // Driving off the map is no more allowed than driving into a wall, so the border bounds it.
      const int to_border = std::min({mx + 1, my + 1, size_x - mx, size_y - my});
      const float inside = std::sqrt(squared[geometry.index(mx, my)]);
      squared[geometry.index(mx, my)] =
        std::min(inside, static_cast<float>(to_border)) * resolution;
    }
  }
  return squared;
}

}  // namespace eltanin::planner::detail
