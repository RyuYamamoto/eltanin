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

#ifndef ELTANIN_TEST__MAP__COSTMAP_FIXTURE_HPP_
#define ELTANIN_TEST__MAP__COSTMAP_FIXTURE_HPP_

#include <eltanin/core/footprint.hpp>
#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/cost_values.hpp>
#include <eltanin/map/grid_map.hpp>

#include <cassert>
#include <cstdint>
#include <initializer_list>
#include <string_view>
#include <vector>

namespace eltanin_test
{

/// Rows are given top-down, so the first row is my = size_y - 1 and matches write_pgm output.
inline eltanin::map::Costmap make_costmap(
  std::initializer_list<std::string_view> rows, double resolution,
  const Eigen::Vector2d & origin = Eigen::Vector2d::Zero())
{
  assert(rows.size() > 0);
  const int size_y = static_cast<int>(rows.size());
  const int size_x = static_cast<int>(rows.begin()->size());
  eltanin::map::Costmap map(
    eltanin::map::MapGeometry(size_x, size_y, resolution, origin), eltanin::map::FREE_SPACE);

  int row_from_top = 0;
  for (const std::string_view row : rows) {
    assert(static_cast<int>(row.size()) == size_x);
    const int my = size_y - 1 - row_from_top;
    for (int mx = 0; mx < size_x; ++mx) {
      switch (row[static_cast<std::size_t>(mx)]) {
        case '.':
          map(mx, my) = eltanin::map::FREE_SPACE;
          break;
        case '#':
          map(mx, my) = eltanin::map::LETHAL_OBSTACLE;
          break;
        case '?':
          map(mx, my) = eltanin::map::NO_INFORMATION;
          break;
        default:
          assert(false && "unsupported costmap fixture character");
          break;
      }
    }
    ++row_from_top;
  }
  return map;
}

/// Cell values listed top-down, in the same row order as make_costmap.
inline std::vector<std::uint8_t> cells_top_down(const eltanin::map::Costmap & map)
{
  std::vector<std::uint8_t> values;
  values.reserve(map.cell_count());
  for (int my = map.size_y() - 1; my >= 0; --my) {
    for (int mx = 0; mx < map.size_x(); ++mx) {
      values.push_back(map(mx, my));
    }
  }
  return values;
}

inline eltanin::map::InflationCostModel make_inflation_model(
  double inscribed, double circumscribed, double inflation, double cost_scaling_factor)
{
  const auto radii = eltanin::CollisionRadii::from_radii(inscribed, circumscribed, inflation);
  assert(radii.has_value());
  const auto model = eltanin::map::InflationCostModel::create(*radii, cost_scaling_factor);
  assert(model.has_value());
  return *model;
}

}  // namespace eltanin_test

#endif  // ELTANIN_TEST__MAP__COSTMAP_FIXTURE_HPP_
