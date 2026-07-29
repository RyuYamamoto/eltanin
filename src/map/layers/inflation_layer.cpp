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

#include <eltanin/map/cost_values.hpp>
#include <eltanin/map/layers/inflation_layer.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>

namespace eltanin::map
{

namespace
{

/// Marks an offset that lies outside inflation_radius; such cells are never written.
constexpr std::int16_t OUTSIDE_INFLATION_RADIUS = -1;

}  // namespace

InflationLayer::InflationLayer(const InflationCostModel & model, bool inflate_unknown)
: model_(model), inflate_unknown_(inflate_unknown)
{
}

void InflationLayer::update_costs(Costmap & master)
{
  const double resolution = master.geometry().resolution();
  assert(resolution > 0.0);
  if (resolution != lut_resolution_) {
    rebuild_cost_lut(resolution);
  }

  const int size_x = master.size_x();
  const int size_y = master.size_y();
  for (int my = 0; my < size_y; ++my) {
    for (int mx = 0; mx < size_x; ++mx) {
      if (master(mx, my) == LETHAL_OBSTACLE) {
        inflate_from(master, mx, my);
      }
    }
  }
}

void InflationLayer::rebuild_cost_lut(double resolution)
{
  constexpr double MAX_RADIUS_IN_CELLS = static_cast<double>(std::numeric_limits<int>::max());
  const double inflation_radius = model_.radii().inflation_radius();
  const double radius_in_cells = std::ceil(inflation_radius / resolution);
  assert(radius_in_cells <= MAX_RADIUS_IN_CELLS);

  cell_inflation_radius_ = static_cast<int>(std::min(radius_in_cells, MAX_RADIUS_IN_CELLS));
  lut_resolution_ = resolution;

  const std::size_t side = static_cast<std::size_t>(cell_inflation_radius_) + 1;
  cost_lut_.assign(side * side, OUTSIDE_INFLATION_RADIUS);
  for (int abs_dy = 0; abs_dy <= cell_inflation_radius_; ++abs_dy) {
    for (int abs_dx = 0; abs_dx <= cell_inflation_radius_; ++abs_dx) {
      const double distance =
        std::hypot(static_cast<double>(abs_dx), static_cast<double>(abs_dy)) * resolution;
      if (distance > inflation_radius) {
        continue;
      }
      const std::size_t offset = static_cast<std::size_t>(abs_dy) * side +
                                 static_cast<std::size_t>(abs_dx);
      cost_lut_[offset] = static_cast<std::int16_t>(model_.cost_at_distance(distance));
    }
  }
}

std::int16_t InflationLayer::lut_cost(int abs_dx, int abs_dy) const noexcept
{
  assert(abs_dx >= 0 && abs_dx <= cell_inflation_radius_);
  assert(abs_dy >= 0 && abs_dy <= cell_inflation_radius_);
  const std::size_t side = static_cast<std::size_t>(cell_inflation_radius_) + 1;
  return cost_lut_[static_cast<std::size_t>(abs_dy) * side + static_cast<std::size_t>(abs_dx)];
}

void InflationLayer::inflate_from(Costmap & master, int mx, int my) const
{
  const int radius = cell_inflation_radius_;
  const int min_x = std::max(0, mx - radius);
  const int max_x = std::min(master.size_x() - 1, mx + radius);
  const int min_y = std::max(0, my - radius);
  const int max_y = std::min(master.size_y() - 1, my + radius);

  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      const std::int16_t cost = lut_cost(std::abs(x - mx), std::abs(y - my));
      if (cost == OUTSIDE_INFLATION_RADIUS) {
        continue;
      }
      std::uint8_t & cell = master(x, y);
      if (cell == NO_INFORMATION) {
        if (!inflate_unknown_) {
          continue;
        }
        cell = static_cast<std::uint8_t>(cost);
      } else {
        cell = std::max(cell, static_cast<std::uint8_t>(cost));
      }
    }
  }
}

}  // namespace eltanin::map
