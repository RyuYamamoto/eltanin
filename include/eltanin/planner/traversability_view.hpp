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

#ifndef ELTANIN__PLANNER__TRAVERSABILITY_VIEW_HPP_
#define ELTANIN__PLANNER__TRAVERSABILITY_VIEW_HPP_

#include <eltanin/core/traversability.hpp>
#include <eltanin/core/types.hpp>
#include <eltanin/map/cell_map.hpp>
#include <eltanin/map/map_geometry.hpp>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace eltanin::planner
{

namespace detail
{

/// One classified cell per byte, holding the Traversability enumerator value.
using TraversabilityGrid = std::vector<std::uint8_t>;

static_assert(static_cast<int>(Traversability::Free) == 0);
static_assert(static_cast<int>(Traversability::Circumscribed) == 1);
static_assert(static_cast<int>(Traversability::Inscribed) == 2);

/// The only place the cell type and traversability model are visible to a planner search core.
template <map::CellMap Map, class Model>
  requires TraversabilityModel<Model, typename Map::value_type>
TraversabilityGrid build_traversability_grid(const Map & map, const Model & model)
{
  const map::MapGeometry & geometry = map.geometry();
  TraversabilityGrid grid(geometry.cell_count(), 0);
  for (int my = 0; my < geometry.size_y(); ++my) {
    for (int mx = 0; mx < geometry.size_x(); ++mx) {
      grid[geometry.index(mx, my)] = static_cast<std::uint8_t>(model.classify(map(mx, my)));
    }
  }
  return grid;
}

}  // namespace detail

/// What one search pass may enter, and what it is charged for entering it.
struct TraversabilityPolicy
{
  /// The worst class the pass may enter; Free is the strict pass every plan starts with.
  Traversability limit{Traversability::Free};
  /// Extra cost of travelling through a Circumscribed cell, as a fraction of the distance.
  double circumscribed_penalty{0.0};
};

/// Non-owning view over a classified grid; the only traversability test a search core may use.
class TraversabilityView
{
public:
  TraversabilityView(
    const map::MapGeometry & geometry, std::span<const std::uint8_t> grid,
    const TraversabilityPolicy & policy = {}) noexcept
  : geometry_(&geometry),
    grid_(grid),
    limit_(static_cast<std::uint8_t>(policy.limit)),
    circumscribed_penalty_(policy.circumscribed_penalty)
  {
    assert(grid_.size() == geometry.cell_count());
  }

  [[nodiscard]] const map::MapGeometry & geometry() const noexcept { return *geometry_; }

  [[nodiscard]] std::size_t cell_count() const noexcept { return grid_.size(); }

  /// True when this pass accepts more than Traversability::Free.
  [[nodiscard]] bool relaxed() const noexcept { return limit_ > FREE_CELL; }

  /// Cells outside the map are not traversable; in_bounds() is always evaluated before index().
  [[nodiscard]] bool traversable(int mx, int my) const noexcept
  {
    return geometry_->in_bounds(mx, my) && grid_[geometry_->index(mx, my)] <= limit_;
  }

  /// Points outside the map are not traversable.
  [[nodiscard]] bool traversable(const Eigen::Vector2d & world) const noexcept
  {
    const auto index = geometry_->world_to_map(world);
    return index.has_value() && grid_[geometry_->index(index->x, index->y)] <= limit_;
  }

  /// Fraction to add to a step of travel ending here; 0 unless the cell is Circumscribed.
  [[nodiscard]] double surcharge(int mx, int my) const noexcept
  {
    if (!geometry_->in_bounds(mx, my)) {
      return 0.0;
    }
    return grid_[geometry_->index(mx, my)] == CIRCUMSCRIBED_CELL ? circumscribed_penalty_ : 0.0;
  }

  /// Precondition: geometry().in_bounds(mx, my).
  [[nodiscard]] Traversability at(int mx, int my) const noexcept
  {
    assert(geometry_->in_bounds(mx, my));
    return static_cast<Traversability>(grid_[geometry_->index(mx, my)]);
  }

private:
  static constexpr std::uint8_t FREE_CELL = static_cast<std::uint8_t>(Traversability::Free);
  static constexpr std::uint8_t CIRCUMSCRIBED_CELL =
    static_cast<std::uint8_t>(Traversability::Circumscribed);

  const map::MapGeometry * geometry_;
  std::span<const std::uint8_t> grid_;
  std::uint8_t limit_{FREE_CELL};
  double circumscribed_penalty_{0.0};
};

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__TRAVERSABILITY_VIEW_HPP_
