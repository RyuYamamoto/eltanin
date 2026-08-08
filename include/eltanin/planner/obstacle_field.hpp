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

#ifndef ELTANIN__PLANNER__OBSTACLE_FIELD_HPP_
#define ELTANIN__PLANNER__OBSTACLE_FIELD_HPP_

#include <eltanin/core/types.hpp>
#include <eltanin/map/map_geometry.hpp>
#include <eltanin/planner/traversability_view.hpp>

#include <span>
#include <vector>

namespace eltanin::planner
{

namespace detail
{

/// Exact Euclidean distance [m] to the nearest non-Free cell; leaving the map counts as blocked.
std::vector<float> build_obstacle_distance(const TraversabilityView & grid);

}  // namespace detail

/// Non-owning view over an obstacle distance field, shared by the search cost and the smoother.
class ObstacleField
{
public:
  ObstacleField() noexcept = default;

  ObstacleField(const map::MapGeometry & geometry, std::span<const float> distance) noexcept
  : geometry_(&geometry), distance_(distance)
  {
  }

  [[nodiscard]] bool empty() const noexcept { return distance_.empty(); }

  /// Distance [m] to the nearest blocked cell; cells outside the map report 0.
  [[nodiscard]] double at(int mx, int my) const noexcept
  {
    if (geometry_ == nullptr || !geometry_->in_bounds(mx, my)) {
      return 0.0;
    }
    return static_cast<double>(distance_[geometry_->index(mx, my)]);
  }

  /// Distance [m] at a world point, taken from the cell containing it.
  [[nodiscard]] double at(const Eigen::Vector2d & world) const noexcept
  {
    if (geometry_ == nullptr) {
      return 0.0;
    }
    const auto index = geometry_->world_to_map(world);
    return index.has_value() ? at(index->x, index->y) : 0.0;
  }

  /// Central difference of at(); points away from the nearest obstacle and is 0 where flat.
  [[nodiscard]] Eigen::Vector2d gradient(const Eigen::Vector2d & world) const noexcept
  {
    if (geometry_ == nullptr) {
      return Eigen::Vector2d::Zero();
    }
    const auto index = geometry_->world_to_map(world);
    if (!index.has_value()) {
      return Eigen::Vector2d::Zero();
    }
    const double span = 2.0 * geometry_->resolution();
    return Eigen::Vector2d{
      (at(index->x + 1, index->y) - at(index->x - 1, index->y)) / span,
      (at(index->x, index->y + 1) - at(index->x, index->y - 1)) / span};
  }

private:
  const map::MapGeometry * geometry_{nullptr};
  std::span<const float> distance_{};
};

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__OBSTACLE_FIELD_HPP_
