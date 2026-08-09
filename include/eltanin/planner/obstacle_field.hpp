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

#include <cmath>
#include <optional>
#include <span>
#include <vector>

namespace eltanin::planner
{

namespace detail
{

/// Exact Euclidean distance [m] to the nearest cell this pass may not enter; off the map counts.
std::vector<float> build_obstacle_distance(const TraversabilityView & grid);

}  // namespace detail

/// A distance field over one window of the map, so the work follows the path and not the map size.
struct ObstacleWindow
{
  map::MapGeometry geometry;
  std::vector<float> distance;
  /// Anything at least this far away, or outside the window, reads as exactly this.
  double reach{0.0};
};

namespace detail
{

/// Distances accurate up to `reach` around `positions`; nullopt when none of them are on the map.
std::optional<ObstacleWindow> build_obstacle_window(
  const TraversabilityView & grid, std::span<const Eigen::Vector2d> positions, double reach);

}  // namespace detail

/// Keeps a search off the walls by charging extra for travel that has little room either side.
struct ClearanceCost
{
  /// Extra cost as a fraction of the distance travelled, at zero clearance; 0 disables the term.
  double penalty{0.0};
  /// Clearance [m] at and above which nothing extra is charged.
  double distance{0.5};
};

/// Fraction to add to a step of travel, falling linearly from `penalty` at a wall to 0 at `distance`.
[[nodiscard]] inline double clearance_penalty(
  const ClearanceCost & cost, double clearance) noexcept
{
  if (cost.penalty <= 0.0 || cost.distance <= 0.0 || clearance >= cost.distance) {
    return 0.0;
  }
  return cost.penalty * (1.0 - clearance / cost.distance);
}

namespace detail
{

/// Throws std::invalid_argument for a negative penalty or a non-positive falloff distance.
void validate_clearance_cost(const ClearanceCost & cost);

}  // namespace detail

/// Non-owning view over an obstacle distance field, shared by the search cost and the smoother.
class ObstacleField
{
public:
  ObstacleField() noexcept = default;

  /// `outside` is what queries off the field report; a window passes its reach, a whole map 0.
  ObstacleField(
    const map::MapGeometry & geometry, std::span<const float> distance,
    double outside = 0.0) noexcept
  : geometry_(&geometry), distance_(distance), outside_(outside)
  {
  }

  /// The field covering just the window, reporting its reach everywhere outside it.
  explicit ObstacleField(const ObstacleWindow & window) noexcept
  : geometry_(&window.geometry), distance_(window.distance), outside_(window.reach)
  {
  }

  [[nodiscard]] bool empty() const noexcept { return distance_.empty(); }

  /// Distance [m] to the nearest blocked cell; queries off the field report `outside`.
  [[nodiscard]] double at(int mx, int my) const noexcept
  {
    if (geometry_ == nullptr || distance_.empty() || !geometry_->in_bounds(mx, my)) {
      return outside_;
    }
    return static_cast<double>(distance_[geometry_->index(mx, my)]);
  }

  /// Distance [m] at a world point, interpolated between cell centers so the field is continuous.
  [[nodiscard]] double at(const Eigen::Vector2d & world) const noexcept
  {
    Patch patch;
    if (!locate(world, patch)) {
      return outside_;
    }
    return (1.0 - patch.fy) * ((1.0 - patch.fx) * patch.d00 + patch.fx * patch.d10) +
           patch.fy * ((1.0 - patch.fx) * patch.d01 + patch.fx * patch.d11);
  }

  /// Gradient of that interpolation; it points away from the nearest obstacle, or is 0 where flat.
  [[nodiscard]] Eigen::Vector2d gradient(const Eigen::Vector2d & world) const noexcept
  {
    Patch patch;
    if (!locate(world, patch)) {
      return Eigen::Vector2d::Zero();
    }
    const double resolution = geometry_->resolution();
    return Eigen::Vector2d{
      ((1.0 - patch.fy) * (patch.d10 - patch.d00) + patch.fy * (patch.d11 - patch.d01)) /
        resolution,
      ((1.0 - patch.fx) * (patch.d01 - patch.d00) + patch.fx * (patch.d11 - patch.d10)) /
        resolution};
  }

private:
  /// The four cell centers around a world point, with where the point sits between them.
  struct Patch
  {
    double d00;
    double d10;
    double d01;
    double d11;
    double fx;
    double fy;
  };

  /// False when there is no field to read; cell centers sit half a cell inside the map corner.
  [[nodiscard]] bool locate(const Eigen::Vector2d & world, Patch & patch) const noexcept
  {
    if (geometry_ == nullptr || distance_.empty() || !world.allFinite()) {
      return false;
    }
    const Eigen::Vector2d offset =
      (world - geometry_->origin()) / geometry_->resolution() - Eigen::Vector2d{0.5, 0.5};
    const double floor_x = std::floor(offset.x());
    const double floor_y = std::floor(offset.y());
    const auto mx = static_cast<int>(floor_x);
    const auto my = static_cast<int>(floor_y);
    patch.fx = offset.x() - floor_x;
    patch.fy = offset.y() - floor_y;
    patch.d00 = at(mx, my);
    patch.d10 = at(mx + 1, my);
    patch.d01 = at(mx, my + 1);
    patch.d11 = at(mx + 1, my + 1);
    return true;
  }

  const map::MapGeometry * geometry_{nullptr};
  std::span<const float> distance_{};
  double outside_{0.0};
};

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__OBSTACLE_FIELD_HPP_
