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

#ifndef ELTANIN__COLLISION__COLLISION_CHECKER_HPP_
#define ELTANIN__COLLISION__COLLISION_CHECKER_HPP_

#include <eltanin/core/polygon.hpp>
#include <eltanin/core/traversability.hpp>
#include <eltanin/core/types.hpp>
#include <eltanin/map/cell_map.hpp>
#include <eltanin/map/map_geometry.hpp>

#include <optional>
#include <span>

namespace eltanin::collision
{

/// Result of one footprint check; OutsideMap means the robot origin left the map.
enum class CollisionCheck
{
  Free,
  Collision,
  OutsideMap
};

namespace detail
{

/// Verdict of the cheap first stage; NeedsExactCheck is the only case that depends on the heading.
enum class FirstStage
{
  NoCollision,
  Collision,
  NeedsExactCheck
};

/// Maps the centre-cell classification onto the two-stage policy of docs/collision-design.md.
FirstStage classify_first_stage(Traversability classification) noexcept;

}  // namespace detail

/// Clamped cell rectangle that can hold a centre inside the polygon; nullopt when it misses the map.
std::optional<map::CellRect> cells_covering(
  const map::MapGeometry & geometry, const Polygon2D & polygon);

/// True when any point lies inside the polygon; points on the boundary count as inside.
bool contains_any(const Polygon2D & polygon, std::span<const Eigen::Vector2d> points);

/// Occupancy of one cell; a cell outside the map is reported as not occupied.
template <map::CellMap Map, class Model>
  requires ObstacleModel<Model, typename Map::value_type>
bool is_cell_occupied(const Map & map, const Model & model, int mx, int my)
{
  const std::optional<typename Map::value_type> cell = map.get(mx, my);
  return cell.has_value() && model.is_obstacle(*cell);
}

/// True when the centre of an occupied cell lies inside the polygon, which must be in world frame.
template <map::CellMap Map, class Model>
  requires ObstacleModel<Model, typename Map::value_type>
bool contains_occupied_cell(const Map & map, const Model & model, const Polygon2D & polygon)
{
  const map::MapGeometry & geometry = map.geometry();
  const std::optional<map::CellRect> rect = cells_covering(geometry, polygon);
  if (!rect.has_value()) {
    return false;
  }
  for (int my = rect->min_y; my <= rect->max_y; ++my) {
    for (int mx = rect->min_x; mx <= rect->max_x; ++mx) {
      // The rectangle is already clamped, so the raw accessor replaces the bounds-checked one here.
      if (model.is_obstacle(map(mx, my)) && contains(polygon, geometry.map_to_world(mx, my))) {
        return true;
      }
    }
  }
  return false;
}

/// Two-stage check; the cheap classification of the centre cell gates the oriented polygon test.
template <map::CellMap Map, class Model>
  requires TraversabilityModel<Model, typename Map::value_type> &&
           ObstacleModel<Model, typename Map::value_type>
CollisionCheck check_footprint(
  const Map & map, const Model & model, const Polygon2D & footprint, const Pose2D & pose)
{
  const std::optional<map::MapIndex> centre = map.geometry().world_to_map(pose.position);
  if (!centre.has_value()) {
    return CollisionCheck::OutsideMap;
  }

  const detail::FirstStage stage =
    detail::classify_first_stage(model.classify(map(centre->x, centre->y)));
  if (stage == detail::FirstStage::NoCollision) {
    return CollisionCheck::Free;
  }
  if (stage == detail::FirstStage::Collision) {
    return CollisionCheck::Collision;
  }
  return contains_occupied_cell(map, model, transform(footprint, pose))
           ? CollisionCheck::Collision
           : CollisionCheck::Free;
}

/// Footprint against a point set such as a projected scan; there is no map and so no OutsideMap.
inline bool footprint_hits_points(
  const Polygon2D & footprint, const Pose2D & pose, std::span<const Eigen::Vector2d> points)
{
  return contains_any(transform(footprint, pose), points);
}

}  // namespace eltanin::collision

#endif  // ELTANIN__COLLISION__COLLISION_CHECKER_HPP_
