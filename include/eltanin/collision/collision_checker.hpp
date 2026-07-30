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

/// Maps the centre-cell classification onto the two-stage policy of docs/safety-design.md.
FirstStage classify_first_stage(Traversability classification) noexcept;

}  // namespace detail

/// Two-stage check; the cheap classification of the centre cell gates the oriented polygon test.
template <map::CellMap Map, class Model>
  requires TraversabilityModel<Model, typename Map::value_type> &&
           ObstacleModel<Model, typename Map::value_type>
CollisionCheck check_footprint(
  const Map & map, const Model & model, const Polygon2D & footprint, const Pose2D & pose)
{
  const map::MapGeometry & geometry = map.geometry();
  const std::optional<map::MapIndex> centre = geometry.world_to_map(pose.position);
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

  const Polygon2D world_footprint = transform(footprint, pose);
  const auto [min, max] = bounding_box(world_footprint);
  const std::optional<map::CellRect> rect = geometry.world_rect_to_cells(min, max);
  if (!rect.has_value()) {
    return CollisionCheck::Free;
  }
  for (int my = rect->min_y; my <= rect->max_y; ++my) {
    for (int mx = rect->min_x; mx <= rect->max_x; ++mx) {
      // Cheap first: a one-byte occupancy compare gates the O(n) containment test.
      if (
        model.is_obstacle(map(mx, my)) &&
        contains(world_footprint, geometry.map_to_world(mx, my))) {
        return CollisionCheck::Collision;
      }
    }
  }
  return CollisionCheck::Free;
}

}  // namespace eltanin::collision

#endif  // ELTANIN__COLLISION__COLLISION_CHECKER_HPP_
