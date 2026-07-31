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

#include <eltanin/collision/collision_checker.hpp>

namespace eltanin::collision
{

namespace detail
{

FirstStage classify_first_stage(Traversability classification) noexcept
{
  switch (classification) {
    case Traversability::Free:
      return FirstStage::NoCollision;
    case Traversability::Inscribed:
      return FirstStage::Collision;
    case Traversability::Circumscribed:
      break;
  }
  return FirstStage::NeedsExactCheck;
}

FirstStage classify_first_stage_exact(Traversability classification) noexcept
{
  return classify_first_stage(classification) == FirstStage::Collision
           ? FirstStage::Collision
           : FirstStage::NeedsExactCheck;
}

}  // namespace detail

std::optional<map::CellRect> cells_covering(
  const map::MapGeometry & geometry, const Polygon2D & polygon)
{
  if (polygon.empty()) {
    return std::nullopt;
  }
  const auto [min, max] = bounding_box(polygon);
  return geometry.world_rect_to_cells(min, max);
}

bool contains_any(const Polygon2D & polygon, std::span<const Eigen::Vector2d> points)
{
  for (const Eigen::Vector2d & point : points) {
    if (contains(polygon, point)) {
      return true;
    }
  }
  return false;
}

}  // namespace eltanin::collision
