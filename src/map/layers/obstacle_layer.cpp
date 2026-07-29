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
#include <eltanin/map/layers/obstacle_layer.hpp>

#include <optional>

namespace eltanin::map
{

void ObstacleLayer::set_points(std::span<const Eigen::Vector2d> points)
{
  points_.assign(points.begin(), points.end());
}

void ObstacleLayer::update_costs(Costmap & master)
{
  const MapGeometry & geometry = master.geometry();
  for (const Eigen::Vector2d & point : points_) {
    const std::optional<MapIndex> index = geometry.world_to_map(point);
    if (!index.has_value()) {
      continue;
    }
    master(index->x, index->y) = LETHAL_OBSTACLE;
  }
}

}  // namespace eltanin::map
