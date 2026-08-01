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

#include <eltanin/map/layered_costmap.hpp>

#include <stdexcept>

namespace eltanin::map
{

LayeredCostmap::LayeredCostmap(const MapGeometry & geometry, std::uint8_t default_cost)
: costmap_(geometry, default_cost), default_cost_(default_cost)
{
  if (geometry.cell_count() == 0 || geometry.resolution() <= 0.0) {
    throw std::invalid_argument("LayeredCostmap requires a usable geometry");
  }
}

void LayeredCostmap::update()
{
  costmap_.fill(default_cost_);
  for (const std::unique_ptr<Layer> & layer : layers_) {
    layer->update_costs(costmap_);
  }
}

void LayeredCostmap::set_origin(const Eigen::Vector2d & origin)
{
  costmap_.set_origin(origin);
}

void LayeredCostmap::center_on(const Eigen::Vector2d & robot_position)
{
  const MapGeometry & map_geometry = costmap_.geometry();
  const Eigen::Vector2d extent{
    static_cast<double>(map_geometry.size_x()) * map_geometry.resolution(),
    static_cast<double>(map_geometry.size_y()) * map_geometry.resolution()};
  set_origin(robot_position - extent / 2.0);
}

}  // namespace eltanin::map
