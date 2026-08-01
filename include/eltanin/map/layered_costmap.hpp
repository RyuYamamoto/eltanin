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

#ifndef ELTANIN__MAP__LAYERED_COSTMAP_HPP_
#define ELTANIN__MAP__LAYERED_COSTMAP_HPP_

#include <eltanin/core/types.hpp>
#include <eltanin/map/cost_values.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/map/layers/layer.hpp>

#include <concepts>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace eltanin::map
{

/// Owns the master costmap and the layers that write into it. Only this class moves the origin.
class LayeredCostmap
{
public:
  /// Throws std::invalid_argument unless geometry describes a usable map.
  LayeredCostmap(const MapGeometry & geometry, std::uint8_t default_cost);

  /// Layers are applied in registration order; the reference stays valid for this object's life.
  template <class LayerType, class... Args>
  LayerType & add_layer(Args &&... args)
  {
    static_assert(std::derived_from<LayerType, Layer>, "a layer must derive from Layer");
    auto layer = std::make_unique<LayerType>(std::forward<Args>(args)...);
    LayerType & added = *layer;
    layers_.push_back(std::move(layer));
    return added;
  }

  /// Resets the master to default_cost, then applies every layer in registration order.
  void update();

  const Costmap & costmap() const noexcept { return costmap_; }

  const MapGeometry & geometry() const noexcept { return costmap_.geometry(); }

  /// Moves the window without shifting the cells; update() regenerates them anyway.
  void set_origin(const Eigen::Vector2d & origin);

  /// Places the robot at the centre of the window.
  void center_on(const Eigen::Vector2d & robot_position);

private:
  Costmap costmap_;
  std::uint8_t default_cost_{NO_INFORMATION};
  std::vector<std::unique_ptr<Layer>> layers_;
};

}  // namespace eltanin::map

#endif  // ELTANIN__MAP__LAYERED_COSTMAP_HPP_
