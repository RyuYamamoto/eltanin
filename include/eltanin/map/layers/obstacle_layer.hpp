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

#ifndef ELTANIN__MAP__LAYERS__OBSTACLE_LAYER_HPP_
#define ELTANIN__MAP__LAYERS__OBSTACLE_LAYER_HPP_

#include <eltanin/core/types.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/map/layers/layer.hpp>

#include <span>
#include <vector>

namespace eltanin::map
{

/// Marks the cells hit by world-frame obstacle points as LETHAL_OBSTACLE.
class ObstacleLayer final : public Layer
{
public:
  ObstacleLayer() = default;

  /// Copies the points, so the caller's buffer only has to outlive this call.
  void set_points(std::span<const Eigen::Vector2d> points);

  /// Points outside the master map are discarded; NO_INFORMATION is overwritten.
  void update_costs(Costmap & master) override;

private:
  std::vector<Eigen::Vector2d> points_;
};

}  // namespace eltanin::map

#endif  // ELTANIN__MAP__LAYERS__OBSTACLE_LAYER_HPP_
