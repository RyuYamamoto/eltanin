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

#ifndef ELTANIN__MAP__LAYERS__RAYTRACE_LAYER_HPP_
#define ELTANIN__MAP__LAYERS__RAYTRACE_LAYER_HPP_

#include <eltanin/core/types.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/map/layers/layer.hpp>
#include <eltanin/map/layers/obstacle_layer.hpp>

#include <span>
#include <vector>

namespace eltanin::map
{

/// Writes one scan as FREE_SPACE along every beam and LETHAL_OBSTACLE where a beam ended.
class RaytraceLayer final : public Layer
{
public:
  /// The composition rules are the specification of this layer; see `docs/costmap-design.md` §16.
  explicit RaytraceLayer(bool clear_static_obstacles = false);

  /// Copies both point sets, so the caller's buffers only have to outlive this call.
  void set_observation(
    const Eigen::Vector2d & sensor_origin, std::span<const Eigen::Vector2d> marking_points,
    std::span<const Eigen::Vector2d> clearing_endpoints);

  /// Drops the observation, so a scan that went stale is not applied again on the next update.
  void clear_observation();

  /// Clears first, then marks, so no beam of this scan can clear a cell another beam marked.
  void update_costs(Costmap & master) override;

private:
  ObstacleLayer marking_;
  std::vector<Eigen::Vector2d> clearing_endpoints_;
  Eigen::Vector2d sensor_origin_{Eigen::Vector2d::Zero()};
  bool clear_static_obstacles_{false};
};

}  // namespace eltanin::map

#endif  // ELTANIN__MAP__LAYERS__RAYTRACE_LAYER_HPP_
