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
#include <eltanin/map/layers/raytrace_layer.hpp>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <optional>

namespace eltanin::map
{

namespace
{

/// Parameters along the segment that stay inside the box, in [0, 1] of the segment length.
struct SegmentSpan
{
  double entry;
  double exit;
};

std::optional<SegmentSpan> clip_to_box(
  const Eigen::Vector2d & from, const Eigen::Vector2d & to, const Eigen::Vector2d & box_min,
  const Eigen::Vector2d & box_max)
{
  const Eigen::Vector2d delta = to - from;
  double entry = 0.0;
  double exit = 1.0;
  for (int axis = 0; axis < 2; ++axis) {
    if (delta[axis] == 0.0) {
      if (from[axis] < box_min[axis] || from[axis] > box_max[axis]) {
        return std::nullopt;
      }
      continue;
    }
    const double to_min = (box_min[axis] - from[axis]) / delta[axis];
    const double to_max = (box_max[axis] - from[axis]) / delta[axis];
    entry = std::max(entry, std::min(to_min, to_max));
    exit = std::min(exit, std::max(to_min, to_max));
    if (entry > exit) {
      return std::nullopt;
    }
  }
  return SegmentSpan{entry, exit};
}

/// A lethal cell survives a clearing ray unless clearing was told it owns the obstacles.
void clear_cell(Costmap & master, int mx, int my, bool clear_static_obstacles)
{
  std::uint8_t & cell = master(mx, my);
  if (cell == LETHAL_OBSTACLE && !clear_static_obstacles) {
    return;
  }
  cell = FREE_SPACE;
}

/// Bresenham from `from` up to but excluding `to`: the last cell may hold what ended the beam.
void clear_line(
  Costmap & master, const MapIndex & from, const MapIndex & to, bool clear_static_obstacles)
{
  const int dx = std::abs(to.x - from.x);
  const int dy = std::abs(to.y - from.y);
  const int step_x = (from.x <= to.x) ? 1 : -1;
  const int step_y = (from.y <= to.y) ? 1 : -1;
  int x = from.x;
  int y = from.y;
  int error = dx - dy;
  while (x != to.x || y != to.y) {
    clear_cell(master, x, y, clear_static_obstacles);
    const int doubled_error = 2 * error;
    if (doubled_error > -dy) {
      error -= dy;
      x += step_x;
    }
    if (doubled_error < dx) {
      error += dx;
      y += step_y;
    }
  }
}

}  // namespace

RaytraceLayer::RaytraceLayer(bool clear_static_obstacles)
: clear_static_obstacles_(clear_static_obstacles)
{
}

void RaytraceLayer::set_observation(
  const Eigen::Vector2d & sensor_origin, std::span<const Eigen::Vector2d> marking_points,
  std::span<const Eigen::Vector2d> clearing_endpoints)
{
  sensor_origin_ = sensor_origin;
  marking_.set_points(marking_points);
  clearing_endpoints_.assign(clearing_endpoints.begin(), clearing_endpoints.end());
}

void RaytraceLayer::clear_observation()
{
  sensor_origin_ = Eigen::Vector2d::Zero();
  marking_.set_points({});
  clearing_endpoints_.clear();
}

void RaytraceLayer::update_costs(Costmap & master)
{
  const MapGeometry & geometry = master.geometry();
  if (geometry.cell_count() == 0) {
    return;
  }

  // Corner cell centres: a point inside them floors to an in-bounds cell, with half a cell spare.
  const Eigen::Vector2d box_min = geometry.map_to_world(0, 0);
  const Eigen::Vector2d box_max =
    geometry.map_to_world(geometry.size_x() - 1, geometry.size_y() - 1);

  for (const Eigen::Vector2d & endpoint : clearing_endpoints_) {
    const std::optional<SegmentSpan> span = clip_to_box(sensor_origin_, endpoint, box_min, box_max);
    if (!span.has_value()) {
      continue;
    }
    const Eigen::Vector2d delta = endpoint - sensor_origin_;
    const std::optional<MapIndex> from = geometry.world_to_map(sensor_origin_ + span->entry * delta);
    const std::optional<MapIndex> to = geometry.world_to_map(sensor_origin_ + span->exit * delta);
    if (!from.has_value() || !to.has_value()) {
      continue;
    }
    clear_line(master, *from, *to, clear_static_obstacles_);
  }

  marking_.update_costs(master);
}

}  // namespace eltanin::map
