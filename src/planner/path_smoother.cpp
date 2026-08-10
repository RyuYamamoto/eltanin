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

#include <eltanin/planner/path_smoother.hpp>

#include <eltanin/core/angle.hpp>

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace eltanin::planner::detail
{

void assign_tangent_yaw(Path & path)
{
  const std::size_t n = path.size();
  if (n <= 1) {
    return;
  }

  bool has_tangent = false;
  double tangent_yaw = 0.0;
  std::size_t first_tangent = n;
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const Eigen::Vector2d delta = path[i + 1].position - path[i].position;
    if (delta.x() != 0.0 || delta.y() != 0.0) {
      tangent_yaw = std::atan2(delta.y(), delta.x());
      has_tangent = true;
      if (first_tangent == n) {
        first_tangent = i;
      }
    }
    // A turn on the spot has no tangent, so the pose keeps the heading the planner gave it.
    if (path.direction_of(i) == Direction::InPlace) {
      continue;
    }
    // A coincident pair carries the previous tangent forward instead of atan2(0, 0).
    if (has_tangent) {
      // The yaw is the body heading, which on a reverse segment is the tangent turned by pi.
      path[i].yaw = path.direction_of(i) == Direction::Reverse
                      ? normalize_angle(tangent_yaw + std::numbers::pi)
                      : tangent_yaw;
    }
  }

  if (first_tangent == n) {
    // Every point coincides, so the terminal yaw is the only orientation available.
    for (std::size_t i = 0; i + 1 < n; ++i) {
      if (path.direction_of(i) != Direction::InPlace) {
        path[i].yaw = path[n - 1].yaw;
      }
    }
    return;
  }
  for (std::size_t i = 0; i < first_tangent; ++i) {
    if (path.direction_of(i) != Direction::InPlace) {
      path[i].yaw = path[first_tangent].yaw;
    }
  }
}

double smoother_reach(const SmootherParams & params) noexcept
{
  return params.weight_obstacle > 0.0 ? params.obstacle_distance : 0.0;
}

Path smooth_on_grid(
  const Path & path, const TraversabilityView & grid, const ClearanceMap & obstacle,
  const SmootherParams & params)
{
  if (path.size() <= 1) {
    return path;
  }

  Path smoothed = path;
  const std::size_t n = smoothed.size();
  const bool push_off_walls = params.weight_obstacle > 0.0 && obstacle.cell_count() > 0;
  if (n >= 3) {
    for (int iteration = 0; iteration < params.max_iterations; ++iteration) {
      double displacement = 0.0;
      for (std::size_t i = 1; i + 1 < n; ++i) {
        // A cusp is where the body reverses; rounding it off would erase the manoeuvre.
        if (smoothed.is_cusp(i)) {
          continue;
        }
        const Eigen::Vector2d & here = smoothed[i].position;
        Eigen::Vector2d step =
          params.weight_data * (path[i].position - here) +
          params.weight_smooth * (smoothed[i - 1].position + smoothed[i + 1].position - 2.0 * here);
        // Bending needs two neighbours each side, so the points next to the ends keep their shape.
        if (params.weight_curvature > 0.0 && i >= 2 && i + 2 < n) {
          step -= params.weight_curvature *
                  (6.0 * here - 4.0 * smoothed[i - 1].position - 4.0 * smoothed[i + 1].position +
                   smoothed[i - 2].position + smoothed[i + 2].position);
        }
        if (push_off_walls) {
          const double room = clearance_at(obstacle, here, params.obstacle_distance);
          if (room < params.obstacle_distance) {
            step += params.weight_obstacle * (params.obstacle_distance - room) *
                    clearance_gradient(obstacle, here);
          }
        }

        const Eigen::Vector2d candidate = here + step;
        // Rejected for this sweep only; a neighbour moving later may make the point movable.
        if (!grid.traversable(candidate)) {
          continue;
        }
        displacement += step.norm();
        smoothed[i].position = candidate;
      }
      if (displacement < params.tolerance) {
        break;
      }
    }
  }
  assign_tangent_yaw(smoothed);
  return smoothed;
}

void validate_smoother_params(const SmootherParams & params)
{
  // The sweep applies the gradient directly, so the stencil's largest eigenvalue has to stay under 2.
  const bool valid =
    std::isfinite(params.weight_data) && params.weight_data >= 0.0 &&
    std::isfinite(params.weight_smooth) && params.weight_smooth >= 0.0 &&
    std::isfinite(params.weight_curvature) && params.weight_curvature >= 0.0 &&
    std::isfinite(params.weight_obstacle) && params.weight_obstacle >= 0.0 &&
    std::isfinite(params.obstacle_distance) && params.obstacle_distance > 0.0 &&
    params.weight_data + 4.0 * params.weight_smooth + 16.0 * params.weight_curvature < 2.0 &&
    std::isfinite(params.tolerance) && params.tolerance >= 0.0 && params.max_iterations >= 0;
  if (!valid) {
    throw std::invalid_argument("invalid SmootherParams");
  }
}

}  // namespace eltanin::planner::detail
