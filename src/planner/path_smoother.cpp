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

#include <cmath>
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
    // A coincident pair carries the previous tangent forward instead of atan2(0, 0).
    if (has_tangent) {
      path[i].yaw = tangent_yaw;
    }
  }

  if (first_tangent == n) {
    // Every point coincides, so the terminal yaw is the only orientation available.
    for (std::size_t i = 0; i + 1 < n; ++i) {
      path[i].yaw = path[n - 1].yaw;
    }
    return;
  }
  for (std::size_t i = 0; i < first_tangent; ++i) {
    path[i].yaw = path[first_tangent].yaw;
  }
}

Path smooth_on_grid(
  const Path & path, const TraversabilityView & grid, const SmootherParams & params)
{
  if (path.size() <= 1) {
    return path;
  }

  Path smoothed = path;
  const std::size_t n = smoothed.size();
  if (n >= 3) {
    for (int iteration = 0; iteration < params.max_iterations; ++iteration) {
      double displacement = 0.0;
      for (std::size_t i = 1; i + 1 < n; ++i) {
        const Eigen::Vector2d candidate =
          smoothed[i].position + params.weight_data * (path[i].position - smoothed[i].position) +
          params.weight_smooth * (smoothed[i - 1].position + smoothed[i + 1].position -
                                  2.0 * smoothed[i].position);
        // Rejected for this sweep only; a neighbour moving later may make the point movable.
        if (!grid.free(candidate)) {
          continue;
        }
        displacement += (candidate - smoothed[i].position).norm();
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
  const bool valid = std::isfinite(params.weight_data) && params.weight_data >= 0.0 &&
                     std::isfinite(params.weight_smooth) && params.weight_smooth >= 0.0 &&
                     params.weight_data + 4.0 * params.weight_smooth < 2.0 &&
                     std::isfinite(params.tolerance) && params.tolerance >= 0.0 &&
                     params.max_iterations >= 0;
  if (!valid) {
    throw std::invalid_argument("invalid SmootherParams");
  }
}

}  // namespace eltanin::planner::detail
