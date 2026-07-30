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

#include <eltanin/planner/astar_planner.hpp>

#include <eltanin/planner/path_smoother.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <numbers>
#include <queue>
#include <utility>

namespace eltanin::planner::detail
{

namespace
{

constexpr std::uint8_t FREE = static_cast<std::uint8_t>(Traversability::Free);

constexpr std::array<int, 8> NEIGHBOR_DX{1, 1, 0, -1, -1, -1, 0, 1};
constexpr std::array<int, 8> NEIGHBOR_DY{0, 1, 1, 1, 0, -1, -1, -1};

using OpenEntry = std::pair<float, std::size_t>;

/// f ascending, then linear index ascending; the tie-break is what makes the search deterministic.
using OpenQueue = std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<>>;

}  // namespace

std::optional<Path> plan_on_grid(
  const map::MapGeometry & geometry, const TraversabilityGrid & grid, const map::MapIndex & start,
  const map::MapIndex & goal, double goal_yaw)
{
  const std::size_t cells = geometry.cell_count();
  assert(cells > 0);
  assert(grid.size() == cells);
  assert(cells <= static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max()));
  assert(geometry.in_bounds(start.x, start.y));
  assert(geometry.in_bounds(goal.x, goal.y));
  assert(grid[geometry.index(start.x, start.y)] == FREE);
  assert(grid[geometry.index(goal.x, goal.y)] == FREE);

  const std::size_t size_x = static_cast<std::size_t>(geometry.size_x());
  const double resolution = geometry.resolution();
  const auto orthogonal_step = static_cast<float>(resolution);
  const auto diagonal_step = static_cast<float>(std::numbers::sqrt2 * resolution);

  // Octile distance; admissible and consistent for the eight-connected step costs above.
  const auto heuristic = [&](int mx, int my) {
    const double dx = std::abs(static_cast<double>(mx - goal.x));
    const double dy = std::abs(static_cast<double>(my - goal.y));
    return static_cast<float>(
      (dx + dy + (std::numbers::sqrt2 - 2.0) * std::min(dx, dy)) * resolution);
  };
  // in_bounds() is evaluated first so that index() never sees an out-of-range cell.
  const auto free_cell = [&](int mx, int my) {
    return geometry.in_bounds(mx, my) && grid[geometry.index(mx, my)] == FREE;
  };

  std::vector<float> g_score(cells, std::numeric_limits<float>::infinity());
  std::vector<std::int32_t> parent(cells, -1);
  std::vector<std::uint8_t> closed(cells, 0);
  OpenQueue open;

  const std::size_t start_index = geometry.index(start.x, start.y);
  const std::size_t goal_index = geometry.index(goal.x, goal.y);
  g_score[start_index] = 0.0F;
  open.push(OpenEntry{heuristic(start.x, start.y), start_index});

  bool reached = false;
  while (!open.empty()) {
    const std::size_t current = open.top().second;
    open.pop();
    if (closed[current] != 0) {
      continue;
    }
    closed[current] = 1;
    if (current == goal_index) {
      reached = true;
      break;
    }

    const int mx = static_cast<int>(current % size_x);
    const int my = static_cast<int>(current / size_x);
    for (std::size_t k = 0; k < NEIGHBOR_DX.size(); ++k) {
      const int dx = NEIGHBOR_DX[k];
      const int dy = NEIGHBOR_DY[k];
      const int nx = mx + dx;
      const int ny = my + dy;
      if (!geometry.in_bounds(nx, ny)) {
        continue;
      }
      const std::size_t neighbor = geometry.index(nx, ny);
      if (grid[neighbor] != FREE || closed[neighbor] != 0) {
        continue;
      }
      const bool diagonal = dx != 0 && dy != 0;
      // Corner cutting is forbidden: a diagonal step needs both orthogonal cells free.
      if (diagonal && (!free_cell(mx + dx, my) || !free_cell(mx, my + dy))) {
        continue;
      }
      const float tentative = g_score[current] + (diagonal ? diagonal_step : orthogonal_step);
      if (tentative < g_score[neighbor]) {
        g_score[neighbor] = tentative;
        parent[neighbor] = static_cast<std::int32_t>(current);
        open.push(OpenEntry{tentative + heuristic(nx, ny), neighbor});
      }
    }
  }

  if (!reached) {
    return std::nullopt;
  }

  std::vector<std::size_t> reversed;
  for (std::int32_t i = static_cast<std::int32_t>(goal_index); i >= 0;
       i = parent[static_cast<std::size_t>(i)]) {
    reversed.push_back(static_cast<std::size_t>(i));
  }

  Path path;
  for (auto it = reversed.rbegin(); it != reversed.rend(); ++it) {
    const int mx = static_cast<int>(*it % size_x);
    const int my = static_cast<int>(*it / size_x);
    path.push_back(Pose2D{geometry.map_to_world(mx, my), 0.0});
  }
  path[path.size() - 1].yaw = goal_yaw;
  assign_tangent_yaw(path);
  return path;
}

}  // namespace eltanin::planner::detail
