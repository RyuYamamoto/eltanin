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
#include <cassert>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

namespace eltanin::planner
{

namespace
{

constexpr std::array<int, 8> NEIGHBOR_DX{1, 1, 0, -1, -1, -1, 0, 1};
constexpr std::array<int, 8> NEIGHBOR_DY{0, 1, 1, 1, 0, -1, -1, -1};

using OpenEntry = std::pair<float, std::size_t>;

/// f ascending, then linear index ascending; the tie-break is what makes the search deterministic.
using OpenQueue = std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<>>;

}  // namespace

AStarPlanner::AStarPlanner(const AStarParams & params) : Planner(params.common), params_(params)
{
  if (params_.smoother.has_value()) {
    detail::validate_smoother_params(*params_.smoother);
  }
  detail::validate_clearance_cost(params_.clearance);
}

PlanResult AStarPlanner::plan_on_grid(const PlanQuery & query) const
{
  const TraversabilityView & grid = query.grid;
  const map::MapGeometry & geometry = grid.geometry();
  const std::size_t cells = grid.cell_count();
  assert(cells > 0);
  if (cells > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())) {
    return PlanResult{PlannerError::StateSpaceTooLarge};
  }
  assert(grid.traversable(query.start_index.x, query.start_index.y));
  assert(grid.traversable(query.goal_index.x, query.goal_index.y));

  const std::size_t size_x = static_cast<std::size_t>(geometry.size_x());
  const double resolution = geometry.resolution();
  const auto orthogonal_step = static_cast<float>(resolution);
  const auto diagonal_step = static_cast<float>(std::numbers::sqrt2 * resolution);

  // Octile distance; admissible and consistent for the eight-connected step costs above.
  const auto heuristic = [&](int mx, int my) {
    const double dx = std::abs(static_cast<double>(mx - query.goal_index.x));
    const double dy = std::abs(static_cast<double>(my - query.goal_index.y));
    return static_cast<float>(
      (dx + dy + (std::numbers::sqrt2 - 2.0) * std::min(dx, dy)) * resolution);
  };

  // Only paid for when something asks for it; the field is one float per cell.
  std::vector<float> clearance;
  if (params_.clearance.penalty > 0.0) {
    clearance = detail::build_obstacle_distance(grid);
  }
  const ObstacleField field{geometry, clearance};
  const auto surcharge = [&](int mx, int my) {
    return static_cast<float>(clearance_penalty(params_.clearance, field.at(mx, my)));
  };

  std::vector<float> g_score(cells, std::numeric_limits<float>::infinity());
  std::vector<std::int32_t> parent(cells, -1);
  std::vector<std::uint8_t> closed(cells, 0);
  OpenQueue open;

  const std::size_t start_index = geometry.index(query.start_index.x, query.start_index.y);
  const std::size_t goal_index = geometry.index(query.goal_index.x, query.goal_index.y);
  g_score[start_index] = 0.0F;
  open.push(OpenEntry{heuristic(query.start_index.x, query.start_index.y), start_index});

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
      if (!grid.traversable(nx, ny)) {
        continue;
      }
      const std::size_t neighbor = geometry.index(nx, ny);
      if (closed[neighbor] != 0) {
        continue;
      }
      const bool diagonal = dx != 0 && dy != 0;
      // Corner cutting is forbidden: a diagonal step needs both orthogonal cells free.
      if (diagonal && (!grid.traversable(mx + dx, my) || !grid.traversable(mx, my + dy))) {
        continue;
      }
      // Charging clearance per metre travelled keeps the octile heuristic a lower bound.
      const float step = diagonal ? diagonal_step : orthogonal_step;
      const float tentative =
        g_score[current] +
        step * (1.0F + surcharge(nx, ny) + static_cast<float>(grid.surcharge(nx, ny)));
      if (tentative < g_score[neighbor]) {
        g_score[neighbor] = tentative;
        parent[neighbor] = static_cast<std::int32_t>(current);
        open.push(OpenEntry{tentative + heuristic(nx, ny), neighbor});
      }
    }
  }

  if (!reached) {
    return PlanResult{PlannerError::Unreachable};
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
  path[path.size() - 1].yaw = query.goal.yaw;
  detail::assign_tangent_yaw(path);
  if (!params_.smoother.has_value()) {
    return PlanResult{std::move(path)};
  }

  // A window around the raw path, so the obstacle term does not pay for the whole map.
  const double reach = detail::smoother_reach(*params_.smoother);
  std::optional<ObstacleWindow> window;
  if (reach > 0.0) {
    std::vector<Eigen::Vector2d> positions;
    positions.reserve(path.size());
    for (const Pose2D & pose : path) {
      positions.push_back(pose.position);
    }
    window = detail::build_obstacle_window(grid, positions, reach);
  }
  const ObstacleField smoothing_field =
    window.has_value() ? ObstacleField{*window} : ObstacleField{};
  return PlanResult{detail::smooth_on_grid(path, grid, smoothing_field, *params_.smoother)};
}

}  // namespace eltanin::planner
