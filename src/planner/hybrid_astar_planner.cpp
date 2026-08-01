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

#include <eltanin/planner/hybrid_astar_planner.hpp>

#include <eltanin/core/angle.hpp>
#include <eltanin/planner/dubins_path.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <numbers>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace eltanin::planner
{

namespace
{

constexpr std::uint8_t FREE = static_cast<std::uint8_t>(Traversability::Free);
constexpr std::size_t INVALID_NODE = std::numeric_limits<std::size_t>::max();

struct Motion
{
  int steering;
};

constexpr std::array<Motion, 3> MOTIONS{Motion{0}, Motion{-1}, Motion{1}};
constexpr std::size_t START_MODE = MOTIONS.size();
constexpr std::size_t MODE_COUNT = MOTIONS.size() + 1;

struct Node
{
  Pose2D pose;
  double g;
  std::size_t parent;
  std::size_t state;
  std::size_t mode;
};

using OpenEntry = std::pair<double, std::size_t>;
using OpenQueue = std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<>>;

Pose2D propagate(const Pose2D & pose, double distance, double curvature)
{
  if (curvature == 0.0) {
    return Pose2D{
      pose.position +
        distance * Eigen::Vector2d{std::cos(pose.yaw), std::sin(pose.yaw)},
      pose.yaw};
  }

  const double next_yaw = normalize_angle(pose.yaw + distance * curvature);
  return Pose2D{
    pose.position +
      Eigen::Vector2d{
        (std::sin(next_yaw) - std::sin(pose.yaw)) / curvature,
        (-std::cos(next_yaw) + std::cos(pose.yaw)) / curvature},
    next_yaw};
}

int heading_bin(double yaw, int bins)
{
  const double bin_width = 2.0 * std::numbers::pi / static_cast<double>(bins);
  const auto rounded = static_cast<int>(std::floor(normalize_angle_positive(yaw) / bin_width + 0.5));
  return rounded % bins;
}

std::optional<Pose2D> collision_free_successor(
  const map::MapGeometry & geometry, const detail::TraversabilityGrid & grid,
  const Pose2D & from, const Motion & motion, double motion_step, double turning_radius,
  double collision_check_step)
{
  const int sample_count =
    std::max(1, static_cast<int>(std::ceil(motion_step / collision_check_step)));
  const double curvature = static_cast<double>(motion.steering) / turning_radius;
  Pose2D candidate = from;
  for (int sample = 1; sample <= sample_count; ++sample) {
    const double distance = motion_step * static_cast<double>(sample) /
                            static_cast<double>(sample_count);
    candidate = propagate(from, distance, curvature);
    const auto index = geometry.world_to_map(candidate.position);
    if (!index.has_value() || grid[geometry.index(index->x, index->y)] != FREE) {
      return std::nullopt;
    }
  }
  return candidate;
}

Path reconstruct_path(const std::vector<Node> & nodes, std::size_t goal_node)
{
  Path path;
  for (std::size_t node = goal_node; node != INVALID_NODE; node = nodes[node].parent) {
    path.push_back(nodes[node].pose);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::optional<Path> connect_goal(
  const map::MapGeometry & geometry, const detail::TraversabilityGrid & grid,
  const Pose2D & start, const Pose2D & goal, double turning_radius, double sample_step)
{
  const auto dubins = solve_dubins_path(start, goal, turning_radius);
  if (!dubins.has_value()) {
    return std::nullopt;
  }

  Path samples;
  if (dubins->length() == 0.0) {
    return samples;
  }
  const int count = std::max(1, static_cast<int>(std::ceil(dubins->length() / sample_step)));
  for (int i = 1; i <= count; ++i) {
    const double s = dubins->length() * static_cast<double>(i) / static_cast<double>(count);
    const Pose2D pose = dubins->sample(s);
    const auto index = geometry.world_to_map(pose.position);
    if (!index.has_value() || grid[geometry.index(index->x, index->y)] != FREE) {
      return std::nullopt;
    }
    samples.push_back(pose);
  }
  return samples;
}

}  // namespace

HybridAStarPlanner::HybridAStarPlanner(const HybridAStarParams & params)
: Planner(params.start_search_radius_cells), params_(params)
{
  const bool valid = params_.heading_bins >= 8 &&
                     std::isfinite(params_.minimum_turning_radius) &&
                     params_.minimum_turning_radius > 0.0 && std::isfinite(params_.motion_step) &&
                     params_.motion_step >= 0.0 &&
                     std::isfinite(params_.collision_check_step) &&
                     params_.collision_check_step >= 0.0 &&
                     std::isfinite(params_.dubins_expansion_distance) &&
                     params_.dubins_expansion_distance > 0.0 &&
                     std::isfinite(params_.steering_penalty) && params_.steering_penalty >= 0.0 &&
                     std::isfinite(params_.steering_change_penalty) &&
                     params_.steering_change_penalty >= 0.0;
  if (!valid) {
    throw std::invalid_argument("invalid HybridAStarParams");
  }
}

std::optional<Path> HybridAStarPlanner::plan_on_grid(
  const map::MapGeometry & geometry, const detail::TraversabilityGrid & grid,
  const map::MapIndex & start_index, const map::MapIndex & goal_index,
  const Pose2D & effective_start, const Pose2D & goal) const
{
  const std::size_t cells = geometry.cell_count();
  assert(cells > 0);
  assert(grid.size() == cells);
  assert(geometry.in_bounds(start_index.x, start_index.y));
  assert(geometry.in_bounds(goal_index.x, goal_index.y));
  assert(grid[geometry.index(start_index.x, start_index.y)] == FREE);
  assert(grid[geometry.index(goal_index.x, goal_index.y)] == FREE);
  if (!std::isfinite(effective_start.yaw) || !std::isfinite(goal.yaw)) {
    return std::nullopt;
  }

  const double motion_step = params_.motion_step > 0.0 ? params_.motion_step : geometry.resolution();
  const double collision_check_step =
    params_.collision_check_step > 0.0 ? params_.collision_check_step : 0.5 * geometry.resolution();
  const std::size_t heading_bins = static_cast<std::size_t>(params_.heading_bins);
  const std::size_t max_size = std::numeric_limits<std::size_t>::max();
  if (heading_bins > max_size / MODE_COUNT || cells > max_size / (heading_bins * MODE_COUNT)) {
    return std::nullopt;
  }
  const std::size_t state_count = cells * heading_bins * MODE_COUNT;

  const auto state_index = [&](const map::MapIndex & index, double yaw, std::size_t mode) {
    const std::size_t cell = geometry.index(index.x, index.y);
    const std::size_t heading = static_cast<std::size_t>(heading_bin(yaw, params_.heading_bins));
    return (cell * heading_bins + heading) * MODE_COUNT + mode;
  };
  const auto heuristic = [&](const Pose2D & pose) {
    return (goal.position - pose.position).norm();
  };
  const auto transition_cost = [&](std::size_t previous_mode, std::size_t next_mode) {
    const Motion & next = MOTIONS[next_mode];
    double multiplier = 1.0;
    if (next.steering != 0) {
      multiplier += params_.steering_penalty;
    }
    if (previous_mode != START_MODE) {
      const Motion & previous = MOTIONS[previous_mode];
      if (previous.steering != next.steering) {
        multiplier += params_.steering_change_penalty;
      }
    }
    return motion_step * multiplier;
  };

  std::vector<double> g_score(state_count, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> best_node(state_count, INVALID_NODE);
  std::vector<std::uint8_t> closed(state_count, 0);
  std::vector<Node> nodes;
  nodes.reserve(std::min(state_count, cells * 8));
  OpenQueue open;

  const std::size_t start_state = state_index(start_index, effective_start.yaw, START_MODE);
  nodes.push_back(Node{effective_start, 0.0, INVALID_NODE, start_state, START_MODE});
  g_score[start_state] = 0.0;
  best_node[start_state] = 0;
  open.push(OpenEntry{heuristic(effective_start), 0});

  std::size_t expansions = 0;
  while (!open.empty()) {
    const std::size_t current_id = open.top().second;
    open.pop();
    const Node current = nodes[current_id];
    if (best_node[current.state] != current_id || closed[current.state] != 0) {
      continue;
    }
    closed[current.state] = 1;

    if ((goal.position - current.pose.position).norm() <= params_.dubins_expansion_distance) {
      const auto connection = connect_goal(
        geometry, grid, current.pose, goal, params_.minimum_turning_radius,
        collision_check_step);
      if (connection.has_value()) {
        Path path = reconstruct_path(nodes, current_id);
        for (const Pose2D & pose : *connection) {
          path.push_back(pose);
        }
        if (connection->empty()) {
          path[path.size() - 1] = goal;
        }
        return path;
      }
    }
    if (params_.max_expansions != 0 && expansions >= params_.max_expansions) {
      return std::nullopt;
    }
    ++expansions;

    for (std::size_t mode = 0; mode < MOTIONS.size(); ++mode) {
      const auto successor = collision_free_successor(
        geometry, grid, current.pose, MOTIONS[mode], motion_step,
        params_.minimum_turning_radius, collision_check_step);
      if (!successor.has_value()) {
        continue;
      }
      const auto successor_index = geometry.world_to_map(successor->position);
      assert(successor_index.has_value());
      const std::size_t successor_state = state_index(*successor_index, successor->yaw, mode);
      if (closed[successor_state] != 0) {
        continue;
      }

      const double tentative = current.g + transition_cost(current.mode, mode);
      if (tentative >= g_score[successor_state]) {
        continue;
      }
      g_score[successor_state] = tentative;
      const std::size_t successor_id = nodes.size();
      nodes.push_back(Node{*successor, tentative, current_id, successor_state, mode});
      best_node[successor_state] = successor_id;
      open.push(OpenEntry{tentative + heuristic(*successor), successor_id});
    }
  }
  return std::nullopt;
}

}  // namespace eltanin::planner
