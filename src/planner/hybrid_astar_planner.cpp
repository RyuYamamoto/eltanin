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
#include <new>
#include <numbers>
#include <optional>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace eltanin::planner
{

namespace
{

constexpr std::uint32_t INVALID_NODE = std::numeric_limits<std::uint32_t>::max();
constexpr std::size_t BITS_PER_WORD = 64;

/// Nodes are appended as they are discovered, so no reservation proportional to the map is needed.
constexpr std::size_t INITIAL_NODE_RESERVE = 4096;

/// g_score and best_node per state; the closed set costs one bit each.
constexpr std::size_t STATE_ARRAY_BYTES_PER_STATE = sizeof(float) + sizeof(std::uint32_t);

struct Motion
{
  /// Curvature as a signed fraction of 1 / minimum_turning_radius.
  double curvature_scale;
};

/// The half-curvature pair exists so that a heading can be nudged by about one bin.
constexpr std::array<Motion, 5> MOTIONS{
  Motion{0.0}, Motion{-1.0}, Motion{1.0}, Motion{-0.5}, Motion{0.5}};
constexpr std::uint8_t START_MODE = static_cast<std::uint8_t>(MOTIONS.size());

struct Node
{
  Pose2D pose;
  float g;
  std::uint32_t parent;
  std::uint32_t state;
  std::uint8_t mode;
};

using OpenEntry = std::pair<float, std::uint32_t>;

/// f ascending, then node id ascending; the tie-break is what makes the search deterministic.
using OpenQueue = std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<>>;

bool is_closed(const std::vector<std::uint64_t> & closed, std::uint32_t state) noexcept
{
  return ((closed[state / BITS_PER_WORD] >> (state % BITS_PER_WORD)) & std::uint64_t{1}) != 0;
}

void set_closed(std::vector<std::uint64_t> & closed, std::uint32_t state) noexcept
{
  closed[state / BITS_PER_WORD] |= std::uint64_t{1} << (state % BITS_PER_WORD);
}

Pose2D propagate(const Pose2D & pose, double distance, double curvature)
{
  if (curvature == 0.0) {
    return Pose2D{
      pose.position + distance * Eigen::Vector2d{std::cos(pose.yaw), std::sin(pose.yaw)}, pose.yaw};
  }

  const double next_yaw = normalize_angle(pose.yaw + distance * curvature);
  return Pose2D{
    pose.position + Eigen::Vector2d{
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

/// Every primitive must leave the current cell, and a turn must also leave the cell or the bin.
bool primitives_can_change_state(
  double motion_step, double resolution, double turning_radius, int heading_bins)
{
  const double cell_diagonal = std::numbers::sqrt2 * resolution;
  if (motion_step < cell_diagonal) {
    return false;
  }
  const double bin_width = 2.0 * std::numbers::pi / static_cast<double>(heading_bins);
  const double chord = 2.0 * turning_radius * std::sin(0.5 * motion_step / turning_radius);
  return motion_step / turning_radius >= bin_width || chord >= cell_diagonal;
}

/// Only the interior of the primitive; the caller has already accepted the end pose.
bool primitive_interior_is_free(
  const TraversabilityView & grid, const Pose2D & from, const Motion & motion, double motion_step,
  double turning_radius, double collision_check_step)
{
  const int sample_count =
    std::max(1, static_cast<int>(std::ceil(motion_step / collision_check_step)));
  const double curvature = motion.curvature_scale / turning_radius;
  for (int sample = 1; sample < sample_count; ++sample) {
    const double distance =
      motion_step * static_cast<double>(sample) / static_cast<double>(sample_count);
    if (!grid.free(propagate(from, distance, curvature).position)) {
      return false;
    }
  }
  return true;
}

std::vector<Pose2D> reconstruct_poses(const std::vector<Node> & nodes, std::uint32_t goal_node)
{
  std::vector<Pose2D> poses;
  for (std::uint32_t node = goal_node; node != INVALID_NODE; node = nodes[node].parent) {
    poses.push_back(nodes[node].pose);
  }
  std::reverse(poses.begin(), poses.end());
  return poses;
}

/// Checked at collision_check_step but emitted at output_step, so the output stays evenly spaced.
std::optional<std::vector<Pose2D>> connect_goal(
  const TraversabilityView & grid, const Pose2D & start, const Pose2D & goal,
  double turning_radius, double collision_check_step, double output_step)
{
  const auto dubins = solve_dubins_path(start, goal, turning_radius);
  if (!dubins.has_value()) {
    return std::nullopt;
  }

  std::vector<Pose2D> samples;
  const double length = dubins->length();
  if (length == 0.0) {
    return samples;
  }

  const int checks = std::max(1, static_cast<int>(std::ceil(length / collision_check_step)));
  for (int i = 1; i <= checks; ++i) {
    const double s = length * static_cast<double>(i) / static_cast<double>(checks);
    if (!grid.free(dubins->sample(s).position)) {
      return std::nullopt;
    }
  }

  // Rounding rather than ceiling keeps every emitted interval inside [0.75, 1.0] * output_step.
  const int count = std::max(1, static_cast<int>(std::lround(length / output_step)));
  samples.reserve(static_cast<std::size_t>(count));
  for (int i = 1; i <= count; ++i) {
    const double s = length * static_cast<double>(i) / static_cast<double>(count);
    samples.push_back(dubins->sample(s));
  }
  return samples;
}

/// Merges the search poses with the analytic tail, dropping a final segment that came out too short.
Path join_with_connection(
  std::vector<Pose2D> poses, const std::vector<Pose2D> & connection, const Pose2D & goal,
  double motion_step)
{
  if (connection.empty()) {
    poses[poses.size() - 1] = goal;
    return Path{std::move(poses)};
  }
  const double seam = (connection.front().position - poses.back().position).norm();
  if (connection.size() == 1 && poses.size() >= 2 && seam < 0.5 * motion_step) {
    poses.pop_back();
  }
  poses.insert(poses.end(), connection.begin(), connection.end());
  return Path{std::move(poses)};
}

PlanResult search(const PlanQuery & query, const HybridAStarParams & params)
{
  const TraversabilityView & grid = query.grid;
  const map::MapGeometry & geometry = grid.geometry();
  const std::size_t cells = grid.cell_count();
  assert(cells > 0);
  assert(grid.free(query.start_index.x, query.start_index.y));
  assert(grid.free(query.goal_index.x, query.goal_index.y));

  const double resolution = geometry.resolution();
  const double motion_step =
    params.motion_step > 0.0 ? params.motion_step : std::numbers::sqrt2 * resolution;
  const double collision_check_step =
    params.collision_check_step > 0.0 ? params.collision_check_step : 0.5 * resolution;
  if (!primitives_can_change_state(
        motion_step, resolution, params.minimum_turning_radius, params.heading_bins)) {
    return PlanResult{PlannerError::ParamsIncompatibleWithMap};
  }

  const auto heading_bins = static_cast<std::size_t>(params.heading_bins);
  constexpr std::size_t MAX_SIZE = std::numeric_limits<std::size_t>::max();
  if (cells > MAX_SIZE / heading_bins) {
    return PlanResult{PlannerError::StateSpaceTooLarge};
  }
  const std::size_t state_count = cells * heading_bins;
  // Node ids and state ids are 32 bit, and the search may append up to 3 nodes per expansion.
  if (state_count > (INVALID_NODE - 1) / (MOTIONS.size() + 1)) {
    return PlanResult{PlannerError::StateSpaceTooLarge};
  }
  const std::size_t closed_words = state_count / BITS_PER_WORD + 1;
  const std::size_t state_bytes =
    state_count * STATE_ARRAY_BYTES_PER_STATE + closed_words * sizeof(std::uint64_t);
  if (state_bytes > params.max_state_memory_bytes) {
    return PlanResult{PlannerError::StateSpaceTooLarge};
  }

  const auto state_index = [&](const map::MapIndex & index, double yaw) {
    const std::size_t cell = geometry.index(index.x, index.y);
    const auto heading = static_cast<std::size_t>(heading_bin(yaw, params.heading_bins));
    return static_cast<std::uint32_t>(cell * heading_bins + heading);
  };
  const auto heuristic = [&](const Pose2D & pose) {
    return (query.goal.position - pose.position).norm();
  };
  // Attempted at every node inside dubins_expansion_distance, and increasingly rarely beyond it.
  const auto analytic_expansion_interval = [&](double remaining) -> std::size_t {
    if (remaining <= params.dubins_expansion_distance) {
      return 0;
    }
    const double cells_to_goal = remaining / resolution;
    return static_cast<std::size_t>(cells_to_goal / params.analytic_expansion_ratio);
  };
  const auto transition_cost = [&](std::uint8_t previous_mode, std::uint8_t next_mode) {
    const Motion & next = MOTIONS[next_mode];
    // Any curvature costs the same, so a gentle primitive is not a cheaper way to travel.
    double multiplier = 1.0;
    if (next.curvature_scale != 0.0) {
      multiplier += params.steering_penalty;
    }
    if (previous_mode != START_MODE) {
      multiplier += params.steering_change_penalty *
                    std::abs(next.curvature_scale - MOTIONS[previous_mode].curvature_scale);
    }
    return motion_step * multiplier;
  };

  std::vector<float> g_score(state_count, std::numeric_limits<float>::infinity());
  std::vector<std::uint32_t> best_node(state_count, INVALID_NODE);
  std::vector<std::uint64_t> closed(closed_words, 0);
  std::vector<Node> nodes;
  nodes.reserve(INITIAL_NODE_RESERVE);
  OpenQueue open;

  const std::uint32_t start_state = state_index(query.start_index, query.start.yaw);
  nodes.push_back(Node{query.start, 0.0F, INVALID_NODE, start_state, START_MODE});
  g_score[start_state] = 0.0F;
  best_node[start_state] = 0;
  open.push(OpenEntry{static_cast<float>(heuristic(query.start)), 0});

  std::size_t expansions = 0;
  std::size_t skipped_attempts = 0;
  while (!open.empty()) {
    const std::uint32_t current_id = open.top().second;
    open.pop();
    const Node current = nodes[current_id];
    if (best_node[current.state] != current_id || is_closed(closed, current.state)) {
      continue;
    }
    set_closed(closed, current.state);

    // The first node is always tried: on an open map the analytic path from the start is the answer.
    const bool analytic_due =
      expansions == 0 || skipped_attempts >= analytic_expansion_interval(heuristic(current.pose));
    if (analytic_due) {
      skipped_attempts = 0;
      const auto connection = connect_goal(
        grid, current.pose, query.goal, params.minimum_turning_radius, collision_check_step,
        motion_step);
      if (connection.has_value()) {
        return PlanResult{join_with_connection(
          reconstruct_poses(nodes, current_id), *connection, query.goal, motion_step)};
      }
    } else {
      ++skipped_attempts;
    }
    if (params.max_expansions != 0 && expansions >= params.max_expansions) {
      return PlanResult{PlannerError::ExpansionLimitReached};
    }
    ++expansions;

    for (std::size_t index = 0; index < MOTIONS.size(); ++index) {
      const auto mode = static_cast<std::uint8_t>(index);
      // The cheap state and cost tests run first, so most primitives cost no collision sampling.
      const Pose2D successor = propagate(
        current.pose, motion_step, MOTIONS[mode].curvature_scale / params.minimum_turning_radius);
      const auto successor_index = geometry.world_to_map(successor.position);
      if (!successor_index.has_value() || !grid.free(successor.position)) {
        continue;
      }
      const std::uint32_t successor_state = state_index(*successor_index, successor.yaw);
      if (is_closed(closed, successor_state)) {
        continue;
      }
      const auto tentative = static_cast<float>(current.g + transition_cost(current.mode, mode));
      if (tentative >= g_score[successor_state]) {
        continue;
      }
      if (!primitive_interior_is_free(
            grid, current.pose, MOTIONS[mode], motion_step, params.minimum_turning_radius,
            collision_check_step)) {
        continue;
      }

      g_score[successor_state] = tentative;
      const auto successor_id = static_cast<std::uint32_t>(nodes.size());
      nodes.push_back(Node{successor, tentative, current_id, successor_state, mode});
      best_node[successor_state] = successor_id;
      open.push(OpenEntry{static_cast<float>(tentative + heuristic(successor)), successor_id});
    }
  }
  return PlanResult{PlannerError::Unreachable};
}

}  // namespace

HybridAStarPlanner::HybridAStarPlanner(const HybridAStarParams & params)
: Planner(params.common), params_(params)
{
  const bool valid =
    params_.heading_bins >= 8 && std::isfinite(params_.minimum_turning_radius) &&
    params_.minimum_turning_radius > 0.0 && std::isfinite(params_.motion_step) &&
    params_.motion_step >= 0.0 && std::isfinite(params_.collision_check_step) &&
    params_.collision_check_step >= 0.0 && std::isfinite(params_.dubins_expansion_distance) &&
    params_.dubins_expansion_distance > 0.0 && std::isfinite(params_.analytic_expansion_ratio) &&
    params_.analytic_expansion_ratio > 0.0 && std::isfinite(params_.steering_penalty) &&
    params_.steering_penalty >= 0.0 && std::isfinite(params_.steering_change_penalty) &&
    params_.steering_change_penalty >= 0.0 && params_.max_state_memory_bytes > 0;
  if (!valid) {
    throw std::invalid_argument("invalid HybridAStarParams");
  }
}

PlanResult HybridAStarPlanner::plan_on_grid(const PlanQuery & query) const
{
  // The state arrays are sized before allocating, but a tight rlimit can still fail the request.
  try {
    return search(query, params_);
  } catch (const std::bad_alloc &) {
    return PlanResult{PlannerError::StateSpaceTooLarge};
  }
}

}  // namespace eltanin::planner
