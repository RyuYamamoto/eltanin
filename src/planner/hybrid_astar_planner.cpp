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
#include <eltanin/core/polygon.hpp>
#include <eltanin/map/crop.hpp>
#include <eltanin/planner/dubins_path.hpp>
#include <eltanin/planner/reeds_shepp_path.hpp>

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
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace eltanin::planner
{

namespace
{

constexpr std::uint32_t INVALID_NODE = std::numeric_limits<std::uint32_t>::max();
constexpr std::size_t BITS_PER_WORD = 64;

/// Below this the analytic tail already points where the caller asked, so no turn is emitted.
constexpr double SAME_HEADING = 1e-9;

/// Nodes are appended as they are discovered, so no reservation proportional to the map is needed.
constexpr std::size_t INITIAL_NODE_RESERVE = 4096;

/// g_score and best_node per state; the closed set costs one bit each.
constexpr std::size_t BYTES_PER_STATE = sizeof(float) + sizeof(std::uint32_t);


constexpr std::array<int, 8> NEIGHBOR_DX{1, 1, 0, -1, -1, -1, 0, 1};
constexpr std::array<int, 8> NEIGHBOR_DY{0, 1, 1, 1, 0, -1, -1, -1};

/// One primitive of a control set, in units the map and the turning radius fix at plan time.
struct Motion
{
  /// Travel as a signed fraction of motion_step; 0 turns on the spot.
  double travel_scale;
  /// Curvature as a signed fraction of 1 / minimum_turning_radius while travelling.
  double curvature_scale;
  /// Heading bins turned when travel_scale is 0.
  int spin_bins;
};

/// Every primitive the widest model can use; a narrower one takes a prefix of this order.
constexpr std::size_t MAX_MOTIONS = 8;

using ControlSet = std::array<Motion, MAX_MOTIONS>;

/// Forward arcs first, then reverse, then the turns on the spot; the search never sees the rest.
std::span<const Motion> control_set(const MotionModel & model, ControlSet & storage) noexcept
{
  std::size_t count = 0;
  storage[count++] = Motion{1.0, 0.0, 0};
  storage[count++] = Motion{1.0, -1.0, 0};
  storage[count++] = Motion{1.0, 1.0, 0};
  if (model.reverse) {
    storage[count++] = Motion{-1.0, 0.0, 0};
    storage[count++] = Motion{-1.0, -1.0, 0};
    storage[count++] = Motion{-1.0, 1.0, 0};
  }
  if (model.turn_in_place) {
    storage[count++] = Motion{0.0, 0.0, -1};
    storage[count++] = Motion{0.0, 0.0, 1};
  }
  return std::span<const Motion>{storage.data(), count};
}

/// How many primitives that model uses, without needing the storage for them.
std::size_t control_set_size(const MotionModel & model) noexcept
{
  return 3 + (model.reverse ? 3 : 0) + (model.turn_in_place ? 2 : 0);
}

/// Not an index into any control set, so "no previous primitive" needs no separate flag.
constexpr std::uint8_t START_MODE = 255;

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

/// Exact unicycle integration of one primitive: travel [m] along the body axis while turning `turn`.
Pose2D propagate(const Pose2D & pose, double travel, double turn)
{
  if (travel == 0.0) {
    return Pose2D{pose.position, normalize_angle(pose.yaw + turn)};
  }
  if (turn == 0.0) {
    return Pose2D{
      pose.position + travel * Eigen::Vector2d{std::cos(pose.yaw), std::sin(pose.yaw)}, pose.yaw};
  }

  const double curvature = turn / travel;
  const double next_yaw = normalize_angle(pose.yaw + turn);
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
bool motion_step_is_usable(
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
bool interior_is_free(
  const TraversabilityView & grid, const Polygon2D & footprint, const Pose2D & from, double travel,
  double turn, double collision_check_step)
{
  // Turning on the spot keeps the reference point where it already is, so nothing new to check.
  if (travel == 0.0) {
    return true;
  }
  const int sample_count =
    std::max(1, static_cast<int>(std::ceil(std::abs(travel) / collision_check_step)));
  for (int sample = 1; sample < sample_count; ++sample) {
    const double fraction = static_cast<double>(sample) / static_cast<double>(sample_count);
    if (!pose_is_usable(grid, footprint, propagate(from, travel * fraction, turn * fraction))) {
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

template <class Curve>
bool curve_is_free(
  const TraversabilityView & grid, const Polygon2D & footprint, const Curve & curve,
  double check_step)
{
  const double length = curve.length();
  const int checks = std::max(1, static_cast<int>(std::ceil(length / check_step)));
  for (int i = 1; i <= checks; ++i) {
    const double s = length * static_cast<double>(i) / static_cast<double>(checks);
    if (!pose_is_usable(grid, footprint, curve.sample(s))) {
      return false;
    }
  }
  return true;
}

/// Emits poses over (from, to]; rounding the count keeps each interval near output_step.
template <class Curve>
void append_run(
  std::vector<Pose2D> & samples, const Curve & curve, double from, double to, double output_step)
{
  const double span = to - from;
  // A run of no length would emit a pose on top of the previous one, which is not a primitive.
  if (span <= SAME_HEADING) {
    return;
  }
  const int count = std::max(1, static_cast<int>(std::lround(span / output_step)));
  for (int i = 1; i <= count; ++i) {
    samples.push_back(
      curve.sample(from + span * static_cast<double>(i) / static_cast<double>(count)));
  }
}

/// The tail is charged like the primitives are, so reverse_penalty still applies at the goal.
double penalised_length(
  const ReedsSheppPath & curve, const HybridAStarParams & params, double motion_step)
{
  double cost = 0.0;
  for (const ReedsSheppSegment & segment : curve.segments()) {
    const double span = std::abs(segment.length);
    cost += segment.length < 0.0 ? span * params.reverse_penalty : span;
  }
  return cost + params.direction_change_penalty * motion_step *
                  static_cast<double>(curve.direction_changes());
}

/// Checked at collision_check_step but emitted at output_step, so the output stays evenly spaced.
std::optional<std::vector<Pose2D>> connect_goal(
  const TraversabilityView & grid, const Pose2D & start, const Pose2D & goal,
  const HybridAStarParams & params, double collision_check_step, double output_step)
{
  const auto dubins = solve_dubins_path(start, goal, params.motion_model.minimum_turning_radius);
  const auto forward_tail = [&]() -> std::optional<std::vector<Pose2D>> {
    std::vector<Pose2D> samples;
    if (!dubins.has_value()) {
      return std::nullopt;
    }
    if (dubins->length() == 0.0) {
      return samples;
    }
    if (!curve_is_free(grid, params.common.footprint, *dubins, collision_check_step)) {
      return std::nullopt;
    }
    append_run(samples, *dubins, 0.0, dubins->length(), output_step);
    return samples;
  };
  if (!params.motion_model.reverse) {
    return forward_tail();
  }

  const auto reeds_shepp = solve_reeds_shepp_path(start, goal, params.motion_model.minimum_turning_radius);
  const auto reversing_tail = [&]() -> std::optional<std::vector<Pose2D>> {
    std::vector<Pose2D> samples;
    if (!reeds_shepp.has_value()) {
      return std::nullopt;
    }
    if (reeds_shepp->length() == 0.0) {
      return samples;
    }
    if (!curve_is_free(grid, params.common.footprint, *reeds_shepp, collision_check_step)) {
      return std::nullopt;
    }
    // Each cusp ends a run, so a direction change always lands on a pose the follower can see.
    double run_start = 0.0;
    double consumed = 0.0;
    int direction = 0;
    for (const ReedsSheppSegment & segment : reeds_shepp->segments()) {
      if (segment.length == 0.0) {
        continue;
      }
      const int sign = segment.length > 0.0 ? 1 : -1;
      if (direction != 0 && sign != direction) {
        append_run(samples, *reeds_shepp, run_start, consumed, output_step);
        run_start = consumed;
      }
      direction = sign;
      consumed += std::abs(segment.length);
    }
    append_run(samples, *reeds_shepp, run_start, consumed, output_step);
    return samples;
  };

  const double infinity = std::numeric_limits<double>::infinity();
  const double reversing_cost = reeds_shepp.has_value()
                                  ? penalised_length(*reeds_shepp, params, output_step)
                                  : infinity;
  const double forward_cost = dubins.has_value() ? dubins->length() : infinity;
  if (forward_cost <= reversing_cost) {
    const auto forward = forward_tail();
    return forward.has_value() ? forward : reversing_tail();
  }
  const auto reversing = reversing_tail();
  return reversing.has_value() ? reversing : forward_tail();
}

/// Cost to reach the goal cell over free cells, ignoring the turning radius; infinity when cut off.
std::vector<float> distance_to_goal(
  const TraversabilityView & grid, const map::MapIndex & goal, bool allow_band)
{
  // With an outline the band is a candidate, so connectivity has to be judged over it too.
  const auto passable = [&](int mx, int my) {
    return grid.traversable(mx, my) ||
           (allow_band && grid.geometry().in_bounds(mx, my) &&
            grid.at(mx, my) == Traversability::Circumscribed);
  };
  const map::MapGeometry & geometry = grid.geometry();
  const std::size_t cells = grid.cell_count();
  const auto size_x = static_cast<std::size_t>(geometry.size_x());
  const auto orthogonal = static_cast<float>(geometry.resolution());
  const auto diagonal = static_cast<float>(std::numbers::sqrt2 * geometry.resolution());

  std::vector<float> cost(cells, std::numeric_limits<float>::infinity());
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<>> open;
  const auto goal_cell = static_cast<std::uint32_t>(geometry.index(goal.x, goal.y));
  cost[goal_cell] = 0.0F;
  open.push(OpenEntry{0.0F, goal_cell});

  while (!open.empty()) {
    const OpenEntry top = open.top();
    open.pop();
    if (top.first > cost[top.second]) {
      continue;
    }
    const int mx = static_cast<int>(top.second % size_x);
    const int my = static_cast<int>(top.second / size_x);
    for (std::size_t k = 0; k < NEIGHBOR_DX.size(); ++k) {
      const int nx = mx + NEIGHBOR_DX[k];
      const int ny = my + NEIGHBOR_DY[k];
      if (!passable(nx, ny)) {
        continue;
      }
      const bool is_diagonal = NEIGHBOR_DX[k] != 0 && NEIGHBOR_DY[k] != 0;
      // Corner cutting is forbidden here too, so the two searches agree on what is connected.
      if (
        is_diagonal &&
        (!passable(mx + NEIGHBOR_DX[k], my) || !passable(mx, my + NEIGHBOR_DY[k]))) {
        continue;
      }
      const auto neighbor = static_cast<std::uint32_t>(geometry.index(nx, ny));
      const float tentative = top.first + (is_diagonal ? diagonal : orthogonal);
      if (tentative < cost[neighbor]) {
        cost[neighbor] = tentative;
        open.push(OpenEntry{tentative, neighbor});
      }
    }
  }
  return cost;
}

/// Arrival heading of the shortest forward path to a point: turn on one circle, then go straight.
std::optional<Pose2D> approach_pose(
  const Pose2D & from, const Eigen::Vector2d & goal_position, double turning_radius)
{
  const Eigen::Vector2d forward{std::cos(from.yaw), std::sin(from.yaw)};
  const Eigen::Vector2d left{-forward.y(), forward.x()};
  std::optional<Pose2D> best;
  double best_length = std::numeric_limits<double>::infinity();

  for (const double turn : {1.0, -1.0}) {
    const Eigen::Vector2d centre = from.position + turn * turning_radius * left;
    const Eigen::Vector2d to_goal = goal_position - centre;
    const double distance = to_goal.norm();
    if (distance < turning_radius) {
      continue;
    }
    const double straight =
      std::sqrt(std::max(0.0, distance * distance - turning_radius * turning_radius));
    const double departure = std::atan2(to_goal.y(), to_goal.x()) -
                             turn * std::atan2(straight, turning_radius);
    const Eigen::Vector2d start_angle = from.position - centre;
    const double swept = normalize_angle_positive(
      turn * (departure - std::atan2(start_angle.y(), start_angle.x())));
    const double length = turning_radius * swept + straight;
    if (length < best_length) {
      best_length = length;
      best = Pose2D{goal_position, normalize_angle(departure + turn * 0.5 * std::numbers::pi)};
    }
  }
  return best;
}

/// Driving to `target` plus the on-the-spot turn from its heading to the one the caller asked for.
double arrival_cost(
  const Pose2D & from, const Pose2D & target, double requested_yaw, double turning_radius)
{
  const auto dubins = solve_dubins_path(from, target, turning_radius);
  if (!dubins.has_value()) {
    return std::numeric_limits<double>::infinity();
  }
  const double turn = std::abs(shortest_angular_distance(target.yaw, requested_yaw));
  return dubins->length() + turn * turning_radius;
}

/// Length of a tail, charged for clearance exactly as a primitive would be.
double tail_cost(
  const std::vector<Pose2D> & connection, const Pose2D & from, const ObstacleField & field,
  const ClearanceCost & clearance)
{
  double cost = 0.0;
  Eigen::Vector2d previous = from.position;
  for (const Pose2D & pose : connection) {
    const double step = (pose.position - previous).norm();
    cost += step * (1.0 + clearance_penalty(clearance, field.at(pose.position)));
    previous = pose.position;
  }
  return cost;
}

/// Merges the search poses with the analytic tail, dropping a final segment that came out too short.
Path attach_tail(
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
  // Reaching the requested heading is a turn on the spot, so it gets a pose of its own.
  if (std::abs(shortest_angular_distance(poses.back().yaw, goal.yaw)) > SAME_HEADING) {
    poses.push_back(goal);
  } else {
    poses[poses.size() - 1] = goal;
  }
  return Path{std::move(poses)};
}

PlanResult search(
  const PlanQuery & query, const HybridAStarParams & params, double * shortest_route = nullptr)
{
  const TraversabilityView & grid = query.grid;
  const map::MapGeometry & geometry = grid.geometry();
  const std::size_t cells = grid.cell_count();
  assert(cells > 0);
  assert(pose_is_usable(grid, params.common.footprint, query.start));
  assert(pose_is_usable(grid, params.common.footprint, query.goal));

  const double resolution = geometry.resolution();
  const double bin_width = 2.0 * std::numbers::pi / static_cast<double>(params.heading_bins);
  // Deriving the step from the bin width keeps one turn worth at least one bin at any radius.
  const double motion_step =
    params.motion_step > 0.0
      ? params.motion_step
      : std::max(std::numbers::sqrt2 * resolution, params.motion_model.minimum_turning_radius * bin_width);
  const double collision_check_step =
    params.collision_check_step > 0.0 ? params.collision_check_step : 0.5 * resolution;
  if (!motion_step_is_usable(
        motion_step, resolution, params.motion_model.minimum_turning_radius, params.heading_bins)) {
    return PlanResult{PlannerError::ParamsIncompatibleWithMap};
  }

  const auto heading_bins = static_cast<std::size_t>(params.heading_bins);
  if (cells > std::numeric_limits<std::size_t>::max() / heading_bins) {
    return PlanResult{PlannerError::StateSpaceTooLarge};
  }
  const std::size_t state_count = cells * heading_bins;
  // Node ids and state ids are 32 bit, and the search may append up to 3 nodes per expansion.
  if (state_count > (INVALID_NODE - 1) / (control_set_size(params.motion_model) + 1)) {
    return PlanResult{PlannerError::StateSpaceTooLarge};
  }
  const std::size_t closed_words = state_count / BITS_PER_WORD + 1;
  const std::size_t clearance_bytes =
    params.clearance.penalty > 0.0 ? cells * sizeof(float) : 0;
  const std::size_t state_bytes = state_count * BYTES_PER_STATE +
                                  closed_words * sizeof(std::uint64_t) + cells * sizeof(float) +
                                  clearance_bytes;
  if (state_bytes > params.max_state_memory_bytes) {
    return PlanResult{PlannerError::StateSpaceTooLarge};
  }

  const auto state_index = [&](const map::MapIndex & index, double yaw) {
    const std::size_t cell = geometry.index(index.x, index.y);
    const auto heading = static_cast<std::size_t>(heading_bin(yaw, params.heading_bins));
    return static_cast<std::uint32_t>(cell * heading_bins + heading);
  };
  // Distance over free cells beats the straight line: a wall between the two is what makes a
  // Euclidean heuristic expand the whole room (§13.19).
  const std::vector<float> cost_to_go =
    distance_to_goal(grid, query.goal_index, !params.common.footprint.empty());
  if (shortest_route != nullptr) {
    *shortest_route =
      static_cast<double>(cost_to_go[geometry.index(query.start_index.x, query.start_index.y)]);
  }
  const auto heuristic = [&](const Pose2D & pose) {
    const double straight = (query.goal.position - pose.position).norm();
    const std::optional<map::MapIndex> cell = geometry.world_to_map(pose.position);
    if (!cell.has_value()) {
      return straight;
    }
    const float over_cells = cost_to_go[geometry.index(cell->x, cell->y)];
    return std::max(straight, params.heuristic_weight * static_cast<double>(over_cells));
  };
  // Attempted at every node inside dubins_expansion_distance, and increasingly rarely beyond it.
  const auto attempt_interval = [&](double remaining) -> std::size_t {
    if (remaining <= params.dubins_expansion_distance) {
      return 0;
    }
    const double cells_to_goal = remaining / resolution;
    return static_cast<std::size_t>(cells_to_goal / params.analytic_expansion_ratio);
  };
  // Only paid for when something asks for it; the field is one float per cell.
  std::vector<float> clearance;
  if (params.clearance.penalty > 0.0) {
    clearance = detail::build_obstacle_distance(grid);
  }
  const ObstacleField field{geometry, clearance};
  // Distance to the walls, plus whatever the relaxed pass charges for using the band at all.
  const auto surcharge = [&](const map::MapIndex & index) {
    return clearance_penalty(params.clearance, field.at(index.x, index.y)) +
           grid.surcharge(index.x, index.y);
  };
  ControlSet primitives{};
  const std::span<const Motion> motions = control_set(params.motion_model, primitives);
  const auto travel_of = [&](const Motion & motion) { return motion.travel_scale * motion_step; };
  const auto turn_of = [&](const Motion & motion) {
    return motion.travel_scale == 0.0
             ? static_cast<double>(motion.spin_bins) * bin_width
             : travel_of(motion) * motion.curvature_scale / params.motion_model.minimum_turning_radius;
  };
  const auto transition_cost = [&](std::uint8_t previous_mode, std::uint8_t next_mode) {
    const Motion & next = motions[next_mode];
    // A turn on the spot is charged the arc it would have cost at the minimum turning radius.
    if (next.travel_scale == 0.0) {
      return std::abs(turn_of(next)) * params.motion_model.minimum_turning_radius;
    }
    // Any curvature costs the same, so a gentle primitive is not a cheaper way to travel.
    double multiplier = 1.0;
    if (next.curvature_scale != 0.0) {
      multiplier += params.steering_penalty;
    }
    double gear_change = 0.0;
    if (previous_mode != START_MODE && motions[previous_mode].travel_scale != 0.0) {
      const Motion & previous = motions[previous_mode];
      multiplier += params.steering_change_penalty *
                    std::abs(next.curvature_scale - previous.curvature_scale);
      // Swapping between forward and reverse costs a full stop whatever the geometry says.
      if (previous.travel_scale * next.travel_scale < 0.0) {
        gear_change = params.direction_change_penalty * motion_step;
      }
    }
    if (next.travel_scale < 0.0) {
      multiplier *= params.reverse_penalty;
    }
    return std::abs(travel_of(next)) * multiplier + gear_change;
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
  double best_cost = std::numeric_limits<double>::infinity();
  std::optional<Path> best_path;
  while (!open.empty()) {
    // The heuristic is a lower bound, so nothing left in the queue can beat the tail already found.
    if (best_path.has_value() && open.top().first >= best_cost) {
      break;
    }
    const std::uint32_t current_id = open.top().second;
    open.pop();
    const Node current = nodes[current_id];
    if (best_node[current.state] != current_id || is_closed(closed, current.state)) {
      continue;
    }
    set_closed(closed, current.state);

    // The first node is always tried: on an open map the analytic path from the start is the answer.
    const bool analytic_due =
      expansions == 0 || skipped_attempts >= attempt_interval(heuristic(current.pose));
    if (analytic_due) {
      skipped_attempts = 0;
      std::array<Pose2D, 3> targets{query.goal, query.goal, query.goal};
      std::size_t target_count = 1;
      // Turning on the spot is a primitive here, so arriving off-heading is a legal way to finish.
      if (params.motion_model.turn_in_place) {
        const std::optional<Pose2D> shortest =
          approach_pose(current.pose, query.goal.position, params.motion_model.minimum_turning_radius);
        targets[1] = shortest.value_or(Pose2D{query.goal.position, current.pose.yaw});
        targets[2] = Pose2D{query.goal.position, current.pose.yaw};
        target_count = targets.size();
        // Arriving off-heading costs the follower a turn on the spot, so charge it as arc length.
        std::sort(
          targets.begin(), targets.end(), [&](const Pose2D & lhs, const Pose2D & rhs) {
            return arrival_cost(current.pose, lhs, query.goal.yaw, params.motion_model.minimum_turning_radius) <
                   arrival_cost(current.pose, rhs, query.goal.yaw, params.motion_model.minimum_turning_radius);
          });
      }
      for (std::size_t i = 0; i < target_count; ++i) {
        const auto connection =
          connect_goal(grid, current.pose, targets[i], params, collision_check_step, motion_step);
        if (!connection.has_value()) {
          continue;
        }
        // Scored rather than taken: a tail that hugs a wall must lose to a search that does not.
        const double cost =
          current.g + tail_cost(*connection, current.pose, field, params.clearance);
        if (cost < best_cost) {
          best_cost = cost;
          best_path = attach_tail(
            reconstruct_poses(nodes, current_id), *connection, query.goal, motion_step);
        }
        break;
      }
    } else {
      ++skipped_attempts;
    }
    if (params.max_expansions != 0 && expansions >= params.max_expansions) {
      return best_path.has_value() ? PlanResult{std::move(*best_path)}
                                   : PlanResult{PlannerError::ExpansionLimitReached};
    }
    ++expansions;

    for (std::size_t index = 0; index < motions.size(); ++index) {
      const auto mode = static_cast<std::uint8_t>(index);
      // The cheap state and cost tests run first, so most primitives cost no collision sampling.
      const Pose2D successor =
        propagate(current.pose, travel_of(motions[mode]), turn_of(motions[mode]));
      const auto successor_index = geometry.world_to_map(successor.position);
      if (!successor_index.has_value() || !pose_is_usable(grid, params.common.footprint, successor)) {
        continue;
      }
      // A cell the goal cannot be reached from is not worth a state, let alone an expansion.
      if (!std::isfinite(cost_to_go[geometry.index(successor_index->x, successor_index->y)])) {
        continue;
      }
      const std::uint32_t successor_state = state_index(*successor_index, successor.yaw);
      if (is_closed(closed, successor_state)) {
        continue;
      }
      // Charged per metre travelled, so the distance heuristic stays a lower bound on the cost.
      const auto tentative = static_cast<float>(
        current.g + transition_cost(current.mode, mode) +
        surcharge(*successor_index) * std::abs(travel_of(motions[mode])));
      if (tentative >= g_score[successor_state]) {
        continue;
      }
      if (!interior_is_free(
            grid, params.common.footprint, current.pose, travel_of(motions[mode]),
            turn_of(motions[mode]), collision_check_step)) {
        continue;
      }

      g_score[successor_state] = tentative;
      const auto successor_id = static_cast<std::uint32_t>(nodes.size());
      nodes.push_back(Node{successor, tentative, current_id, successor_state, mode});
      best_node[successor_state] = successor_id;
      open.push(OpenEntry{static_cast<float>(tentative + heuristic(successor)), successor_id});
    }
  }
  if (best_path.has_value()) {
    return PlanResult{std::move(*best_path)};
  }
  return PlanResult{PlannerError::Unreachable};
}

}  // namespace

HybridAStarPlanner::HybridAStarPlanner(const HybridAStarParams & params)
: Planner(params.common), params_(params)
{
  const bool valid =
    params_.heading_bins >= 8 && std::isfinite(params_.motion_model.minimum_turning_radius) &&
    params_.motion_model.minimum_turning_radius > 0.0 && std::isfinite(params_.motion_step) &&
    params_.motion_step >= 0.0 && std::isfinite(params_.collision_check_step) &&
    params_.collision_check_step >= 0.0 && std::isfinite(params_.dubins_expansion_distance) &&
    params_.dubins_expansion_distance > 0.0 && std::isfinite(params_.heuristic_weight) &&
    params_.heuristic_weight >= 0.0 && std::isfinite(params_.analytic_expansion_ratio) &&
    params_.analytic_expansion_ratio > 0.0 && std::isfinite(params_.steering_penalty) &&
    params_.steering_penalty >= 0.0 && std::isfinite(params_.steering_change_penalty) &&
    params_.steering_change_penalty >= 0.0 && std::isfinite(params_.reverse_penalty) &&
    params_.reverse_penalty > 0.0 && std::isfinite(params_.direction_change_penalty) &&
    params_.direction_change_penalty >= 0.0 && params_.max_state_memory_bytes > 0;
  if (!valid) {
    throw std::invalid_argument("invalid HybridAStarParams");
  }
  detail::validate_clearance_cost(params_.clearance);
}

PlanResult HybridAStarPlanner::plan_on_grid(const PlanQuery & query) const
{
  // The state arrays are sized before allocating, but a tight rlimit can still fail the request.
  try {
    double shortest = 0.0;
    PlanResult roomy = search(query, params_, &shortest);
    if (!params_.clearance_fallback.enabled) {
      return roomy;
    }

    HybridAStarParams tight = params_;
    tight.clearance = params_.clearance_fallback.clearance;
    tight.clearance_fallback.enabled = false;
    if (!roomy.has_value()) {
      return search(query, tight);
    }
    // Demanding room is worth a detour, but not a walk around the building.
    const double travelled = path_length(*roomy);
    const double allowed = shortest * (1.0 + params_.clearance_fallback.detour_tolerance);
    if (!std::isfinite(shortest) || shortest <= 0.0 || travelled <= allowed) {
      return roomy;
    }
    PlanResult direct = search(query, tight);
    if (!direct.has_value() || path_length(*direct) >= travelled) {
      return roomy;
    }
    return direct;
  } catch (const std::bad_alloc &) {
    return PlanResult{PlannerError::StateSpaceTooLarge};
  }
}

}  // namespace eltanin::planner
