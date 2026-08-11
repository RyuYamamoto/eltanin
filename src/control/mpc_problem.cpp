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

#include <eltanin/control/mpc_problem.hpp>

#include <eltanin/core/angle.hpp>
#include <eltanin/core/differential_drive.hpp>
#include <eltanin/core/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace eltanin::control::detail
{

namespace
{

/// State weight rotated into the reference heading, so w_lat really is the across-track weight.
struct StateWeight
{
  double xx{0.0};
  double xy{0.0};
  double yy{0.0};
  double yaw{0.0};
};

StateWeight weight_at(const MpcFollowerParams & params, double yaw, double scale)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double along = scale * params.weight_longitudinal;
  const double across = scale * params.weight_lateral;
  return StateWeight{
    along * c * c + across * s * s, (along - across) * c * s, along * s * s + across * c * c,
    scale * params.weight_yaw};
}

}  // namespace

Pose2D pose_at_arc(const Path & path, std::span<const double> arc_lengths, double arc)
{
  if (path.empty()) {
    return Pose2D{};
  }
  if (!(arc > arc_lengths.front())) {
    return path[0];
  }
  if (arc >= arc_lengths.back()) {
    return path[path.size() - 1];
  }
  const auto upper = std::lower_bound(arc_lengths.begin(), arc_lengths.end(), arc);
  const std::size_t high = static_cast<std::size_t>(upper - arc_lengths.begin());
  const std::size_t low = high - 1;
  const double span = arc_lengths[high] - arc_lengths[low];
  const double ratio = span > 0.0 ? (arc - arc_lengths[low]) / span : 0.0;
  return interpolate_pose(path[low], path[high], ratio);
}

PathProjection project_on_path(
  const Path & path, std::span<const double> arc_lengths, const Eigen::Vector2d & position,
  std::size_t from, std::size_t to)
{
  PathProjection best{std::min(from, path.size() - 1), arc_lengths[std::min(from, path.size() - 1)]};
  if (path.size() < 2) {
    return best;
  }

  double best_distance = std::numeric_limits<double>::infinity();
  const std::size_t first = std::min(from, path.size() - 2);
  const std::size_t last = std::min(to, path.size() - 1);
  for (std::size_t i = first; i + 1 <= last; ++i) {
    const Eigen::Vector2d & a = path[i].position;
    const Eigen::Vector2d & b = path[i + 1].position;
    const double distance = distance_to_segment(position, a, b);
    if (distance >= best_distance) {
      continue;
    }
    best_distance = distance;
    const Eigen::Vector2d segment = b - a;
    const double squared = segment.squaredNorm();
    const double ratio =
      squared > 0.0 ? std::clamp((position - a).dot(segment) / squared, 0.0, 1.0) : 0.0;
    best.index = i;
    best.arc = arc_lengths[i] + ratio * (arc_lengths[i + 1] - arc_lengths[i]);
  }
  return best;
}

void MpcReference::resize(int horizon)
{
  const auto steps = static_cast<std::size_t>(horizon);
  states.assign(steps + 1, Pose2D{});
  arc.assign(steps + 1, 0.0);
  linear_vel.assign(steps, 0.0);
  angular_vel.assign(steps, 0.0);
}

void build_reference(
  const Path & path, std::span<const double> arc_lengths, const PathProjection & start,
  double prediction_dt, double run_speed, const VelocityProfile * profile, const ReferenceRun & run,
  MpcReference & reference)
{
  const std::size_t steps = reference.linear_vel.size();
  const double ceiling = std::max(0.0, run_speed);
  const double stop_arc = std::min(arc_lengths.back(), run.end_arc);

  double arc = start.arc;
  for (std::size_t k = 0; k <= steps; ++k) {
    reference.arc[k] = arc;
    reference.states[k] = pose_at_arc(path, arc_lengths, arc);
    if (k == steps) {
      break;
    }
    const double bound = profile != nullptr ? profile->at_arc(arc) : ceiling;
    // The arc is unsigned, so the run direction only decides the sign the follower is asked for.
    const double magnitude = arc >= stop_arc ? 0.0 : std::min(ceiling, bound);
    reference.linear_vel[k] = run.direction == Direction::Reverse ? -magnitude : magnitude;
    arc += magnitude * prediction_dt;
  }

  for (std::size_t k = 0; k < steps; ++k) {
    const double turn =
      shortest_angular_distance(reference.states[k].yaw, reference.states[k + 1].yaw);
    reference.angular_vel[k] =
      std::abs(reference.linear_vel[k]) > 0.0 ? turn / prediction_dt : 0.0;
  }
}

MpcProblem::MpcProblem(const MpcFollowerParams & params)
: params_(params), horizon_(params.prediction_horizon)
{
  build_structure();
  p_values_.assign(structure_.p_nonzeros(), 0.0);
  a_values_.assign(structure_.a_nonzeros(), 0.0);
  q_.assign(static_cast<std::size_t>(structure_.variables), 0.0);
  lower_.assign(static_cast<std::size_t>(structure_.constraints), 0.0);
  upper_.assign(static_cast<std::size_t>(structure_.constraints), 0.0);
}

void MpcProblem::build_structure()
{
  const int n = horizon_;
  const int states = 3 * (n + 1);
  structure_.variables = states + 2 * n;
  structure_.constraints = 3 + 3 * n + 2 * n + 2 * n;

  structure_.p_indptr.assign(1, 0);
  structure_.p_indices.clear();
  for (int k = 0; k <= n; ++k) {
    for (int i = 0; i < 3; ++i) {
      if (k > 0) {
        if (i == 0) {
          structure_.p_indices.push_back(3 * k);
        } else if (i == 1) {
          structure_.p_indices.push_back(3 * k);
          structure_.p_indices.push_back(3 * k + 1);
        } else {
          structure_.p_indices.push_back(3 * k + 2);
        }
      }
      structure_.p_indptr.push_back(static_cast<int>(structure_.p_indices.size()));
    }
  }
  for (int j = 0; j < n; ++j) {
    for (int c = 0; c < 2; ++c) {
      if (j > 0) {
        structure_.p_indices.push_back(states + 2 * (j - 1) + c);
      }
      structure_.p_indices.push_back(states + 2 * j + c);
      structure_.p_indptr.push_back(static_cast<int>(structure_.p_indices.size()));
    }
  }

  const int dynamics_row = 3;
  const int box_row = 3 + 3 * n;
  const int rate_row = box_row + 2 * n;

  structure_.a_indptr.assign(1, 0);
  structure_.a_indices.clear();
  for (int k = 0; k <= n; ++k) {
    for (int i = 0; i < 3; ++i) {
      if (k == 0) {
        structure_.a_indices.push_back(i);
      } else {
        structure_.a_indices.push_back(dynamics_row + 3 * (k - 1) + i);
      }
      if (k < n) {
        if (i == 2) {
          structure_.a_indices.push_back(dynamics_row + 3 * k);
          structure_.a_indices.push_back(dynamics_row + 3 * k + 1);
          structure_.a_indices.push_back(dynamics_row + 3 * k + 2);
        } else {
          structure_.a_indices.push_back(dynamics_row + 3 * k + i);
        }
      }
      structure_.a_indptr.push_back(static_cast<int>(structure_.a_indices.size()));
    }
  }
  for (int j = 0; j < n; ++j) {
    for (int c = 0; c < 2; ++c) {
      if (c == 0) {
        structure_.a_indices.push_back(dynamics_row + 3 * j);
        structure_.a_indices.push_back(dynamics_row + 3 * j + 1);
      } else {
        structure_.a_indices.push_back(dynamics_row + 3 * j + 2);
      }
      structure_.a_indices.push_back(box_row + 2 * j + c);
      structure_.a_indices.push_back(rate_row + 2 * j + c);
      if (j + 1 < n) {
        structure_.a_indices.push_back(rate_row + 2 * (j + 1) + c);
      }
      structure_.a_indptr.push_back(static_cast<int>(structure_.a_indices.size()));
    }
  }
}

std::pair<double, double> MpcProblem::linear_bounds(Direction direction) const noexcept
{
  // The run says which way the body drives it, so the box must not let the solver cross zero: a
  // forward run reached by backing up is a plan the planner did not make.
  switch (direction) {
    case Direction::Forward:
      return {0.0, params_.max_linear_vel};
    case Direction::Reverse:
      return {std::min(0.0, params_.min_linear_vel), 0.0};
    case Direction::InPlace:
      return {0.0, 0.0};
  }
  return {params_.min_linear_vel, params_.max_linear_vel};
}

void MpcProblem::update(
  const MpcReference & reference, const Pose2D & robot, const Twist2D & measured,
  Direction direction)
{
  const std::pair<double, double> linear = linear_bounds(direction);
  const int n = horizon_;
  const double dt = params_.prediction_dt;
  const double input_weight[2] = {params_.weight_linear_vel, params_.weight_angular_vel};
  const double rate_weight[2] = {params_.weight_linear_vel_rate, params_.weight_angular_vel_rate};
  const double accel[2] = {params_.max_linear_accel, params_.max_angular_accel};

  std::size_t p = 0;
  for (int k = 0; k <= n; ++k) {
    if (k == 0) {
      continue;
    }
    const double scale = k == n ? params_.terminal_weight_scale : 1.0;
    const StateWeight weight = weight_at(params_, reference.states[static_cast<std::size_t>(k)].yaw, scale);
    p_values_[p++] = 2.0 * weight.xx;
    p_values_[p++] = 2.0 * weight.xy;
    p_values_[p++] = 2.0 * weight.yy;
    p_values_[p++] = 2.0 * weight.yaw;
  }
  for (int j = 0; j < n; ++j) {
    for (int c = 0; c < 2; ++c) {
      if (j > 0) {
        p_values_[p++] = -2.0 * rate_weight[c];
      }
      const double own = j + 1 < n ? 2.0 * rate_weight[c] : rate_weight[c];
      p_values_[p++] = 2.0 * (input_weight[c] + own);
    }
  }

  std::fill(q_.begin(), q_.end(), 0.0);
  const auto reference_input = [&](int step, int component) {
    if (step < 0) {
      return component == 0 ? measured.linear.x() : measured.angular;
    }
    const auto index = static_cast<std::size_t>(step);
    return component == 0 ? reference.linear_vel[index] : reference.angular_vel[index];
  };
  for (int j = 0; j < n; ++j) {
    for (int c = 0; c < 2; ++c) {
      const double own = reference_input(j, c) - reference_input(j - 1, c);
      double linear = 2.0 * rate_weight[c] * own;
      if (j + 1 < n) {
        linear -= 2.0 * rate_weight[c] * (reference_input(j + 1, c) - reference_input(j, c));
      }
      q_[input_index(j) + static_cast<std::size_t>(c)] = linear;
    }
  }

  std::size_t a = 0;
  for (int k = 0; k <= n; ++k) {
    const double yaw = reference.states[static_cast<std::size_t>(k)].yaw;
    const double speed = k < n ? reference.linear_vel[static_cast<std::size_t>(k)] : 0.0;
    for (int i = 0; i < 3; ++i) {
      a_values_[a++] = k == 0 ? 1.0 : -1.0;
      if (k < n) {
        if (i == 2) {
          a_values_[a++] = -dt * speed * std::sin(yaw);
          a_values_[a++] = dt * speed * std::cos(yaw);
          a_values_[a++] = 1.0;
        } else {
          a_values_[a++] = 1.0;
        }
      }
    }
  }
  for (int j = 0; j < n; ++j) {
    const double yaw = reference.states[static_cast<std::size_t>(j)].yaw;
    for (int c = 0; c < 2; ++c) {
      if (c == 0) {
        a_values_[a++] = dt * std::cos(yaw);
        a_values_[a++] = dt * std::sin(yaw);
      } else {
        a_values_[a++] = dt;
      }
      a_values_[a++] = 1.0;
      a_values_[a++] = 1.0;
      if (j + 1 < n) {
        a_values_[a++] = -1.0;
      }
    }
  }

  const Pose2D & first = reference.states.front();
  lower_[0] = robot.position.x() - first.position.x();
  lower_[1] = robot.position.y() - first.position.y();
  lower_[2] = shortest_angular_distance(first.yaw, robot.yaw);
  upper_[0] = lower_[0];
  upper_[1] = lower_[1];
  upper_[2] = lower_[2];

  for (int k = 0; k < n; ++k) {
    const auto index = static_cast<std::size_t>(k);
    const Twist2D input{
      Eigen::Vector2d{reference.linear_vel[index], 0.0}, reference.angular_vel[index]};
    const Pose2D rolled = integrate_differential_drive(reference.states[index], input, dt);
    const Pose2D & next = reference.states[index + 1];
    const auto row = static_cast<std::size_t>(3 + 3 * k);
    lower_[row] = -(rolled.position.x() - next.position.x());
    lower_[row + 1] = -(rolled.position.y() - next.position.y());
    lower_[row + 2] = -shortest_angular_distance(next.yaw, rolled.yaw);
    upper_[row] = lower_[row];
    upper_[row + 1] = lower_[row + 1];
    upper_[row + 2] = lower_[row + 2];
  }

  const auto box_row = static_cast<std::size_t>(3 + 3 * n);
  const auto rate_row = box_row + static_cast<std::size_t>(2 * n);
  for (int j = 0; j < n; ++j) {
    for (int c = 0; c < 2; ++c) {
      const double reference_value = reference_input(j, c);
      const double low = c == 0 ? linear.first : -params_.max_angular_vel;
      const double high = c == 0 ? linear.second : params_.max_angular_vel;
      const auto box = box_row + static_cast<std::size_t>(2 * j + c);
      lower_[box] = low - reference_value;
      upper_[box] = high - reference_value;

      const double step = accel[c] * dt;
      const double drift = reference_value - reference_input(j - 1, c);
      const auto rate = rate_row + static_cast<std::size_t>(2 * j + c);
      lower_[rate] = -step - drift;
      upper_[rate] = step - drift;
    }
  }
}

}  // namespace eltanin::control::detail
