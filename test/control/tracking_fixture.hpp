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

#ifndef ELTANIN_TEST__CONTROL__TRACKING_FIXTURE_HPP_
#define ELTANIN_TEST__CONTROL__TRACKING_FIXTURE_HPP_

#include <eltanin/control/pure_pursuit.hpp>
#include <eltanin/core/angle.hpp>
#include <eltanin/core/geometry.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/core/types.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <utility>

namespace eltanin_test
{

/// Mirror of the private constant in src/control/pure_pursuit.cpp; tests assert exact ramp values.
inline constexpr double LINEAR_VEL_GAIN = 1.0;

/// Mirror of the private constant in src/control/pure_pursuit.cpp.
inline constexpr double ALIGNMENT_ANGULAR_VEL_RATIO = 0.5;

/// Simulation step matching navyu's update_frequency of 100 Hz [s].
inline constexpr double SIMULATION_DT = 0.01;

struct TrackingResult
{
  bool reached{false};
  std::size_t steps{0};
  double travelled{0.0};
  double max_lateral_error{0.0};
  double max_lateral_error_after_gate{0.0};
  double final_lateral_error{0.0};
  double max_linear_vel{0.0};
  double max_abs_angular_vel{0.0};
  eltanin::Pose2D final_pose{};
};

/// Minimum distance from `position` to the path polyline.
inline double lateral_error(const eltanin::Path & path, const Eigen::Vector2d & position)
{
  assert(!path.empty());
  if (path.size() == 1) {
    return (position - path[0].position).norm();
  }
  double minimum = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < path.size(); ++i) {
    minimum = std::min(
      minimum, eltanin::distance_to_segment(position, path[i - 1].position, path[i].position));
  }
  return minimum;
}

/// Explicit Euler integration of a differential-drive robot; `command` returns nullopt when done.
template <class CommandFn>
TrackingResult simulate(
  CommandFn && command, const eltanin::Path & path, const eltanin::Pose2D & start,
  double dt = SIMULATION_DT, double gate = 1.0, std::size_t max_steps = 20000)
{
  TrackingResult result;
  eltanin::Pose2D pose = start;
  for (std::size_t step = 0; step < max_steps; ++step) {
    const double error = lateral_error(path, pose.position);
    result.max_lateral_error = std::max(result.max_lateral_error, error);
    if (result.travelled >= gate) {
      result.max_lateral_error_after_gate = std::max(result.max_lateral_error_after_gate, error);
    }

    const std::optional<eltanin::Twist2D> twist = command(pose, dt);
    if (!twist.has_value()) {
      result.reached = true;
      break;
    }
    result.max_linear_vel = std::max(result.max_linear_vel, twist->linear.x());
    result.max_abs_angular_vel = std::max(result.max_abs_angular_vel, std::abs(twist->angular));

    const double v = twist->linear.x();
    pose.position.x() += v * std::cos(pose.yaw) * dt;
    pose.position.y() += v * std::sin(pose.yaw) * dt;
    pose.yaw = eltanin::normalize_angle(pose.yaw + twist->angular * dt);
    result.travelled += v * dt;
    result.steps = step + 1;
  }
  result.final_pose = pose;
  result.final_lateral_error = lateral_error(path, pose.position);
  return result;
}

/// Straight path along +x from the origin; the last pose sits at exactly `length`.
inline eltanin::Path make_straight_path(double length, double spacing)
{
  assert(length > 0.0 && spacing > 0.0);
  const auto steps = static_cast<std::size_t>(std::llround(length / spacing));
  eltanin::Path path;
  for (std::size_t i = 0; i <= steps; ++i) {
    const double x = length * static_cast<double>(i) / static_cast<double>(steps);
    path.push_back(eltanin::Pose2D{Eigen::Vector2d{x, 0.0}, 0.0});
  }
  return path;
}

/// Circular arc starting at the origin heading +x; `sign` of +1 turns left, -1 turns right.
inline eltanin::Path make_arc_path(double radius, double sweep, double spacing, double sign)
{
  assert(radius > 0.0 && sweep > 0.0 && spacing > 0.0);
  const auto steps = static_cast<std::size_t>(std::llround(radius * sweep / spacing));
  eltanin::Path path;
  for (std::size_t i = 0; i <= steps; ++i) {
    const double theta = sweep * static_cast<double>(i) / static_cast<double>(steps);
    path.push_back(eltanin::Pose2D{
      Eigen::Vector2d{radius * std::sin(theta), sign * radius * (1.0 - std::cos(theta))},
      sign * theta});
  }
  return path;
}

/// Global nearest pose, matching the tracker's tie-breaking (smallest index).
inline std::size_t fixture_nearest_index(
  const eltanin::Path & path, const Eigen::Vector2d & position)
{
  std::size_t nearest = 0;
  double min_distance = (path[0].position - position).squaredNorm();
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double distance = (path[i].position - position).squaredNorm();
    if (distance < min_distance) {
      min_distance = distance;
      nearest = i;
    }
  }
  return nearest;
}

/// First pose at or beyond `lookahead` walking forward from `from`; the last pose otherwise.
inline std::size_t fixture_select_target_index(
  const eltanin::Path & path, const Eigen::Vector2d & position, std::size_t from, double lookahead)
{
  for (std::size_t i = from; i < path.size(); ++i) {
    if ((path[i].position - position).norm() >= lookahead) {
      return i;
    }
  }
  return path.size() - 1;
}

/// Adapts PurePursuit to the simulate() command signature.
class PurePursuitDriver
{
public:
  PurePursuitDriver(eltanin::control::PurePursuit pursuit, const eltanin::Path & path)
  : pursuit_(std::move(pursuit)), path_(&path)
  {
  }

  std::optional<eltanin::Twist2D> operator()(const eltanin::Pose2D & robot, double dt)
  {
    const auto result = pursuit_.compute(robot, *path_, dt);
    if (result.status != eltanin::control::PurePursuit::Status::Tracking) {
      return std::nullopt;
    }
    return result.command;
  }

private:
  eltanin::control::PurePursuit pursuit_;
  const eltanin::Path * path_;
};

/// The navyu curvature (w = v sin(alpha) / L_nominal) with every other step identical.
class NavyuFormReference
{
public:
  NavyuFormReference(
    const eltanin::control::PurePursuitParams & params, const eltanin::Path & path)
  : params_(params), path_(&path)
  {
  }

  std::optional<eltanin::Twist2D> operator()(const eltanin::Pose2D & robot, double dt)
  {
    if (path_->empty()) {
      return std::nullopt;
    }
    const std::size_t nearest = fixture_nearest_index(*path_, robot.position);
    if (nearest + 1 == path_->size()) {
      return std::nullopt;
    }
    const double lookahead = params_.lookahead_time * linear_vel_ + params_.min_lookahead_dist;
    const std::size_t target =
      fixture_select_target_index(*path_, robot.position, nearest, lookahead);
    const Eigen::Vector2d delta = (*path_)[target].position - robot.position;
    const double alpha = eltanin::normalize_angle(std::atan2(delta.y(), delta.x()) - robot.yaw);

    if (!yaw_aligned_) {
      if (std::abs(alpha) < params_.yaw_tolerance) {
        yaw_aligned_ = true;
      } else {
        const double sign = (alpha > 0.0) ? 1.0 : -1.0;
        return eltanin::Twist2D{
          Eigen::Vector2d::Zero(),
          sign * params_.max_angular_vel * ALIGNMENT_ANGULAR_VEL_RATIO};
      }
    }

    linear_vel_ += LINEAR_VEL_GAIN * (params_.desired_linear_vel - linear_vel_) * dt;
    linear_vel_ = std::clamp(linear_vel_, 0.0, params_.desired_linear_vel);
    // The defect under test: coefficient 2 missing and the nominal lookahead used as the radius.
    const double angular_vel = std::clamp(
      linear_vel_ * std::sin(alpha) / lookahead, -params_.max_angular_vel,
      params_.max_angular_vel);
    return eltanin::Twist2D{Eigen::Vector2d{linear_vel_, 0.0}, angular_vel};
  }

private:
  eltanin::control::PurePursuitParams params_{};
  const eltanin::Path * path_;
  double linear_vel_{0.0};
  bool yaw_aligned_{false};
};

}  // namespace eltanin_test

#endif  // ELTANIN_TEST__CONTROL__TRACKING_FIXTURE_HPP_
