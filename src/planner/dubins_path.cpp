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

#include <eltanin/planner/dubins_path.hpp>

#include <eltanin/core/angle.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>

namespace eltanin::planner
{

namespace
{

using Type = DubinsSegmentType;

struct Candidate
{
  std::array<Type, 3> types;
  std::array<double, 3> lengths;
};

double mod2pi(double angle)
{
  return normalize_angle_positive(angle);
}

std::optional<double> root(double value)
{
  constexpr double epsilon = 1e-12;
  if (value < -epsilon) {
    return std::nullopt;
  }
  return std::sqrt(std::max(0.0, value));
}

std::optional<double> acos_clamped(double value)
{
  constexpr double epsilon = 1e-12;
  if (value < -1.0 - epsilon || value > 1.0 + epsilon) {
    return std::nullopt;
  }
  return std::acos(std::clamp(value, -1.0, 1.0));
}

std::optional<Candidate> lsl(double alpha, double beta, double distance)
{
  const double tmp = distance + std::sin(alpha) - std::sin(beta);
  const auto p = root(
    2.0 + distance * distance - 2.0 * std::cos(alpha - beta) +
    2.0 * distance * (std::sin(alpha) - std::sin(beta)));
  if (!p.has_value()) {
    return std::nullopt;
  }
  const double angle = std::atan2(std::cos(beta) - std::cos(alpha), tmp);
  return Candidate{
    {Type::Left, Type::Straight, Type::Left},
    {mod2pi(-alpha + angle), *p, mod2pi(beta - angle)}};
}

std::optional<Candidate> rsr(double alpha, double beta, double distance)
{
  const double tmp = distance - std::sin(alpha) + std::sin(beta);
  const auto p = root(
    2.0 + distance * distance - 2.0 * std::cos(alpha - beta) +
    2.0 * distance * (-std::sin(alpha) + std::sin(beta)));
  if (!p.has_value()) {
    return std::nullopt;
  }
  const double angle = std::atan2(std::cos(alpha) - std::cos(beta), tmp);
  return Candidate{
    {Type::Right, Type::Straight, Type::Right},
    {mod2pi(alpha - angle), *p, mod2pi(-beta + angle)}};
}

std::optional<Candidate> lsr(double alpha, double beta, double distance)
{
  const auto p = root(
    -2.0 + distance * distance + 2.0 * std::cos(alpha - beta) +
    2.0 * distance * (std::sin(alpha) + std::sin(beta)));
  if (!p.has_value()) {
    return std::nullopt;
  }
  const double angle =
    std::atan2(-std::cos(alpha) - std::cos(beta), distance + std::sin(alpha) + std::sin(beta)) -
    std::atan2(-2.0, *p);
  return Candidate{
    {Type::Left, Type::Straight, Type::Right},
    {mod2pi(-alpha + angle), *p, mod2pi(-beta + angle)}};
}

std::optional<Candidate> rsl(double alpha, double beta, double distance)
{
  const auto p = root(
    -2.0 + distance * distance + 2.0 * std::cos(alpha - beta) -
    2.0 * distance * (std::sin(alpha) + std::sin(beta)));
  if (!p.has_value()) {
    return std::nullopt;
  }
  const double angle =
    std::atan2(std::cos(alpha) + std::cos(beta), distance - std::sin(alpha) - std::sin(beta)) -
    std::atan2(2.0, *p);
  return Candidate{
    {Type::Right, Type::Straight, Type::Left},
    {mod2pi(alpha - angle), *p, mod2pi(beta - angle)}};
}

std::optional<Candidate> rlr(double alpha, double beta, double distance)
{
  const auto angle = acos_clamped(
    (6.0 - distance * distance + 2.0 * std::cos(alpha - beta) +
     2.0 * distance * (std::sin(alpha) - std::sin(beta))) /
    8.0);
  if (!angle.has_value()) {
    return std::nullopt;
  }
  const double p = mod2pi(2.0 * std::numbers::pi - *angle);
  const double t = mod2pi(
    alpha -
    std::atan2(
      std::cos(alpha) - std::cos(beta), distance - std::sin(alpha) + std::sin(beta)) +
    0.5 * p);
  return Candidate{
    {Type::Right, Type::Left, Type::Right},
    {t, p, mod2pi(alpha - beta - t + p)}};
}

std::optional<Candidate> lrl(double alpha, double beta, double distance)
{
  const auto angle = acos_clamped(
    (6.0 - distance * distance + 2.0 * std::cos(alpha - beta) +
     2.0 * distance * (-std::sin(alpha) + std::sin(beta))) /
    8.0);
  if (!angle.has_value()) {
    return std::nullopt;
  }
  const double p = mod2pi(2.0 * std::numbers::pi - *angle);
  const double t = mod2pi(
    -alpha -
    std::atan2(
      std::cos(alpha) - std::cos(beta), distance + std::sin(alpha) - std::sin(beta)) +
    0.5 * p);
  return Candidate{
    {Type::Left, Type::Right, Type::Left},
    {t, p, mod2pi(beta - alpha - t + p)}};
}

Pose2D advance(const Pose2D & pose, DubinsSegmentType type, double distance, double radius)
{
  if (type == Type::Straight) {
    return Pose2D{
      pose.position + distance * Eigen::Vector2d{std::cos(pose.yaw), std::sin(pose.yaw)},
      pose.yaw};
  }

  const double curvature = type == Type::Left ? 1.0 / radius : -1.0 / radius;
  const double yaw = normalize_angle(pose.yaw + distance * curvature);
  return Pose2D{
    pose.position +
      Eigen::Vector2d{
        (std::sin(yaw) - std::sin(pose.yaw)) / curvature,
        (-std::cos(yaw) + std::cos(pose.yaw)) / curvature},
    yaw};
}

}  // namespace

DubinsPath::DubinsPath(
  const Pose2D & start, const Pose2D & goal, double turning_radius,
  const std::array<DubinsSegment, 3> & segments)
: start_(start), goal_(goal), turning_radius_(turning_radius), length_(0.0), segments_(segments)
{
  for (const DubinsSegment & segment : segments_) {
    length_ += segment.length;
  }
}

Pose2D DubinsPath::sample(double s) const
{
  if (!(s > 0.0)) {
    return start_;
  }
  if (s >= length_) {
    return goal_;
  }

  Pose2D pose = start_;
  double remaining = s;
  for (const DubinsSegment & segment : segments_) {
    const double distance = std::min(remaining, segment.length);
    pose = advance(pose, segment.type, distance, turning_radius_);
    remaining -= distance;
    if (remaining <= 0.0) {
      break;
    }
  }
  return pose;
}

std::optional<DubinsPath> solve_dubins_path(
  const Pose2D & start, const Pose2D & goal, double turning_radius)
{
  if (!start.position.allFinite() || !goal.position.allFinite() || !std::isfinite(start.yaw) ||
      !std::isfinite(goal.yaw) || !std::isfinite(turning_radius) || turning_radius <= 0.0) {
    return std::nullopt;
  }

  const Eigen::Vector2d delta = goal.position - start.position;
  const double distance = delta.norm() / turning_radius;
  const double theta = mod2pi(std::atan2(delta.y(), delta.x()));
  const double alpha = mod2pi(start.yaw - theta);
  const double beta = mod2pi(goal.yaw - theta);

  using Solver = std::optional<Candidate> (*)(double, double, double);
  constexpr std::array<Solver, 6> solvers{lsl, rsr, lsr, rsl, rlr, lrl};
  std::optional<Candidate> best;
  double best_length = std::numeric_limits<double>::infinity();
  for (const Solver solver : solvers) {
    const auto candidate = solver(alpha, beta, distance);
    if (!candidate.has_value()) {
      continue;
    }
    const double length = candidate->lengths[0] + candidate->lengths[1] + candidate->lengths[2];
    if (length < best_length) {
      best_length = length;
      best = candidate;
    }
  }
  if (!best.has_value()) {
    return std::nullopt;
  }

  std::array<DubinsSegment, 3> segments;
  for (std::size_t i = 0; i < segments.size(); ++i) {
    segments[i] = DubinsSegment{best->types[i], best->lengths[i] * turning_radius};
  }
  return DubinsPath(start, goal, turning_radius, segments);
}

}  // namespace eltanin::planner
