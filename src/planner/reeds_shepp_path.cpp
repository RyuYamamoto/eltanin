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

#include <eltanin/planner/reeds_shepp_path.hpp>

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

using Type = ReedsSheppSegmentType;
using Lengths = std::array<double, ReedsSheppPath::MAX_SEGMENTS>;
using Word = std::array<Type, ReedsSheppPath::MAX_SEGMENTS>;

constexpr double PI = std::numbers::pi;
constexpr double HALF_PI = 0.5 * std::numbers::pi;

/// Slack on the sign tests below, so a solution exactly on a branch boundary is not dropped.
constexpr double SIGN_EPSILON = 1e-12;

constexpr Type N = Type::None;
constexpr Type L = Type::Left;
constexpr Type S = Type::Straight;
constexpr Type R = Type::Right;

// The eighteen words of Reeds and Shepp; each family below reaches four of them by symmetry.
constexpr Word LRL{L, R, L, N, N};
constexpr Word RLR{R, L, R, N, N};
constexpr Word LRLR{L, R, L, R, N};
constexpr Word RLRL{R, L, R, L, N};
constexpr Word LRSL{L, R, S, L, N};
constexpr Word RLSR{R, L, S, R, N};
constexpr Word LSRL{L, S, R, L, N};
constexpr Word RSLR{R, S, L, R, N};
constexpr Word LRSR{L, R, S, R, N};
constexpr Word RLSL{R, L, S, L, N};
constexpr Word RSRL{R, S, R, L, N};
constexpr Word LSLR{L, S, L, R, N};
constexpr Word LSR{L, S, R, N, N};
constexpr Word RSL{R, S, L, N, N};
constexpr Word LSL{L, S, L, N, N};
constexpr Word RSR{R, S, R, N, N};
constexpr Word LRSLR{L, R, S, L, R};
constexpr Word RLSRL{R, L, S, R, L};

/// A word with its signed segment lengths, in units of the turning radius.
struct Candidate
{
  Word word{N, N, N, N, N};
  Lengths lengths{0.0, 0.0, 0.0, 0.0, 0.0};
  double total{std::numeric_limits<double>::infinity()};
};

Candidate make(const Word & word, const Lengths & lengths)
{
  double total = 0.0;
  for (const double length : lengths) {
    total += std::abs(length);
  }
  return Candidate{word, lengths, total};
}

void keep_shorter(Candidate & best, const Word & word, const Lengths & lengths)
{
  const Candidate candidate = make(word, lengths);
  if (candidate.total < best.total) {
    best = candidate;
  }
}

/// Wraps to (-pi, pi]; the Reeds-Shepp branch conditions are stated on that interval.
double mod2pi(double angle) { return normalize_angle(angle); }

void polar(double x, double y, double & radius, double & theta)
{
  radius = std::sqrt(x * x + y * y);
  theta = std::atan2(y, x);
}

/// Shared tail of formulas 8.7 and 8.8, which solve the two middle arcs of a four-arc word.
void tau_omega(
  double u, double v, double xi, double eta, double phi, double & tau, double & omega)
{
  const double delta = mod2pi(u - v);
  const double a = std::sin(u) - std::sin(delta);
  const double b = std::cos(u) - std::cos(delta) - 1.0;
  const double t1 = std::atan2(eta * a - xi * b, xi * a + eta * b);
  const double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
  tau = t2 < 0.0 ? mod2pi(t1 + PI) : mod2pi(t1);
  omega = mod2pi(tau - u + v - phi);
}

/// Formula 8.1: a left arc, a straight run and a left arc, all driven forward.
bool forward_left_straight_left(double x, double y, double phi, double & t, double & u, double & v)
{
  polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);
  if (t < -SIGN_EPSILON) {
    return false;
  }
  v = mod2pi(phi - t);
  return v >= -SIGN_EPSILON;
}

/// Formula 8.2: a left arc, a straight run and a right arc, all driven forward.
bool forward_left_straight_right(double x, double y, double phi, double & t, double & u, double & v)
{
  double radius = 0.0;
  double theta = 0.0;
  polar(x + std::sin(phi), y - 1.0 - std::cos(phi), radius, theta);
  const double squared = radius * radius;
  if (squared < 4.0) {
    return false;
  }
  u = std::sqrt(squared - 4.0);
  t = mod2pi(theta + std::atan2(2.0, u));
  v = mod2pi(t - phi);
  return t >= -SIGN_EPSILON && v >= -SIGN_EPSILON;
}

/// Formulas 8.3 and 8.4: three arcs, the middle one driven in reverse.
bool left_reverse_right_left(double x, double y, double phi, double & t, double & u, double & v)
{
  const double xi = x - std::sin(phi);
  const double eta = y - 1.0 + std::cos(phi);
  double radius = 0.0;
  double theta = 0.0;
  polar(xi, eta, radius, theta);
  if (radius > 4.0) {
    return false;
  }
  u = -2.0 * std::asin(0.25 * radius);
  t = mod2pi(theta + 0.5 * u + PI);
  v = mod2pi(phi - t + u);
  return t >= -SIGN_EPSILON && u <= SIGN_EPSILON;
}

/// Formula 8.7: four arcs whose two middle ones are equal and opposite.
bool left_right_forward_left_right_reverse(
  double x, double y, double phi, double & t, double & u, double & v)
{
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));
  if (rho > 1.0) {
    return false;
  }
  u = std::acos(rho);
  tau_omega(u, -u, xi, eta, phi, t, v);
  return t >= -SIGN_EPSILON && v <= SIGN_EPSILON;
}

/// Formula 8.8: four arcs whose two middle ones are equal.
bool left_right_reverse_left_right_forward(
  double x, double y, double phi, double & t, double & u, double & v)
{
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double rho = (20.0 - xi * xi - eta * eta) / 16.0;
  if (rho < 0.0 || rho > 1.0) {
    return false;
  }
  u = -std::acos(rho);
  if (u < -HALF_PI) {
    return false;
  }
  tau_omega(u, u, xi, eta, phi, t, v);
  return t >= -SIGN_EPSILON && v >= -SIGN_EPSILON;
}

/// Formula 8.9: two arcs, a straight run and an arc, with a quarter turn between the first two.
bool left_right_reverse_straight_left(
  double x, double y, double phi, double & t, double & u, double & v)
{
  const double xi = x - std::sin(phi);
  const double eta = y - 1.0 + std::cos(phi);
  double radius = 0.0;
  double theta = 0.0;
  polar(xi, eta, radius, theta);
  if (radius < 2.0) {
    return false;
  }
  const double leg = std::sqrt(radius * radius - 4.0);
  u = 2.0 - leg;
  t = mod2pi(theta + std::atan2(leg, -2.0));
  v = mod2pi(phi - HALF_PI - t);
  return t >= -SIGN_EPSILON && u <= SIGN_EPSILON && v <= SIGN_EPSILON;
}

/// Formula 8.10: the same shape as 8.9 with the last arc turning the other way.
bool left_right_reverse_straight_right(
  double x, double y, double phi, double & t, double & u, double & v)
{
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  double radius = 0.0;
  double theta = 0.0;
  polar(-eta, xi, radius, theta);
  if (radius < 2.0) {
    return false;
  }
  t = theta;
  u = 2.0 - radius;
  v = mod2pi(t + HALF_PI - phi);
  return t >= -SIGN_EPSILON && u <= SIGN_EPSILON && v <= SIGN_EPSILON;
}

/// Formula 8.11: a straight run flanked by a quarter turn at each end.
bool left_right_reverse_straight_left_right(
  double x, double y, double phi, double & t, double & u, double & v)
{
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  double radius = 0.0;
  double theta = 0.0;
  polar(xi, eta, radius, theta);
  if (radius < 2.0) {
    return false;
  }
  u = 4.0 - std::sqrt(radius * radius - 4.0);
  if (u > SIGN_EPSILON) {
    return false;
  }
  t = mod2pi(std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
  v = mod2pi(t - phi);
  return t >= -SIGN_EPSILON && v >= -SIGN_EPSILON;
}

using Solver = bool (*)(double, double, double, double &, double &, double &);

/// The four symmetries every family is searched under: identity, time flip, reflection, and both.
struct Variant
{
  double x_sign;
  double y_sign;
  double phi_sign;
  /// Time flip negates every segment length; reflection swaps left for right.
  bool time_flipped;
  bool reflected;
};

constexpr std::array<Variant, 4> VARIANTS{
  Variant{1.0, 1.0, 1.0, false, false}, Variant{-1.0, 1.0, -1.0, true, false},
  Variant{1.0, -1.0, -1.0, false, true}, Variant{-1.0, -1.0, 1.0, true, true}};

/// Moves along one segment by the signed arc length `distance`, which may be driven in reverse.
Pose2D advance(const Pose2D & pose, ReedsSheppSegmentType type, double distance, double radius)
{
  if (type == Type::None || distance == 0.0) {
    return pose;
  }
  if (type == Type::Straight) {
    return Pose2D{
      pose.position + distance * Eigen::Vector2d{std::cos(pose.yaw), std::sin(pose.yaw)},
      pose.yaw};
  }

  const double curvature = type == Type::Left ? 1.0 / radius : -1.0 / radius;
  const double yaw = normalize_angle(pose.yaw + distance * curvature);
  return Pose2D{
    pose.position + Eigen::Vector2d{
                      (std::sin(yaw) - std::sin(pose.yaw)) / curvature,
                      (-std::cos(yaw) + std::cos(pose.yaw)) / curvature},
    yaw};
}

}  // namespace

ReedsSheppPath::ReedsSheppPath(
  const Pose2D & start, const Pose2D & goal, double turning_radius,
  const std::array<ReedsSheppSegment, MAX_SEGMENTS> & segments)
: start_(start), goal_(goal), turning_radius_(turning_radius), length_(0.0), segments_(segments)
{
  for (const ReedsSheppSegment & segment : segments_) {
    length_ += std::abs(segment.length);
  }
}

Pose2D ReedsSheppPath::sample(double s) const
{
  if (!(s > 0.0)) {
    return start_;
  }
  if (s >= length_) {
    return goal_;
  }

  Pose2D pose = start_;
  double remaining = s;
  for (const ReedsSheppSegment & segment : segments_) {
    const double span = std::abs(segment.length);
    const double travelled = std::min(remaining, span);
    pose = advance(pose, segment.type, std::copysign(travelled, segment.length), turning_radius_);
    remaining -= travelled;
    if (remaining <= 0.0) {
      break;
    }
  }
  return pose;
}

bool ReedsSheppPath::reverse_at(double s) const
{
  const double clamped = std::clamp(s, 0.0, length_);
  double consumed = 0.0;
  for (const ReedsSheppSegment & segment : segments_) {
    const double span = std::abs(segment.length);
    if (span == 0.0) {
      continue;
    }
    consumed += span;
    if (clamped < consumed) {
      return segment.length < 0.0;
    }
  }
  // Exactly at the end, so the last segment that carries any length decides.
  for (auto it = segments_.rbegin(); it != segments_.rend(); ++it) {
    if (it->length != 0.0) {
      return it->length < 0.0;
    }
  }
  return false;
}

int ReedsSheppPath::direction_changes() const noexcept
{
  int changes = 0;
  int previous = 0;
  for (const ReedsSheppSegment & segment : segments_) {
    if (segment.length == 0.0) {
      continue;
    }
    const int sign = segment.length > 0.0 ? 1 : -1;
    if (previous != 0 && sign != previous) {
      ++changes;
    }
    previous = sign;
  }
  return changes;
}

std::optional<ReedsSheppPath> solve_reeds_shepp_path(
  const Pose2D & start, const Pose2D & goal, double turning_radius)
{
  if (
    !start.position.allFinite() || !goal.position.allFinite() || !std::isfinite(start.yaw) ||
    !std::isfinite(goal.yaw) || !std::isfinite(turning_radius) || turning_radius <= 0.0) {
    return std::nullopt;
  }

  // Everything below works in the start frame with the turning radius as the unit of length.
  const Eigen::Vector2d delta = goal.position - start.position;
  const double cos_yaw = std::cos(start.yaw);
  const double sin_yaw = std::sin(start.yaw);
  const double x = (delta.x() * cos_yaw + delta.y() * sin_yaw) / turning_radius;
  const double y = (-delta.x() * sin_yaw + delta.y() * cos_yaw) / turning_radius;
  const double phi = mod2pi(goal.yaw - start.yaw);
  // Reading the word from the goal backwards covers the families that are not their own reverse.
  const double x_back = x * std::cos(phi) + y * std::sin(phi);
  const double y_back = x * std::sin(phi) - y * std::cos(phi);

  Candidate best;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;

  const auto sweep = [&](Solver solver, const Word & plain, const Word & mirrored, bool backwards,
                         const auto & build) {
    for (const Variant & variant : VARIANTS) {
      const double sx = backwards ? x_back : x;
      const double sy = backwards ? y_back : y;
      if (!solver(variant.x_sign * sx, variant.y_sign * sy, variant.phi_sign * phi, t, u, v)) {
        continue;
      }
      const double flip = variant.time_flipped ? -1.0 : 1.0;
      keep_shorter(best, variant.reflected ? mirrored : plain, build(flip * t, flip * u, flip * v, flip));
    }
  };

  const auto three = [](double a, double b, double c, double) -> Lengths {
    return Lengths{a, b, c, 0.0, 0.0};
  };
  const auto three_reversed = [](double a, double b, double c, double) -> Lengths {
    return Lengths{c, b, a, 0.0, 0.0};
  };

  sweep(forward_left_straight_left, LSL, RSR, false, three);
  sweep(forward_left_straight_right, LSR, RSL, false, three);
  sweep(left_reverse_right_left, LRL, RLR, false, three);
  sweep(left_reverse_right_left, LRL, RLR, true, three_reversed);

  sweep(
    left_right_forward_left_right_reverse, LRLR, RLRL, false,
    [](double a, double b, double c, double) -> Lengths { return Lengths{a, b, -b, c, 0.0}; });
  sweep(
    left_right_reverse_left_right_forward, LRLR, RLRL, false,
    [](double a, double b, double c, double) -> Lengths { return Lengths{a, b, b, c, 0.0}; });

  sweep(
    left_right_reverse_straight_left, LRSL, RLSR, false,
    [](double a, double b, double c, double flip) -> Lengths {
      return Lengths{a, -flip * HALF_PI, b, c, 0.0};
    });
  sweep(
    left_right_reverse_straight_right, LRSR, RLSL, false,
    [](double a, double b, double c, double flip) -> Lengths {
      return Lengths{a, -flip * HALF_PI, b, c, 0.0};
    });
  sweep(
    left_right_reverse_straight_left, LSRL, RSLR, true,
    [](double a, double b, double c, double flip) -> Lengths {
      return Lengths{c, b, -flip * HALF_PI, a, 0.0};
    });
  sweep(
    left_right_reverse_straight_right, RSRL, LSLR, true,
    [](double a, double b, double c, double flip) -> Lengths {
      return Lengths{c, b, -flip * HALF_PI, a, 0.0};
    });

  sweep(
    left_right_reverse_straight_left_right, LRSLR, RLSRL, false,
    [](double a, double b, double c, double flip) -> Lengths {
      return Lengths{a, -flip * HALF_PI, b, -flip * HALF_PI, c};
    });

  if (!std::isfinite(best.total)) {
    return std::nullopt;
  }

  std::array<ReedsSheppSegment, ReedsSheppPath::MAX_SEGMENTS> segments;
  for (std::size_t i = 0; i < segments.size(); ++i) {
    segments[i] = ReedsSheppSegment{best.word[i], best.lengths[i] * turning_radius};
  }
  return ReedsSheppPath(start, goal, turning_radius, segments);
}

}  // namespace eltanin::planner
