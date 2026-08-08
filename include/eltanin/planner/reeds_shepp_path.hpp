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

#ifndef ELTANIN__PLANNER__REEDS_SHEPP_PATH_HPP_
#define ELTANIN__PLANNER__REEDS_SHEPP_PATH_HPP_

#include <eltanin/core/types.hpp>

#include <array>
#include <cstddef>
#include <optional>

namespace eltanin::planner
{

enum class ReedsSheppSegmentType
{
  None,
  Left,
  Straight,
  Right,
};

struct ReedsSheppSegment
{
  ReedsSheppSegmentType type{ReedsSheppSegmentType::None};
  /// Signed arc length [m]; a negative value is driven in reverse.
  double length{0.0};
};

/// Shortest forward-and-reverse path between two poses at a fixed turning radius.
class ReedsSheppPath
{
public:
  static constexpr std::size_t MAX_SEGMENTS = 5;

  const Pose2D & start() const noexcept { return start_; }

  const Pose2D & goal() const noexcept { return goal_; }

  double turning_radius() const noexcept { return turning_radius_; }

  /// Sum of the absolute segment lengths [m]; this is what sample() measures along.
  double length() const noexcept { return length_; }

  const std::array<ReedsSheppSegment, MAX_SEGMENTS> & segments() const noexcept
  {
    return segments_;
  }

  /// Pose at arc length `s` [m]. Values outside the path are clamped to its endpoints.
  Pose2D sample(double s) const;

  /// Whether the segment covering arc length `s` is driven in reverse.
  bool reverse_at(double s) const;

  /// Number of times the direction of travel changes; each one is a stop and a gear change.
  int direction_changes() const noexcept;

private:
  ReedsSheppPath(
    const Pose2D & start, const Pose2D & goal, double turning_radius,
    const std::array<ReedsSheppSegment, MAX_SEGMENTS> & segments);

  Pose2D start_;
  Pose2D goal_;
  double turning_radius_;
  double length_;
  std::array<ReedsSheppSegment, MAX_SEGMENTS> segments_;

  friend std::optional<ReedsSheppPath> solve_reeds_shepp_path(
    const Pose2D &, const Pose2D &, double);
};

/// Shortest Reeds-Shepp path. Returns nullopt for non-finite input or a non-positive radius.
std::optional<ReedsSheppPath> solve_reeds_shepp_path(
  const Pose2D & start, const Pose2D & goal, double turning_radius);

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__REEDS_SHEPP_PATH_HPP_
