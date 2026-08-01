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

#ifndef ELTANIN__PLANNER__DUBINS_PATH_HPP_
#define ELTANIN__PLANNER__DUBINS_PATH_HPP_

#include <eltanin/core/types.hpp>

#include <array>
#include <optional>

namespace eltanin::planner
{

enum class DubinsSegmentType
{
  Left,
  Straight,
  Right,
};

struct DubinsSegment
{
  DubinsSegmentType type;
  double length;
};

class DubinsPath
{
public:
  const Pose2D & start() const noexcept { return start_; }

  const Pose2D & goal() const noexcept { return goal_; }

  double turning_radius() const noexcept { return turning_radius_; }

  double length() const noexcept { return length_; }

  const std::array<DubinsSegment, 3> & segments() const noexcept { return segments_; }

  /// Pose at arc length `s` [m]. Values outside the path are clamped to its endpoints.
  Pose2D sample(double s) const;

private:
  DubinsPath(
    const Pose2D & start, const Pose2D & goal, double turning_radius,
    const std::array<DubinsSegment, 3> & segments);

  Pose2D start_;
  Pose2D goal_;
  double turning_radius_;
  double length_;
  std::array<DubinsSegment, 3> segments_;

  friend std::optional<DubinsPath> solve_dubins_path(
    const Pose2D &, const Pose2D &, double);
};

/// Shortest forward-only Dubins path. Returns nullopt for non-finite input or invalid radius.
std::optional<DubinsPath> solve_dubins_path(
  const Pose2D & start, const Pose2D & goal, double turning_radius);

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__DUBINS_PATH_HPP_
