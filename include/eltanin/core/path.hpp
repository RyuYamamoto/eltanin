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

#ifndef ELTANIN__CORE__PATH_HPP_
#define ELTANIN__CORE__PATH_HPP_

#include <eltanin/core/types.hpp>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <utility>
#include <vector>

namespace eltanin
{

/// How the body traverses one segment of a path; a pose yaw is the body heading, never the tangent.
enum class Direction : std::uint8_t
{
  Forward,  ///< the body drives along the tangent
  Reverse,  ///< the body drives along the tangent turned by pi
  InPlace,  ///< the two poses share a position and only the yaw changes
};

/// Sequence of poses, optionally carrying the direction each segment between them is driven in.
class Path
{
public:
  Path() = default;

  explicit Path(std::vector<Pose2D> poses) : poses_(std::move(poses)) {}

  Path(std::initializer_list<Pose2D> poses) : poses_(poses) {}

  /// `directions` is empty for an all-forward path, or holds one entry per segment.
  Path(std::vector<Pose2D> poses, std::vector<Direction> directions)
  : poses_(std::move(poses)), directions_(std::move(directions))
  {
    assert(directions_.empty() || directions_.size() + 1 == poses_.size());
  }

  const std::vector<Pose2D> & poses() const noexcept { return poses_; }

  bool empty() const noexcept { return poses_.empty(); }

  std::size_t size() const noexcept { return poses_.size(); }

  const Pose2D & operator[](std::size_t i) const { return poses_[i]; }

  Pose2D & operator[](std::size_t i) { return poses_[i]; }

  auto begin() const noexcept { return poses_.begin(); }

  auto end() const noexcept { return poses_.end(); }

  auto begin() noexcept { return poses_.begin(); }

  auto end() noexcept { return poses_.end(); }

  void push_back(const Pose2D & pose)
  {
    const bool directed = !directions_.empty();
    poses_.push_back(pose);
    if (directed) {
      directions_.push_back(Direction::Forward);
    }
  }

  /// Appends a pose reached by driving the segment that ends at it in `direction`.
  void push_back(const Pose2D & pose, Direction direction)
  {
    if (poses_.empty()) {
      poses_.push_back(pose);
      return;
    }
    if (directions_.empty()) {
      directions_.assign(poses_.size() - 1, Direction::Forward);
    }
    poses_.push_back(pose);
    directions_.push_back(direction);
  }

  void clear() noexcept
  {
    poses_.clear();
    directions_.clear();
  }

  /// Whether the path carries an explicit direction per segment; one that does not is all forward.
  bool has_directions() const noexcept { return !directions_.empty(); }

  const std::vector<Direction> & directions() const noexcept { return directions_; }

  /// How the segment from pose `segment` to pose `segment + 1` is driven.
  Direction direction_of(std::size_t segment) const noexcept
  {
    return segment < directions_.size() ? directions_[segment] : Direction::Forward;
  }

  bool has_reverse() const noexcept
  {
    for (const Direction direction : directions_) {
      if (direction == Direction::Reverse) {
        return true;
      }
    }
    return false;
  }

  /// Whether pose `pose` is a cusp: an interior pose whose two segments are driven differently.
  bool is_cusp(std::size_t pose) const noexcept
  {
    if (pose == 0 || pose + 1 >= poses_.size()) {
      return false;
    }
    return direction_of(pose - 1) != direction_of(pose);
  }

  /// First and last pose index, both inclusive, of the run of like segments `pose` sits in.
  std::pair<std::size_t, std::size_t> run_bounds(std::size_t pose) const noexcept
  {
    if (poses_.empty()) {
      return {0, 0};
    }
    const std::size_t last_pose = poses_.size() - 1;
    std::size_t first = pose < last_pose ? pose : last_pose;
    while (first > 0 && !is_cusp(first)) {
      --first;
    }
    std::size_t last = first < last_pose ? first + 1 : first;
    while (last < last_pose && !is_cusp(last)) {
      ++last;
    }
    return {first, last};
  }

private:
  std::vector<Pose2D> poses_;
  std::vector<Direction> directions_;
};

/// Sum of distances between consecutive positions [m]; 0 for an empty or single-pose path.
double path_length(const Path & path);

/// Arc length from the first pose to each pose [m]; one entry per pose, [0] is 0, non-decreasing.
std::vector<double> cumulative_arc_length(const Path & path);

/// Signed circumcircle curvature [1/m] of the poses `window` of arc length away on each side.
std::vector<double> path_curvature(const Path & path, double window);

}  // namespace eltanin

#endif  // ELTANIN__CORE__PATH_HPP_
