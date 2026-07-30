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

#include <cstddef>
#include <initializer_list>
#include <utility>
#include <vector>

namespace eltanin
{

/// Sequence of poses. Ownership and basic operations only; resampling belongs in free functions.
class Path
{
public:
  Path() = default;

  explicit Path(std::vector<Pose2D> poses) : poses_(std::move(poses)) {}

  Path(std::initializer_list<Pose2D> poses) : poses_(poses) {}

  const std::vector<Pose2D> & poses() const noexcept { return poses_; }

  bool empty() const noexcept { return poses_.empty(); }

  std::size_t size() const noexcept { return poses_.size(); }

  const Pose2D & operator[](std::size_t i) const { return poses_[i]; }

  Pose2D & operator[](std::size_t i) { return poses_[i]; }

  auto begin() const noexcept { return poses_.begin(); }

  auto end() const noexcept { return poses_.end(); }

  auto begin() noexcept { return poses_.begin(); }

  auto end() noexcept { return poses_.end(); }

  void push_back(const Pose2D & pose) { poses_.push_back(pose); }

  void clear() noexcept { poses_.clear(); }

private:
  std::vector<Pose2D> poses_;
};

/// Sum of distances between consecutive positions [m]; 0 for an empty or single-pose path.
double path_length(const Path & path);

/// Arc length from the first pose to each pose [m]; one entry per pose, [0] is 0, non-decreasing.
std::vector<double> cumulative_arc_length(const Path & path);

}  // namespace eltanin

#endif  // ELTANIN__CORE__PATH_HPP_
