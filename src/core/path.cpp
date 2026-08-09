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

#include <eltanin/core/path.hpp>

#include <algorithm>
#include <cmath>

namespace eltanin
{

double path_length(const Path & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += (path[i].position - path[i - 1].position).norm();
  }
  return length;
}

std::vector<double> cumulative_arc_length(const Path & path)
{
  std::vector<double> lengths;
  lengths.reserve(path.size());
  double length = 0.0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    // Same accumulation order as path_length, so back() matches it exactly.
    if (i > 0) {
      length += (path[i].position - path[i - 1].position).norm();
    }
    lengths.push_back(length);
  }
  return lengths;
}

std::vector<double> path_curvature(const Path & path, double window)
{
  std::vector<double> curvature(path.size(), 0.0);
  if (path.size() < 3) {
    return curvature;
  }

  const std::vector<double> arc = cumulative_arc_length(path);
  const double span = (std::isfinite(window) && window > 0.0) ? window : 0.0;

  std::size_t back = 0;
  std::size_t forward = 1;
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    while (back + 1 < i && arc[i] - arc[back + 1] >= span) {
      ++back;
    }
    forward = std::max(forward, i + 1);
    while (forward + 1 < path.size() && arc[forward] - arc[i] < span) {
      ++forward;
    }
    // Endpoints the window does not reach measure the sampling instead of the path, so drop them.
    if (arc[i] - arc[back] < span || arc[forward] - arc[i] < span) {
      continue;
    }

    const Eigen::Vector2d incoming = path[i].position - path[back].position;
    const Eigen::Vector2d outgoing = path[forward].position - path[i].position;
    const Eigen::Vector2d chord = path[forward].position - path[back].position;
    const double denominator = incoming.norm() * outgoing.norm() * chord.norm();
    if (denominator > 0.0) {
      const double twice_area = incoming.x() * outgoing.y() - incoming.y() * outgoing.x();
      curvature[i] = 2.0 * twice_area / denominator;
    }
  }
  return curvature;
}

}  // namespace eltanin
