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

#include <eltanin/sensor/scan_projection.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>

namespace eltanin::sensor
{

namespace detail
{

RangeBounds effective_bounds(const ScanData & scan, const ScanFilter & filter)
{
  double lower = filter.min_range;
  double upper = filter.max_range;
  if (std::isfinite(scan.range_min)) {
    lower = std::max(lower, scan.range_min);
  }
  if (std::isfinite(scan.range_max)) {
    upper = std::min(upper, scan.range_max);
  }
  return RangeBounds{lower, upper};
}

}  // namespace detail

void project_scan(
  const ScanData & scan, const ScanFilter & filter, std::vector<Eigen::Vector2d> & out)
{
  assert(std::isfinite(scan.angle_min) && std::isfinite(scan.angle_increment));
  assert(std::isfinite(filter.min_range) && filter.min_range >= 0.0);
  assert(filter.min_range <= filter.max_range);
  assert(
    !filter.angle_range.has_value() ||
    (std::isfinite(filter.angle_range->from) && std::isfinite(filter.angle_range->to)));

  out.clear();
  out.reserve(scan.ranges.size());

  const detail::RangeBounds bounds = detail::effective_bounds(scan, filter);
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = static_cast<double>(scan.ranges[i]);
    if (!std::isfinite(range) || range < bounds.lower || range > bounds.upper) {
      continue;
    }
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    if (
      filter.angle_range.has_value() &&
      !angle_in_range(angle, filter.angle_range->from, filter.angle_range->to)) {
      continue;
    }
    out.push_back(Eigen::Vector2d{range * std::cos(angle), range * std::sin(angle)});
  }
}

void project_scan(
  const ScanData & scan, const ScanFilter & filter, const Transform2D & sensor_to_world,
  std::vector<Eigen::Vector2d> & out)
{
  project_scan(scan, filter, out);
  for (Eigen::Vector2d & point : out) {
    point = sensor_to_world * point;
  }
}

}  // namespace eltanin::sensor
