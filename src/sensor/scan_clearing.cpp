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

#include <eltanin/sensor/scan_clearing.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace eltanin::sensor
{

namespace
{

/// The clearing pass keeps the marking sector and near limit; only the far limit is its own.
ScanFilter with_clearing_range(const ScanFilter & marking_filter, double clearing_max_range)
{
  ScanFilter filter = marking_filter;
  filter.max_range = clearing_max_range;
  return filter;
}

/// A beam that reported no return says the sensor's whole range is free; anything else is unusable.
double free_range(const ScanData & scan, double range)
{
  if (range == std::numeric_limits<double>::infinity()) {
    return scan.range_max;
  }
  return range;
}

}  // namespace

void project_scan_for_clearing(
  const ScanData & scan, const ScanFilter & marking_filter, double clearing_max_range,
  std::vector<Eigen::Vector2d> & out)
{
  detail::validate_scan_arguments(scan, marking_filter);
  if (std::isnan(clearing_max_range) || clearing_max_range < 0.0) {
    throw std::invalid_argument("clearing max range must be non-negative");
  }

  out.clear();
  out.reserve(scan.ranges.size());

  const detail::RangeBounds bounds =
    detail::effective_bounds(scan, with_clearing_range(marking_filter, clearing_max_range));
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = free_range(scan, static_cast<double>(scan.ranges[i]));
    if (!std::isfinite(range)) {
      continue;
    }
    const double distance = std::min(range, bounds.upper);
    if (distance < bounds.lower) {
      continue;
    }
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    if (
      marking_filter.angle_range.has_value() &&
      !angle_in_range(angle, marking_filter.angle_range->from, marking_filter.angle_range->to)) {
      continue;
    }
    out.push_back(Eigen::Vector2d{distance * std::cos(angle), distance * std::sin(angle)});
  }
}

void project_scan_for_clearing(
  const ScanData & scan, const ScanFilter & marking_filter, double clearing_max_range,
  const Transform2D & sensor_to_world, std::vector<Eigen::Vector2d> & out)
{
  project_scan_for_clearing(scan, marking_filter, clearing_max_range, out);
  for (Eigen::Vector2d & endpoint : out) {
    endpoint = sensor_to_world * endpoint;
  }
}

}  // namespace eltanin::sensor
