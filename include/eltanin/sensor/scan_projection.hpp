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

#ifndef ELTANIN__SENSOR__SCAN_PROJECTION_HPP_
#define ELTANIN__SENSOR__SCAN_PROJECTION_HPP_

#include <eltanin/core/angle.hpp>
#include <eltanin/core/types.hpp>

#include <Eigen/Core>

#include <limits>
#include <optional>
#include <vector>

namespace eltanin::sensor
{

/// One planar scan. Beam i points at angle_min + i * angle_increment in the sensor frame.
struct ScanData
{
  double angle_min{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{std::numeric_limits<double>::infinity()};
  std::vector<float> ranges;
};

/// Which beams to trust. The defaults let only the ScanData's own limits apply.
struct ScanFilter
{
  double min_range{0.0};
  double max_range{std::numeric_limits<double>::infinity()};
  std::optional<AngleRange> angle_range;
};

/// Projects the surviving beams into the sensor frame; `out` is cleared first, never appended to.
void project_scan(
  const ScanData & scan, const ScanFilter & filter, std::vector<Eigen::Vector2d> & out);

/// Same, then maps every point with `sensor_to_world` (the transform taking sensor to world).
void project_scan(
  const ScanData & scan, const ScanFilter & filter, const Transform2D & sensor_to_world,
  std::vector<Eigen::Vector2d> & out);

}  // namespace eltanin::sensor

#endif  // ELTANIN__SENSOR__SCAN_PROJECTION_HPP_
