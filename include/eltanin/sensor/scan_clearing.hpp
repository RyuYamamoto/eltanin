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

#ifndef ELTANIN__SENSOR__SCAN_CLEARING_HPP_
#define ELTANIN__SENSOR__SCAN_CLEARING_HPP_

#include <eltanin/core/types.hpp>
#include <eltanin/sensor/scan_projection.hpp>

#include <Eigen/Core>

#include <vector>

namespace eltanin::sensor
{

/// Far end of each beam's free segment, in the sensor frame; see `docs/sensor-design.md` §11.
void project_scan_for_clearing(
  const ScanData & scan, const ScanFilter & marking_filter, double clearing_max_range,
  std::vector<Eigen::Vector2d> & out);

/// Same, then maps every endpoint with `sensor_to_world` (the transform taking sensor to world).
void project_scan_for_clearing(
  const ScanData & scan, const ScanFilter & marking_filter, double clearing_max_range,
  const Transform2D & sensor_to_world, std::vector<Eigen::Vector2d> & out);

}  // namespace eltanin::sensor

#endif  // ELTANIN__SENSOR__SCAN_CLEARING_HPP_
