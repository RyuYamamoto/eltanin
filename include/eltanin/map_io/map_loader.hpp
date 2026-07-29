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

#ifndef ELTANIN__MAP_IO__MAP_LOADER_HPP_
#define ELTANIN__MAP_IO__MAP_LOADER_HPP_

#include <eltanin/core/types.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/map_io/load_error.hpp>

#include <cstdint>
#include <filesystem>

namespace eltanin::map_io
{

/// ROS map_server style metadata. No yaml-cpp type appears in this interface.
struct MapMetadata
{
  std::filesystem::path image_path;
  double resolution{0.0};
  Vec2 origin{Vec2::Zero()};
  bool negate{false};
  double occupied_thresh{0.65};
  double free_thresh{0.196};
};

/// Reads and validates the YAML alone. Throws LoadError.
MapMetadata load_map_metadata(const std::filesystem::path & yaml_path);

/// Reads YAML plus PGM into a costmap, flipping rows so that my = 0 is the bottom. Throws LoadError.
map::Costmap load_map(const std::filesystem::path & yaml_path);

/// Applies the ROS map_server occupancy thresholds to one pixel value.
std::uint8_t occupancy_cost(std::uint8_t pixel, const MapMetadata & metadata) noexcept;

}  // namespace eltanin::map_io

#endif  // ELTANIN__MAP_IO__MAP_LOADER_HPP_
