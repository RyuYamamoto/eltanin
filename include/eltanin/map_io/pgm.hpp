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

#ifndef ELTANIN__MAP_IO__PGM_HPP_
#define ELTANIN__MAP_IO__PGM_HPP_

#include <eltanin/map/grid_map.hpp>
#include <eltanin/map_io/error.hpp>

#include <cstdint>
#include <filesystem>
#include <vector>

namespace eltanin::map_io
{

/// Raw P5 image in file order: row 0 is the top of the image.
struct PgmImage
{
  int width{0};
  int height{0};
  std::vector<std::uint8_t> pixels;
};

/// Reads binary P5 with maxval 255. Comment lines are skipped anywhere in the header.
PgmImage read_pgm(const std::filesystem::path & path);

/// Debug dump of raw cell values, flipped so that my = 0 becomes the bottom image row.
void write_pgm(const std::filesystem::path & path, const map::Costmap & costmap);

}  // namespace eltanin::map_io

#endif  // ELTANIN__MAP_IO__PGM_HPP_
