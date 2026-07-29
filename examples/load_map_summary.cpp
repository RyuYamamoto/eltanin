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

#include <eltanin/map/cost_values.hpp>
#include <eltanin/map_io/map_loader.hpp>

#include <cstddef>
#include <cstdint>
#include <iostream>

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " <map.yaml>\n";
    return 1;
  }

  try {
    const eltanin::map::Costmap costmap = eltanin::map_io::load_map(argv[1]);
    const eltanin::map::MapGeometry & geometry = costmap.geometry();

    std::cout << "size        : " << geometry.size_x() << " x " << geometry.size_y() << '\n'
              << "resolution  : " << geometry.resolution() << " m/cell\n"
              << "origin      : (" << geometry.origin().x() << ", " << geometry.origin().y()
              << ")\n"
              << "cells       : " << costmap.cell_count() << '\n';

    std::size_t free_cells = 0;
    std::size_t inflated_cells = 0;
    std::size_t inscribed_cells = 0;
    std::size_t lethal_cells = 0;
    std::size_t unknown_cells = 0;
    for (std::size_t i = 0; i < costmap.cell_count(); ++i) {
      const std::uint8_t cost = costmap[i];
      if (cost == eltanin::map::FREE_SPACE) {
        ++free_cells;
      } else if (cost <= eltanin::map::MAX_NON_OBSTACLE) {
        ++inflated_cells;
      } else if (cost == eltanin::map::INSCRIBED_INFLATED_OBSTACLE) {
        ++inscribed_cells;
      } else if (cost == eltanin::map::LETHAL_OBSTACLE) {
        ++lethal_cells;
      } else {
        ++unknown_cells;
      }
    }

    std::cout << "free        : " << free_cells << '\n'
              << "inflated    : " << inflated_cells << '\n'
              << "inscribed   : " << inscribed_cells << '\n'
              << "lethal      : " << lethal_cells << '\n'
              << "unknown     : " << unknown_cells << '\n';
  } catch (const eltanin::map_io::MapIoError & error) {
    std::cerr << "failed to load map: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
