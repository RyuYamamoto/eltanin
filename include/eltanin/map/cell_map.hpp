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

#ifndef ELTANIN__PLANNER__CELL_MAP_HPP_
#define ELTANIN__PLANNER__CELL_MAP_HPP_

#include <eltanin/map/map_geometry.hpp>

#include <concepts>

namespace eltanin::planner
{

/// Minimum grid-map surface the planner reads; map::GridMap<T> satisfies it.
template <class Map>
concept CellMap = requires(const Map & map, int mx, int my) {
  typename Map::value_type;
  { map.geometry() } -> std::convertible_to<const map::MapGeometry &>;
  { map(mx, my) } -> std::convertible_to<typename Map::value_type>;
};

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__CELL_MAP_HPP_
