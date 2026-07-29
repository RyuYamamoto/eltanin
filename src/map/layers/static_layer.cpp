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

#include <eltanin/map/layers/static_layer.hpp>

#include <algorithm>
#include <cassert>
#include <optional>
#include <utility>

namespace eltanin::map
{

StaticLayer::StaticLayer(Costmap map) : map_(std::move(map))
{
  assert(map_.cell_count() > 0);
  assert(map_.geometry().resolution() > 0.0);
}

void StaticLayer::update_costs(Costmap & master)
{
  if (master.geometry() == map_.geometry()) {
    std::copy(map_.data().begin(), map_.data().end(), master.data().begin());
    return;
  }

  const MapGeometry & master_geometry = master.geometry();
  const MapGeometry & source_geometry = map_.geometry();
  for (int my = 0; my < master_geometry.size_y(); ++my) {
    for (int mx = 0; mx < master_geometry.size_x(); ++mx) {
      const std::optional<MapIndex> source =
        source_geometry.world_to_map(master_geometry.map_to_world(mx, my));
      if (!source.has_value()) {
        continue;
      }
      master(mx, my) = map_(source->x, source->y);
    }
  }
}

}  // namespace eltanin::map
