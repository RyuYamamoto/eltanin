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

#ifndef ELTANIN__MAP__LAYERS__STATIC_LAYER_HPP_
#define ELTANIN__MAP__LAYERS__STATIC_LAYER_HPP_

#include <eltanin/map/grid_map.hpp>
#include <eltanin/map/layers/layer.hpp>

namespace eltanin::map
{

/// Copies the cells of an owned static map into the master, leaving the master geometry alone.
class StaticLayer final : public Layer
{
public:
  /// Precondition: map.cell_count() > 0 and map.geometry().resolution() > 0.
  explicit StaticLayer(Costmap map);

  /// Cells outside the source map are left untouched, so the reset value survives there.
  void update_costs(Costmap & master) override;

private:
  Costmap map_;
};

}  // namespace eltanin::map

#endif  // ELTANIN__MAP__LAYERS__STATIC_LAYER_HPP_
