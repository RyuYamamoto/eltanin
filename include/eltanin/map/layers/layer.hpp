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

#ifndef ELTANIN__MAP__LAYERS__LAYER_HPP_
#define ELTANIN__MAP__LAYERS__LAYER_HPP_

#include <eltanin/map/grid_map.hpp>

namespace eltanin::map
{

/// Writes cost values into the master costmap. Must never change its geometry.
class Layer
{
public:
  virtual ~Layer() = default;

  virtual void update_costs(Costmap & master) = 0;

protected:
  /// Protected rather than deleted, so that derived classes keep their implicit copy and move.
  Layer() = default;
  Layer(const Layer &) = default;
  Layer & operator=(const Layer &) = default;
  Layer(Layer &&) = default;
  Layer & operator=(Layer &&) = default;
};

}  // namespace eltanin::map

#endif  // ELTANIN__MAP__LAYERS__LAYER_HPP_
