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

#ifndef ELTANIN__MAP__LAYERS__INFLATION_LAYER_HPP_
#define ELTANIN__MAP__LAYERS__INFLATION_LAYER_HPP_

#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/map/layers/layer.hpp>

#include <cstdint>
#include <vector>

namespace eltanin::map
{

/// Expands the cost of every LETHAL_OBSTACLE cell over its neighbourhood.
class InflationLayer final : public Layer
{
public:
  explicit InflationLayer(const InflationCostModel & model, bool inflate_unknown = false);

  /// Precondition: master.geometry().resolution() > 0.
  void update_costs(Costmap & master) override;

private:
  void rebuild_cost_lut(double resolution);

  /// Cost for the offset, or a negative value when the offset lies outside inflation_radius.
  std::int16_t lut_cost(int abs_dx, int abs_dy) const noexcept;

  void inflate_from(Costmap & master, int mx, int my) const;

  InflationCostModel model_;
  bool inflate_unknown_{false};
  double lut_resolution_{0.0};
  int cell_inflation_radius_{0};
  std::vector<std::int16_t> cost_lut_;
};

}  // namespace eltanin::map

#endif  // ELTANIN__MAP__LAYERS__INFLATION_LAYER_HPP_
