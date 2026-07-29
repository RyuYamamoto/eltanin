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

#include <eltanin/map/cost_model.hpp>

#include <algorithm>
#include <cmath>

namespace eltanin::map
{

InflationCostModel::InflationCostModel(const CollisionRadii & radii, double cost_scaling_factor)
: radii_(radii), cost_scaling_factor_(cost_scaling_factor)
{
  circumscribed_cost_ =
    std::max<std::uint8_t>(1, cost_at_distance(radii_.circumscribed_radius()));
}

std::optional<InflationCostModel> InflationCostModel::create(
  const CollisionRadii & radii, double cost_scaling_factor)
{
  if (!std::isfinite(cost_scaling_factor) || cost_scaling_factor < 0.0) {
    return std::nullopt;
  }
  return InflationCostModel(radii, cost_scaling_factor);
}

std::uint8_t InflationCostModel::cost_at_distance(double distance) const noexcept
{
  if (std::isnan(distance)) {
    return LETHAL_OBSTACLE;
  }
  if (distance < radii_.inscribed_radius()) {
    return INSCRIBED_INFLATED_OBSTACLE;
  }
  if (distance > radii_.inflation_radius()) {
    return FREE_SPACE;
  }
  const double decay =
    std::exp(-cost_scaling_factor_ * (distance - radii_.inscribed_radius()));
  const double cost = static_cast<double>(MAX_NON_OBSTACLE) * decay;
  return static_cast<std::uint8_t>(std::clamp(cost, 0.0, static_cast<double>(MAX_NON_OBSTACLE)));
}

}  // namespace eltanin::map
