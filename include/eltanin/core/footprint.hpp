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

#ifndef ELTANIN__CORE__FOOTPRINT_HPP_
#define ELTANIN__CORE__FOOTPRINT_HPP_

#include <eltanin/core/polygon.hpp>
#include <eltanin/core/traversability.hpp>

#include <optional>

namespace eltanin
{

/// Minimum distance [m] from the robot origin to any edge; nullopt if undefined for this polygon.
std::optional<double> inscribed_radius(const Polygon2D & footprint);

/// Maximum distance [m] from the robot origin to any vertex; nullopt for a degenerate polygon.
std::optional<double> circumscribed_radius(const Polygon2D & footprint);

/// Distance-input traversability model; its thresholds are the footprint distance_model in [m].
class DistanceTraversabilityModel
{
public:
  static std::optional<DistanceTraversabilityModel> from_footprint(
    const Polygon2D & footprint, double inflation_radius);

  static std::optional<DistanceTraversabilityModel> from_radii(
    double inscribed, double circumscribed, double inflation);

  double inscribed_radius() const noexcept { return inscribed_; }

  double circumscribed_radius() const noexcept { return circumscribed_; }

  double inflation_radius() const noexcept { return inflation_; }

  /// Distance-input traversability model; `distance` is the obstacle distance in [m].
  Traversability classify(double distance) const noexcept
  {
    if (distance < inscribed_) {
      return Traversability::Inscribed;
    }
    if (distance < circumscribed_) {
      return Traversability::Circumscribed;
    }
    return Traversability::Free;
  }

  /// Occupancy for the exact footprint check; only a cell at zero distance is the obstacle itself.
  bool is_obstacle(double distance) const noexcept { return !(distance > 0.0); }

  /// The cell value already is the obstacle distance [m]; this is what makes the proximity ramp work.
  double clearance(double distance) const noexcept { return distance; }

private:
  DistanceTraversabilityModel(double inscribed, double circumscribed, double inflation)
  : inscribed_(inscribed), circumscribed_(circumscribed), inflation_(inflation)
  {
  }

  double inscribed_{0.0};
  double circumscribed_{0.0};
  double inflation_{0.0};
};

static_assert(TraversabilityModel<DistanceTraversabilityModel, double>);
static_assert(ObstacleModel<DistanceTraversabilityModel, double>);
static_assert(ClearanceModel<DistanceTraversabilityModel, double>);
static_assert(ClearanceModel<DistanceTraversabilityModel, float>);

}  // namespace eltanin

#endif  // ELTANIN__CORE__FOOTPRINT_HPP_
