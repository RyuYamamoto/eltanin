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

#include <eltanin/core/footprint.hpp>

#include <cmath>
#include <cstddef>
#include <limits>

namespace eltanin
{
namespace
{

/// Areas below this [m^2] are treated as degenerate.
constexpr double kDegenerateAreaTolerance = 1e-12;

}  // namespace

std::optional<double> inscribed_radius(const Polygon2D & footprint)
{
  const std::vector<Eigen::Vector2d> & v = footprint.vertices();
  const std::size_t n = v.size();
  if (n < 3) {
    return std::nullopt;
  }
  if (std::abs(signed_area(footprint)) <= kDegenerateAreaTolerance) {
    return std::nullopt;
  }
  if (!contains(footprint, Eigen::Vector2d::Zero())) {
    return std::nullopt;
  }

  double minimum = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const double distance = distance_to_segment(Eigen::Vector2d::Zero(), v[j], v[i]);
    if (distance < minimum) {
      minimum = distance;
    }
  }
  return minimum;
}

std::optional<double> circumscribed_radius(const Polygon2D & footprint)
{
  const std::vector<Eigen::Vector2d> & v = footprint.vertices();
  if (v.size() < 3) {
    return std::nullopt;
  }
  double maximum = 0.0;
  for (const Eigen::Vector2d & vertex : v) {
    const double norm = vertex.norm();
    if (norm > maximum) {
      maximum = norm;
    }
  }
  return maximum;
}

std::optional<DistanceTraversabilityModel> DistanceTraversabilityModel::from_footprint(
  const Polygon2D & footprint, double inflation_radius)
{
  const std::optional<double> inscribed = eltanin::inscribed_radius(footprint);
  const std::optional<double> circumscribed = eltanin::circumscribed_radius(footprint);
  if (!inscribed.has_value() || !circumscribed.has_value()) {
    return std::nullopt;
  }
  return from_radii(*inscribed, *circumscribed, inflation_radius);
}

std::optional<DistanceTraversabilityModel> DistanceTraversabilityModel::from_radii(
  double inscribed, double circumscribed, double inflation)
{
  if (!std::isfinite(inscribed) || !std::isfinite(circumscribed) || !std::isfinite(inflation)) {
    return std::nullopt;
  }
  if (inscribed < 0.0 || inscribed > circumscribed || circumscribed > inflation) {
    return std::nullopt;
  }
  return DistanceTraversabilityModel(inscribed, circumscribed, inflation);
}

}  // namespace eltanin
