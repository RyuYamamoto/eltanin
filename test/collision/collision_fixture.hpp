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

#ifndef ELTANIN_TEST__COLLISION__COLLISION_FIXTURE_HPP_
#define ELTANIN_TEST__COLLISION__COLLISION_FIXTURE_HPP_

#include <eltanin/core/footprint.hpp>
#include <eltanin/core/polygon.hpp>
#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/cost_values.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/map/layers/inflation_layer.hpp>
#include <eltanin/collision/velocity_limiter.hpp>

#include <algorithm>
#include <cassert>
#include <utility>
#include <vector>

namespace eltanin_test
{

/// Map plus the traversability model derived from the same distance_model the limiter uses.
struct CollisionScenario
{
  eltanin::map::Costmap map;
  eltanin::map::CostTraversabilityModel model;
};

inline constexpr double COST_SCALING_FACTOR = 3.0;

/// 0.6 m square, counter-clockwise; the default of VelocityLimiterParams.
inline eltanin::Polygon2D default_footprint()
{
  return eltanin::Polygon2D{
    Eigen::Vector2d{-0.3, 0.3}, Eigen::Vector2d{-0.3, -0.3}, Eigen::Vector2d{0.3, -0.3},
    Eigen::Vector2d{0.3, 0.3}};
}

inline eltanin::Polygon2D reversed_footprint(const eltanin::Polygon2D & footprint)
{
  std::vector<Eigen::Vector2d> vertices = footprint.vertices();
  std::reverse(vertices.begin(), vertices.end());
  return eltanin::Polygon2D(std::move(vertices));
}

inline eltanin::collision::VelocityLimiter make_limiter(
  const eltanin::collision::VelocityLimiterParams & params)
{
  const auto limiter = eltanin::collision::VelocityLimiter::create(params);
  assert(limiter.has_value());
  return *limiter;
}

/// Inflation model built from the very footprint the limiter checks, as required by C-1.
inline eltanin::map::InflationCostModel footprint_inflation_model(
  const eltanin::Polygon2D & footprint, double inflation_radius)
{
  const auto distance_model = eltanin::DistanceTraversabilityModel::from_footprint(footprint, inflation_radius);
  assert(distance_model.has_value());
  const auto model =
    eltanin::map::InflationCostModel::create(*distance_model, COST_SCALING_FACTOR);
  assert(model.has_value());
  return *model;
}

inline CollisionScenario inflate_into_scenario(
  eltanin::map::Costmap map, const eltanin::map::InflationCostModel & inflation)
{
  eltanin::map::InflationLayer layer(inflation);
  layer.update_costs(map);
  return CollisionScenario{
    std::move(map), eltanin::map::CostTraversabilityModel(inflation.circumscribed_cost())};
}

/// F-A: 1.2 m square map at 0.05 m with one lethal wall column; cell centres are 0.025 + 0.05 * i.
inline CollisionScenario wall_scenario(bool wall_on_right)
{
  const eltanin::map::InflationCostModel inflation =
    footprint_inflation_model(default_footprint(), 0.55);
  eltanin::map::Costmap map(
    eltanin::map::MapGeometry(24, 24, 0.05, Eigen::Vector2d::Zero()), eltanin::map::FREE_SPACE);
  const int wall_x = wall_on_right ? 23 : 0;
  for (int my = 0; my < map.size_y(); ++my) {
    map(wall_x, my) = eltanin::map::LETHAL_OBSTACLE;
  }
  return inflate_into_scenario(std::move(map), inflation);
}

/// F-B: one lethal cell offset from the robot cell (11, 11), used for the heading-dependent band.
inline CollisionScenario single_obstacle_scenario(int offset_x, int offset_y)
{
  const eltanin::map::InflationCostModel inflation =
    footprint_inflation_model(default_footprint(), 0.55);
  eltanin::map::Costmap map(
    eltanin::map::MapGeometry(24, 24, 0.05, Eigen::Vector2d::Zero()), eltanin::map::FREE_SPACE);
  map(11 + offset_x, 11 + offset_y) = eltanin::map::LETHAL_OBSTACLE;
  return inflate_into_scenario(std::move(map), inflation);
}

/// F-E: same geometry as F-B but with no inflation at all, the shape collision_predictor sees.
inline CollisionScenario uninflated_scenario(int offset_x, int offset_y)
{
  eltanin::map::Costmap map(
    eltanin::map::MapGeometry(24, 24, 0.05, Eigen::Vector2d::Zero()), eltanin::map::FREE_SPACE);
  map(11 + offset_x, 11 + offset_y) = eltanin::map::LETHAL_OBSTACLE;
  // Any threshold in (FREE_SPACE, INSCRIBED_INFLATED_OBSTACLE] classifies the three values alike.
  return CollisionScenario{std::move(map), eltanin::map::CostTraversabilityModel(1)};
}

/// F-C: 0.25 m cells so that every cell centre 0.125 + 0.25 * i is exact in binary.
inline eltanin::Polygon2D boundary_footprint()
{
  return eltanin::Polygon2D{
    Eigen::Vector2d{-0.5, 0.5}, Eigen::Vector2d{-0.5, -0.5}, Eigen::Vector2d{0.5, -0.5},
    Eigen::Vector2d{0.5, 0.5}};
}

inline CollisionScenario boundary_scenario(int obstacle_x, int obstacle_y)
{
  const eltanin::map::InflationCostModel inflation =
    footprint_inflation_model(boundary_footprint(), 1.0);
  eltanin::map::Costmap map(
    eltanin::map::MapGeometry(8, 8, 0.25, Eigen::Vector2d::Zero()), eltanin::map::FREE_SPACE);
  map(obstacle_x, obstacle_y) = eltanin::map::LETHAL_OBSTACLE;
  return inflate_into_scenario(std::move(map), inflation);
}

/// F-D: no obstacle anywhere; the prediction horizon of 1.0 m stays inside the map.
inline CollisionScenario free_scenario()
{
  const eltanin::map::InflationCostModel inflation =
    footprint_inflation_model(default_footprint(), 0.55);
  eltanin::map::Costmap map(
    eltanin::map::MapGeometry(24, 24, 0.05, Eigen::Vector2d::Zero()), eltanin::map::FREE_SPACE);
  return inflate_into_scenario(std::move(map), inflation);
}

}  // namespace eltanin_test

#endif  // ELTANIN_TEST__COLLISION__COLLISION_FIXTURE_HPP_
