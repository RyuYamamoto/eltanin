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

#ifndef ELTANIN_TEST__PLANNER__PLANNER_FIXTURE_HPP_
#define ELTANIN_TEST__PLANNER__PLANNER_FIXTURE_HPP_

#include <eltanin/core/footprint.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/core/traversability.hpp>
#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/cost_values.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/planner/astar_planner.hpp>
#include <eltanin/planner/path_smoother.hpp>

#include <gtest/gtest.h>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <numbers>
#include <string_view>

namespace eltanin_test
{

/// Cost threshold at which CostTraversabilityModel starts reporting Circumscribed.
inline constexpr std::uint8_t CIRCUMSCRIBED_COST = 128;

/// A cost inside [CIRCUMSCRIBED_COST, INSCRIBED_INFLATED_OBSTACLE), so it classifies Circumscribed.
inline constexpr std::uint8_t CIRCUMSCRIBED_BAND_COST = 200;

inline eltanin::map::CostTraversabilityModel make_cost_model(bool unknown_is_free = false)
{
  return eltanin::map::CostTraversabilityModel(CIRCUMSCRIBED_COST, unknown_is_free);
}

/// Radii used by every distance-map fixture; 0 < inscribed < circumscribed keeps all three bands.
inline eltanin::CollisionRadii make_radii()
{
  const auto radii = eltanin::CollisionRadii::from_radii(0.20, 0.40, 0.60);
  assert(radii.has_value());
  return *radii;
}

/// Rows are given top-down like make_costmap; '#' is Inscribed, 'c' Circumscribed, '.' Free.
inline eltanin::map::DistanceMap make_distance_map(
  std::initializer_list<std::string_view> rows, double resolution,
  const eltanin::CollisionRadii & radii,
  const Eigen::Vector2d & origin = Eigen::Vector2d::Zero())
{
  assert(rows.size() > 0);
  const int size_y = static_cast<int>(rows.size());
  const int size_x = static_cast<int>(rows.begin()->size());
  eltanin::map::DistanceMap map(
    eltanin::map::MapGeometry(size_x, size_y, resolution, origin), 0.0F);

  const auto free_distance = static_cast<float>(radii.inflation_radius());
  const auto band_distance =
    static_cast<float>(0.5 * (radii.inscribed_radius() + radii.circumscribed_radius()));

  int row_from_top = 0;
  for (const std::string_view row : rows) {
    assert(static_cast<int>(row.size()) == size_x);
    const int my = size_y - 1 - row_from_top;
    for (int mx = 0; mx < size_x; ++mx) {
      switch (row[static_cast<std::size_t>(mx)]) {
        case '.':
          map(mx, my) = free_distance;
          break;
        case 'c':
          map(mx, my) = band_distance;
          break;
        case '#':
          map(mx, my) = 0.0F;
          break;
        default:
          assert(false && "unsupported distance map fixture character");
          break;
      }
    }
    ++row_from_top;
  }
  return map;
}

/// Distance field whose CollisionRadii classification matches the costmap's cost classification.
inline eltanin::map::DistanceMap distance_map_from_costmap(
  const eltanin::map::Costmap & costmap, const eltanin::map::CostTraversabilityModel & model,
  const eltanin::CollisionRadii & radii)
{
  eltanin::map::DistanceMap map(costmap.geometry(), 0.0F);
  const auto free_distance = static_cast<float>(radii.inflation_radius());
  const auto band_distance =
    static_cast<float>(0.5 * (radii.inscribed_radius() + radii.circumscribed_radius()));
  for (std::size_t i = 0; i < costmap.cell_count(); ++i) {
    switch (model.classify(costmap[i])) {
      case eltanin::Traversability::Free:
        map[i] = free_distance;
        break;
      case eltanin::Traversability::Circumscribed:
        map[i] = band_distance;
        break;
      case eltanin::Traversability::Inscribed:
        map[i] = 0.0F;
        break;
    }
  }
  return map;
}

/// Sum of second-difference magnitudes; the smoothing term reduces exactly this quantity.
inline double smoothness_cost(const eltanin::Path & path)
{
  double cost = 0.0;
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    cost += (path[i - 1].position - 2.0 * path[i].position + path[i + 1].position).norm();
  }
  return cost;
}

/// The objective the smoothing sweep actually descends: a data term plus squared segment lengths.
inline double smoother_energy(
  const eltanin::Path & path, const eltanin::Path & original,
  const eltanin::planner::SmootherParams & params)
{
  double energy = 0.0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    energy += 0.5 * params.weight_data * (path[i].position - original[i].position).squaredNorm();
  }
  for (std::size_t i = 1; i < path.size(); ++i) {
    energy += 0.5 * params.weight_smooth * (path[i].position - path[i - 1].position).squaredNorm();
  }
  return energy;
}

/// The data-plus-smoothing sweep these expectations describe; the newer terms stay switched off.
inline eltanin::planner::SmootherParams basic_smoother_params(
  double weight_data, double weight_smooth, double tolerance, int max_iterations)
{
  eltanin::planner::SmootherParams params;
  params.weight_data = weight_data;
  params.weight_smooth = weight_smooth;
  params.weight_curvature = 0.0;
  params.weight_obstacle = 0.0;
  params.tolerance = tolerance;
  params.max_iterations = max_iterations;
  return params;
}

/// Smoothing and clearance shaping off; the octile length expectations describe the raw path.
inline eltanin::planner::AStarParams raw_astar_params(int start_search_radius_cells = 8)
{
  eltanin::planner::AStarParams params;
  params.common.start_search_radius_cells = start_search_radius_cells;
  params.smoother.reset();
  params.clearance.penalty = 0.0;
  return params;
}

/// Consecutive raw A* poses are neighbouring cell centers, so no step may exceed the diagonal.
inline void expect_grid_connected(const eltanin::Path & path, double resolution)
{
  const double limit = std::numbers::sqrt2 * resolution * (1.0 + 1e-9);
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double step = (path[i].position - path[i - 1].position).norm();
    EXPECT_LE(step, limit) << "step " << i << " is " << step;
    EXPECT_GT(step, 0.0) << "step " << i << " is degenerate";
  }
}

template <class Map, class Model>
void expect_all_free(const eltanin::Path & path, const Map & map, const Model & model)
{
  const auto & geometry = map.geometry();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto index = geometry.world_to_map(path[i].position);
    ASSERT_TRUE(index.has_value()) << "pose " << i << " is outside the map";
    EXPECT_EQ(model.classify(map(index->x, index->y)), eltanin::Traversability::Free)
      << "pose " << i << " is not traversable";
  }
}

inline void expect_same_path(const eltanin::Path & lhs, const eltanin::Path & rhs)
{
  ASSERT_EQ(lhs.size(), rhs.size());
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    EXPECT_EQ(lhs[i].position.x(), rhs[i].position.x()) << "pose " << i;
    EXPECT_EQ(lhs[i].position.y(), rhs[i].position.y()) << "pose " << i;
    EXPECT_EQ(lhs[i].yaw, rhs[i].yaw) << "pose " << i;
  }
}

}  // namespace eltanin_test

#endif  // ELTANIN_TEST__PLANNER__PLANNER_FIXTURE_HPP_
