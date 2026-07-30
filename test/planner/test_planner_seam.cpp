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
#include <eltanin/core/path.hpp>
#include <eltanin/core/traversability.hpp>
#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/planner/astar_planner.hpp>
#include <eltanin/planner/path_smoother.hpp>
#include <eltanin/planner/traversable_search.hpp>

#include <map/costmap_fixture.hpp>
#include <planner/planner_fixture.hpp>

#include <gtest/gtest.h>

#include <numbers>

namespace
{

using Eigen::Vector2d;
using eltanin::CollisionRadii;
using eltanin::Path;
using eltanin::Pose2D;
using eltanin::Traversability;
using eltanin::path_length;
using eltanin::map::Costmap;
using eltanin::map::CostTraversabilityModel;
using eltanin::map::DistanceMap;
using eltanin::map::MapIndex;
using eltanin::planner::CellMap;
using eltanin::planner::SmootherParams;
using eltanin::planner::find_nearest_traversable;
using eltanin::planner::plan;
using eltanin::planner::smooth;
using eltanin_test::distance_map_from_costmap;
using eltanin_test::expect_same_path;
using eltanin_test::make_cost_model;
using eltanin_test::make_distance_map;
using eltanin_test::make_radii;

constexpr double RESOLUTION = 0.1;
constexpr double TOLERANCE = 1e-9;

static_assert(eltanin::TraversabilityModel<CostTraversabilityModel, Costmap::value_type>);
static_assert(eltanin::TraversabilityModel<CollisionRadii, DistanceMap::value_type>);
static_assert(CellMap<Costmap>);
static_assert(CellMap<DistanceMap>);

}  // namespace

TEST(PlannerSeam, PlansOnACostmap)
{
  const Costmap map = eltanin_test::make_costmap(
    {
      ".....",
      "..#..",
      "..#..",
      "..#..",
      ".....",
    },
    RESOLUTION);
  const auto path = plan(
    map, make_cost_model(), Pose2D{map.geometry().map_to_world(0, 2), 0.0},
    Pose2D{map.geometry().map_to_world(4, 2), 0.0});
  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path_length(*path), (4.0 + 2.0 * std::numbers::sqrt2) * RESOLUTION, TOLERANCE);
}

TEST(PlannerSeam, PlansOnAHandBuiltDistanceField)
{
  const CollisionRadii radii = make_radii();
  const DistanceMap map = make_distance_map(
    {
      ".....",
      "..#..",
      "..#..",
      "..#..",
      ".....",
    },
    RESOLUTION, radii);
  const auto path = plan(
    map, radii, Pose2D{map.geometry().map_to_world(0, 2), 0.0},
    Pose2D{map.geometry().map_to_world(4, 2), 0.0});
  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path_length(*path), (4.0 + 2.0 * std::numbers::sqrt2) * RESOLUTION, TOLERANCE);
}

TEST(PlannerSeam, BothModelsProduceTheSamePath)
{
  const Costmap costmap = eltanin_test::make_costmap(
    {
      "........",
      "...##...",
      "...##...",
      "...##...",
      "...##...",
      "........",
      "..#####.",
      "........",
    },
    RESOLUTION);
  const CollisionRadii radii = make_radii();
  const DistanceMap distance_map = distance_map_from_costmap(costmap, make_cost_model(), radii);

  const Pose2D start{costmap.geometry().map_to_world(0, 0), 0.4};
  const Pose2D goal{costmap.geometry().map_to_world(7, 5), -1.1};

  const auto from_cost = plan(costmap, make_cost_model(), start, goal);
  const auto from_distance = plan(distance_map, radii, start, goal);
  ASSERT_TRUE(from_cost.has_value());
  ASSERT_TRUE(from_distance.has_value());
  expect_same_path(*from_cost, *from_distance);

  const SmootherParams params{0.5, 0.3, 0.0, 100};
  expect_same_path(
    smooth(*from_cost, costmap, make_cost_model(), params),
    smooth(*from_distance, distance_map, radii, params));
}

TEST(PlannerSeam, FindsTheNearestTraversableCellWithBothModels)
{
  const Costmap costmap = eltanin_test::make_costmap(
    {
      ".....",
      "..#..",
      ".###.",
      "..#..",
      ".....",
    },
    RESOLUTION);
  const CollisionRadii radii = make_radii();
  const DistanceMap distance_map = distance_map_from_costmap(costmap, make_cost_model(), radii);

  const auto from_cost = find_nearest_traversable(costmap, make_cost_model(), MapIndex{2, 2}, 4);
  const auto from_distance = find_nearest_traversable(distance_map, radii, MapIndex{2, 2}, 4);
  ASSERT_TRUE(from_cost.has_value());
  ASSERT_TRUE(from_distance.has_value());
  EXPECT_EQ(from_cost->x, from_distance->x);
  EXPECT_EQ(from_cost->y, from_distance->y);
}

TEST(PlannerSeam, CircumscribedBandOfTheDistanceFieldBlocksTheSearch)
{
  const CollisionRadii radii = make_radii();
  const DistanceMap map = make_distance_map(
    {
      "..c..",
      "..c..",
      "..c..",
      "..c..",
      "..c..",
    },
    RESOLUTION, radii);
  for (int my = 0; my < map.size_y(); ++my) {
    EXPECT_EQ(radii.classify(map(2, my)), Traversability::Circumscribed);
  }
  EXPECT_FALSE(plan(
                 map, radii, Pose2D{map.geometry().map_to_world(0, 0), 0.0},
                 Pose2D{map.geometry().map_to_world(4, 4), 0.0})
                 .has_value());
}

TEST(PlannerSeam, SmoothsOnBothModels)
{
  const Costmap costmap = eltanin_test::make_costmap(
    {
      ".....",
      ".....",
      ".....",
      ".....",
      ".....",
    },
    RESOLUTION);
  const CollisionRadii radii = make_radii();
  const DistanceMap distance_map = distance_map_from_costmap(costmap, make_cost_model(), radii);

  Path input;
  input.push_back(Pose2D{Vector2d{0.05, 0.25}, 0.0});
  input.push_back(Pose2D{Vector2d{0.15, 0.35}, 0.0});
  input.push_back(Pose2D{Vector2d{0.25, 0.25}, 0.0});
  input.push_back(Pose2D{Vector2d{0.35, 0.35}, 0.0});
  input.push_back(Pose2D{Vector2d{0.45, 0.25}, 0.7});

  expect_same_path(
    smooth(input, costmap, make_cost_model()), smooth(input, distance_map, radii));
}
