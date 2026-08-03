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

#include <eltanin/core/angle.hpp>
#include <eltanin/map/cost_values.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/planner/astar_planner.hpp>
#include <eltanin/planner/dubins_path.hpp>
#include <eltanin/planner/hybrid_astar_planner.hpp>
#include <eltanin/planner/planner.hpp>

#include <map/costmap_fixture.hpp>
#include <planner/planner_fixture.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>

namespace
{

using Eigen::Vector2d;
using eltanin::Path;
using eltanin::Pose2D;
using eltanin::shortest_angular_distance;
using eltanin::map::Costmap;
using eltanin::map::FREE_SPACE;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::MapGeometry;
using eltanin::planner::AStarPlanner;
using eltanin::planner::HybridAStarParams;
using eltanin::planner::HybridAStarPlanner;
using eltanin::planner::Planner;
using eltanin::planner::plan_hybrid_astar;
using eltanin_test::expect_all_free;
using eltanin_test::expect_same_path;
using eltanin_test::make_cost_model;

constexpr double RESOLUTION = 0.1;
constexpr double TOLERANCE = 1e-9;

Costmap open_map(int size_x, int size_y)
{
  return Costmap(MapGeometry(size_x, size_y, RESOLUTION, Vector2d::Zero()), FREE_SPACE);
}

Pose2D at_cell(const Costmap & map, int mx, int my, double yaw = 0.0)
{
  return Pose2D{map.geometry().map_to_world(mx, my), yaw};
}

void expect_goal(const Path & path, const Pose2D & goal)
{
  ASSERT_FALSE(path.empty());
  const Pose2D & last = path[path.size() - 1];
  EXPECT_NEAR((last.position - goal.position).norm(), 0.0, TOLERANCE);
  EXPECT_NEAR(shortest_angular_distance(last.yaw, goal.yaw), 0.0, TOLERANCE);
}

}  // namespace

TEST(HybridAStarPlanner, PlansStraightForwardWithoutChangingHeading)
{
  const Costmap map = open_map(30, 12);
  const Pose2D start = at_cell(map, 2, 6, 0.0);
  const Pose2D goal = at_cell(map, 24, 6, 0.0);

  const auto path = plan_hybrid_astar(map, make_cost_model(), start, goal);

  ASSERT_TRUE(path.has_value());
  ASSERT_GE(path->size(), 2u);
  expect_goal(*path, goal);
  expect_all_free(*path, map, make_cost_model());
  for (const Pose2D & pose : *path) {
    EXPECT_NEAR(pose.position.y(), start.position.y(), TOLERANCE);
    EXPECT_NEAR(pose.yaw, 0.0, TOLERANCE);
  }
}

TEST(HybridAStarPlanner, ReturnsOnePoseWhenStartEqualsGoal)
{
  const Costmap map = open_map(12, 12);
  const Pose2D pose = at_cell(map, 6, 6, 0.3);

  const auto path = plan_hybrid_astar(map, make_cost_model(), pose, pose);

  ASSERT_TRUE(path.has_value());
  ASSERT_EQ(path->size(), 1u);
  expect_goal(*path, pose);
}

TEST(HybridAStarPlanner, ConstructorRejectsInvalidParameters)
{
  HybridAStarParams params;
  params.common.start_search_radius_cells = -1;
  EXPECT_THROW(HybridAStarPlanner{params}, std::invalid_argument);

  params = HybridAStarParams{};
  params.heading_bins = 7;
  EXPECT_THROW(HybridAStarPlanner{params}, std::invalid_argument);

  constexpr std::array<double HybridAStarParams::*, 6> scalar_parameters{
    &HybridAStarParams::minimum_turning_radius, &HybridAStarParams::motion_step,
    &HybridAStarParams::collision_check_step, &HybridAStarParams::dubins_expansion_distance,
    &HybridAStarParams::steering_penalty, &HybridAStarParams::steering_change_penalty};
  for (double HybridAStarParams::* const member : scalar_parameters) {
    params = HybridAStarParams{};
    params.*member = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(HybridAStarPlanner{params}, std::invalid_argument);
  }

  params = HybridAStarParams{};
  params.minimum_turning_radius = 0.0;
  EXPECT_THROW(HybridAStarPlanner{params}, std::invalid_argument);
  params = HybridAStarParams{};
  params.dubins_expansion_distance = 0.0;
  EXPECT_THROW(HybridAStarPlanner{params}, std::invalid_argument);
}

TEST(HybridAStarPlanner, ChangesHeadingWithBoundedCurvature)
{
  const Costmap map = open_map(32, 32);
  const Pose2D start = at_cell(map, 5, 5, 0.0);
  const Pose2D goal = at_cell(map, 22, 22, std::numbers::pi / 2.0);
  HybridAStarParams params;
  params.minimum_turning_radius = 0.4;

  const auto path = plan_hybrid_astar(map, make_cost_model(), start, goal, params);

  ASSERT_TRUE(path.has_value());
  ASSERT_GE(path->size(), 3u);
  expect_goal(*path, goal);
  expect_all_free(*path, map, make_cost_model());
  const double radius = params.minimum_turning_radius;
  for (std::size_t i = 0; i + 1 < path->size(); ++i) {
    const double chord = ((*path)[i + 1].position - (*path)[i].position).norm();
    const double delta_yaw =
      std::abs(shortest_angular_distance((*path)[i].yaw, (*path)[i + 1].yaw));
    // Largest heading change a chord of this length can cover at the minimum turning radius.
    const double turn_limit = 2.0 * std::asin(std::min(1.0, chord / (2.0 * radius)));
    EXPECT_LE(delta_yaw, turn_limit + TOLERANCE) << "step " << i << " chord " << chord;
  }
}

TEST(HybridAStarPlanner, DetoursAroundAWall)
{
  Costmap map = open_map(36, 30);
  for (int my = 0; my < 21; ++my) {
    map(18, my) = LETHAL_OBSTACLE;
  }
  const Pose2D start = at_cell(map, 5, 10, 0.0);
  const Pose2D goal = at_cell(map, 30, 10, 0.0);

  const auto path = plan_hybrid_astar(map, make_cost_model(), start, goal);

  ASSERT_TRUE(path.has_value());
  expect_goal(*path, goal);
  expect_all_free(*path, map, make_cost_model());
  bool passes_gap = false;
  for (const Pose2D & pose : *path) {
    passes_gap = passes_gap || pose.position.y() > map.geometry().map_to_world(0, 20).y();
  }
  EXPECT_TRUE(passes_gap);
}

TEST(HybridAStarPlanner, RejectsAMapSplitByAWall)
{
  Costmap map = open_map(24, 16);
  for (int my = 0; my < map.geometry().size_y(); ++my) {
    map(12, my) = LETHAL_OBSTACLE;
  }

  EXPECT_FALSE(
    plan_hybrid_astar(
      map, make_cost_model(), at_cell(map, 4, 8, 0.0), at_cell(map, 19, 8, 0.0))
      .has_value());
}

TEST(HybridAStarPlanner, SamplesForCollisionBetweenPrimitiveEndpoints)
{
  Costmap map = open_map(12, 1);
  map(3, 0) = LETHAL_OBSTACLE;
  HybridAStarParams params;
  params.motion_step = 0.4;
  params.collision_check_step = 0.05;

  EXPECT_FALSE(
    plan_hybrid_astar(
      map, make_cost_model(), at_cell(map, 1, 0, 0.0), at_cell(map, 9, 0, 0.0), params)
      .has_value());
}

TEST(HybridAStarPlanner, RespectsTheExpansionLimit)
{
  const Costmap map = open_map(24, 12);
  Costmap blocked = map;
  blocked(10, 6) = LETHAL_OBSTACLE;
  HybridAStarParams params;
  params.max_expansions = 1;

  EXPECT_FALSE(
    plan_hybrid_astar(
      blocked, make_cost_model(), at_cell(blocked, 2, 6, 0.0), at_cell(blocked, 20, 6, 0.0),
      params)
      .has_value());
}

TEST(HybridAStarPlanner, RejectsNonFiniteYaw)
{
  const Costmap map = open_map(12, 12);
  const double nan = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(
    plan_hybrid_astar(
      map, make_cost_model(), at_cell(map, 2, 6, nan), at_cell(map, 9, 6, 0.0))
      .has_value());
  EXPECT_FALSE(
    plan_hybrid_astar(
      map, make_cost_model(), at_cell(map, 2, 6, 0.0), at_cell(map, 9, 6, nan))
      .has_value());
}

TEST(HybridAStarPlanner, SharesBlockedStartRescueWithAStar)
{
  Costmap map = open_map(20, 12);
  map(2, 6) = LETHAL_OBSTACLE;
  const Pose2D blocked_start = at_cell(map, 2, 6, 0.0);
  const Pose2D goal = at_cell(map, 16, 6, 0.0);
  HybridAStarParams params;
  params.common.start_search_radius_cells = 1;

  const auto path = plan_hybrid_astar(map, make_cost_model(), blocked_start, goal, params);

  ASSERT_TRUE(path.has_value());
  ASSERT_FALSE(path->empty());
  const auto rescued_index = map.geometry().world_to_map((*path)[0].position);
  ASSERT_TRUE(rescued_index.has_value());
  EXPECT_FALSE(rescued_index->x == 2 && rescued_index->y == 6);
  expect_all_free(*path, map, make_cost_model());
}

TEST(PlannerInterface, SelectsPlannerAtRuntimeThroughBaseReference)
{
  const Costmap map = open_map(24, 12);
  const Pose2D start = at_cell(map, 2, 6, 0.0);
  const Pose2D goal = at_cell(map, 20, 6, 0.0);

  std::unique_ptr<Planner> selected = std::make_unique<AStarPlanner>();
  const auto astar_path = selected->plan(map, make_cost_model(), start, goal);
  selected = std::make_unique<HybridAStarPlanner>();
  const auto hybrid_path = selected->plan(map, make_cost_model(), start, goal);

  ASSERT_TRUE(astar_path.has_value());
  ASSERT_TRUE(hybrid_path.has_value());
  expect_goal(*astar_path, goal);
  expect_goal(*hybrid_path, goal);
}

TEST(HybridAStarPlanner, IsDeterministic)
{
  const Costmap map = open_map(28, 28);
  const Pose2D start = at_cell(map, 4, 4, 0.0);
  const Pose2D goal = at_cell(map, 20, 20, std::numbers::pi / 2.0);

  const auto first = plan_hybrid_astar(map, make_cost_model(), start, goal);
  const auto second = plan_hybrid_astar(map, make_cost_model(), start, goal);

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  expect_same_path(*first, *second);
}

TEST(HybridAStarPlanner, KeepsTheOutputSpacingWithinOneAndAHalfMotionSteps)
{
  const Costmap map = open_map(40, 40);
  const double motion_step = std::numbers::sqrt2 * RESOLUTION;

  // The Dubins tail length depends on where the search lands, so several goals are swept.
  for (int offset = 0; offset < 7; ++offset) {
    const Pose2D start = at_cell(map, 4, 20, 0.0);
    const Pose2D goal{
      at_cell(map, 30, 20).position + Vector2d{0.013 * static_cast<double>(offset), 0.0}, 0.4};
    const auto path = plan_hybrid_astar(map, make_cost_model(), start, goal);
    ASSERT_TRUE(path.has_value()) << "offset " << offset;
    ASSERT_GE(path->size(), 3u) << "offset " << offset;

    double shortest = std::numeric_limits<double>::infinity();
    double longest = 0.0;
    for (std::size_t i = 1; i < path->size(); ++i) {
      const double step = ((*path)[i].position - (*path)[i - 1].position).norm();
      EXPECT_GT(step, 0.0) << "offset " << offset << " step " << i;
      shortest = std::min(shortest, step);
      longest = std::max(longest, step);
    }
    EXPECT_LE(longest, 1.5 * motion_step + TOLERANCE) << "offset " << offset;
    EXPECT_LE(longest / shortest, 1.5 + TOLERANCE) << "offset " << offset;
  }
}

TEST(HybridAStarPlanner, EveryReturnedYawIsFinite)
{
  const Costmap map = open_map(28, 28);
  const eltanin::planner::PlanResult results[] = {
    plan_hybrid_astar(
      map, make_cost_model(), at_cell(map, 4, 4, 0.0), at_cell(map, 20, 20, std::numbers::pi / 2.0)),
    plan_hybrid_astar(map, make_cost_model(), at_cell(map, 6, 6, 0.3), at_cell(map, 6, 6, 0.3))};

  for (const eltanin::planner::PlanResult & result : results) {
    ASSERT_TRUE(result.has_value());
    for (std::size_t i = 0; i < result->size(); ++i) {
      EXPECT_TRUE(std::isfinite((*result)[i].yaw)) << "pose " << i;
      EXPECT_TRUE(result->operator[](i).position.allFinite()) << "pose " << i;
    }
  }
}

TEST(PlannerInterface, BothPlannersMeetTheOutputContractWithoutPostProcessing)
{
  Costmap map = open_map(36, 30);
  for (int my = 0; my < 21; ++my) {
    map(18, my) = LETHAL_OBSTACLE;
  }
  const Pose2D start = at_cell(map, 5, 10, 0.0);
  const Pose2D goal = at_cell(map, 30, 10, 0.4);
  const double spacing_limit = 1.5 * std::numbers::sqrt2 * RESOLUTION + TOLERANCE;

  std::unique_ptr<Planner> planners[2];
  planners[0] = std::make_unique<AStarPlanner>();
  planners[1] = std::make_unique<HybridAStarPlanner>();

  for (const std::unique_ptr<Planner> & planner : planners) {
    const auto path = planner->plan(map, make_cost_model(), start, goal);
    ASSERT_TRUE(path.has_value()) << eltanin::planner::to_string(path.error());
    ASSERT_GE(path->size(), 2u);
    expect_all_free(*path, map, make_cost_model());
    EXPECT_NEAR(
      ((*path)[path->size() - 1].position - goal.position).norm(), 0.0,
      std::numbers::sqrt2 * 0.5 * RESOLUTION + TOLERANCE);
    for (std::size_t i = 0; i < path->size(); ++i) {
      EXPECT_TRUE(std::isfinite((*path)[i].yaw)) << "pose " << i;
    }
    for (std::size_t i = 1; i < path->size(); ++i) {
      const double step = ((*path)[i].position - (*path)[i - 1].position).norm();
      EXPECT_GT(step, 0.0) << "step " << i;
      EXPECT_LE(step, spacing_limit) << "step " << i;
    }
  }
}

TEST(HybridAStarPlanner, ReturnsTheOptimalDubinsPathOnAnOpenMap)
{
  const Costmap map = open_map(120, 120);
  const Pose2D start = at_cell(map, 10, 60, 0.0);
  const double motion_step = std::numbers::sqrt2 * RESOLUTION;

  for (int goal_x = 40; goal_x < 110; goal_x += 10) {
    for (int quadrant = 0; quadrant < 8; ++quadrant) {
      const Pose2D goal = at_cell(map, goal_x, 60, quadrant * std::numbers::pi / 4.0);
      const auto path = plan_hybrid_astar(map, make_cost_model(), start, goal);
      ASSERT_TRUE(path.has_value()) << "goal_x " << goal_x << " quadrant " << quadrant;
      const auto dubins = eltanin::planner::solve_dubins_path(start, goal, 0.4);
      ASSERT_TRUE(dubins.has_value());

      double length = 0.0;
      double turning = 0.0;
      for (std::size_t i = 1; i < path->size(); ++i) {
        length += ((*path)[i].position - (*path)[i - 1].position).norm();
        turning += std::abs(shortest_angular_distance((*path)[i - 1].yaw, (*path)[i].yaw));
      }
      // Chords are shorter than the arc, so the sampled length only ever undershoots.
      EXPECT_LE(length, dubins->length() + motion_step);
      // A curvature-limited forward path cannot turn less than the shortest Dubins path does.
      EXPECT_LE(turning, dubins->length() / 0.4 + TOLERANCE)
        << "goal_x " << goal_x << " quadrant " << quadrant;
      expect_goal(*path, goal);
    }
  }
}
