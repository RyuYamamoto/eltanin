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

#include <eltanin/map/cost_values.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/planner/traversable_search.hpp>

#include <planner/planner_fixture.hpp>

#include <gtest/gtest.h>

#include <climits>

namespace
{

using Eigen::Vector2d;
using eltanin::map::Costmap;
using eltanin::map::FREE_SPACE;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::MapGeometry;
using eltanin::map::MapIndex;
using eltanin::planner::find_nearest_traversable;
using eltanin_test::CIRCUMSCRIBED_BAND_COST;
using eltanin_test::make_cost_model;

constexpr double RESOLUTION = 0.1;

/// Everything blocked; the tests open exactly the cells they care about.
Costmap blocked_map(int size_x, int size_y)
{
  return Costmap(MapGeometry(size_x, size_y, RESOLUTION, Vector2d::Zero()), LETHAL_OBSTACLE);
}

}  // namespace

TEST(TraversableSearch, ReturnsSelfWhenFree)
{
  Costmap map = blocked_map(9, 9);
  map(4, 4) = FREE_SPACE;
  map(4, 5) = FREE_SPACE;

  const auto found = find_nearest_traversable(map, make_cost_model(), MapIndex{4, 4}, 8);
  ASSERT_TRUE(found.has_value());
  EXPECT_EQ(found->x, 4);
  EXPECT_EQ(found->y, 4);
}

TEST(TraversableSearch, FindsEuclideanNearestNotChebyshev)
{
  Costmap map = blocked_map(9, 9);
  map(7, 7) = FREE_SPACE;
  map(4, 8) = FREE_SPACE;

  const auto found = find_nearest_traversable(map, make_cost_model(), MapIndex{4, 4}, 8);
  ASSERT_TRUE(found.has_value());
  EXPECT_EQ(found->x, 4);
  EXPECT_EQ(found->y, 8);
}

TEST(TraversableSearch, StopsAtTheRingThatCanNoLongerImprove)
{
  Costmap map = blocked_map(9, 9);
  map(5, 5) = FREE_SPACE;
  map(4, 6) = FREE_SPACE;

  const auto found = find_nearest_traversable(map, make_cost_model(), MapIndex{4, 4}, 8);
  ASSERT_TRUE(found.has_value());
  EXPECT_EQ(found->x, 5);
  EXPECT_EQ(found->y, 5);
}

TEST(TraversableSearch, TieBreakIsDeterministic)
{
  Costmap map = blocked_map(9, 9);
  map(3, 3) = FREE_SPACE;
  map(5, 5) = FREE_SPACE;
  map(3, 5) = FREE_SPACE;
  map(5, 3) = FREE_SPACE;

  const auto first = find_nearest_traversable(map, make_cost_model(), MapIndex{4, 4}, 8);
  const auto second = find_nearest_traversable(map, make_cost_model(), MapIndex{4, 4}, 8);
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->x, 3);
  EXPECT_EQ(first->y, 3);
  EXPECT_EQ(second->x, first->x);
  EXPECT_EQ(second->y, first->y);
}

TEST(TraversableSearch, RespectsMaxRadius)
{
  Costmap map = blocked_map(9, 9);
  map(8, 4) = FREE_SPACE;

  EXPECT_FALSE(find_nearest_traversable(map, make_cost_model(), MapIndex{4, 4}, 3).has_value());
  const auto found = find_nearest_traversable(map, make_cost_model(), MapIndex{4, 4}, 4);
  ASSERT_TRUE(found.has_value());
  EXPECT_EQ(found->x, 8);
  EXPECT_EQ(found->y, 4);
}

TEST(TraversableSearch, ZeroRadiusChecksOnlyItself)
{
  Costmap map = blocked_map(9, 9);
  map(5, 4) = FREE_SPACE;

  EXPECT_FALSE(find_nearest_traversable(map, make_cost_model(), MapIndex{4, 4}, 0).has_value());
  const auto found = find_nearest_traversable(map, make_cost_model(), MapIndex{5, 4}, 0);
  ASSERT_TRUE(found.has_value());
  EXPECT_EQ(found->x, 5);
  EXPECT_EQ(found->y, 4);
}

TEST(TraversableSearch, AcceptsOutOfBoundsFrom)
{
  Costmap map = blocked_map(9, 9);
  map(0, 2) = FREE_SPACE;

  const auto found = find_nearest_traversable(map, make_cost_model(), MapIndex{-3, 2}, 3);
  ASSERT_TRUE(found.has_value());
  EXPECT_EQ(found->x, 0);
  EXPECT_EQ(found->y, 2);
  EXPECT_FALSE(find_nearest_traversable(map, make_cost_model(), MapIndex{-3, 2}, 2).has_value());
}

TEST(TraversableSearch, HandlesSaturatedFrom)
{
  Costmap map = blocked_map(9, 9);
  map(0, 0) = FREE_SPACE;

  EXPECT_FALSE(
    find_nearest_traversable(map, make_cost_model(), MapIndex{INT_MIN, INT_MAX}, 8).has_value());
  EXPECT_FALSE(
    find_nearest_traversable(map, make_cost_model(), MapIndex{INT_MAX, INT_MIN}, 8).has_value());
}

TEST(TraversableSearch, AllBlockedReturnsNullopt)
{
  const Costmap map = blocked_map(9, 9);
  EXPECT_FALSE(find_nearest_traversable(map, make_cost_model(), MapIndex{4, 4}, 8).has_value());
}

TEST(TraversableSearch, CircumscribedIsNotTraversable)
{
  Costmap map = blocked_map(9, 9);
  map(4, 5) = CIRCUMSCRIBED_BAND_COST;
  map(4, 7) = FREE_SPACE;

  const auto found = find_nearest_traversable(map, make_cost_model(), MapIndex{4, 4}, 8);
  ASSERT_TRUE(found.has_value());
  EXPECT_EQ(found->x, 4);
  EXPECT_EQ(found->y, 7);
}

TEST(TraversableSearch, UnknownIsFreeIsHonoured)
{
  Costmap map = blocked_map(9, 9);
  map(4, 5) = eltanin::map::NO_INFORMATION;

  EXPECT_FALSE(
    find_nearest_traversable(map, make_cost_model(false), MapIndex{4, 4}, 8).has_value());
  const auto found = find_nearest_traversable(map, make_cost_model(true), MapIndex{4, 4}, 8);
  ASSERT_TRUE(found.has_value());
  EXPECT_EQ(found->x, 4);
  EXPECT_EQ(found->y, 5);
}
