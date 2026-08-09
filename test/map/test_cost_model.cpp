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

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

namespace
{

using eltanin::DistanceTraversabilityModel;
using eltanin::Traversability;
using eltanin::map::CostTraversabilityModel;
using eltanin::map::FREE_SPACE;
using eltanin::map::InflationCostModel;
using eltanin::map::INSCRIBED_INFLATED_OBSTACLE;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::MAX_NON_OBSTACLE;
using eltanin::map::NO_INFORMATION;

DistanceTraversabilityModel distance_model(double inscribed = 0.3, double circumscribed = 0.5, double inflation = 1.0)
{
  const auto value = DistanceTraversabilityModel::from_radii(inscribed, circumscribed, inflation);
  return *value;
}

InflationCostModel model(double cost_scaling_factor = 3.0, const DistanceTraversabilityModel & r = distance_model())
{
  const auto value = InflationCostModel::create(r, cost_scaling_factor);
  return *value;
}

}  // namespace

TEST(CostModel, CreateRejectsInvalidScalingFactor)
{
  EXPECT_TRUE(InflationCostModel::create(distance_model(), 0.0).has_value());
  EXPECT_TRUE(InflationCostModel::create(distance_model(), 3.0).has_value());
  EXPECT_FALSE(InflationCostModel::create(distance_model(), -1.0).has_value());
  EXPECT_FALSE(
    InflationCostModel::create(distance_model(), std::numeric_limits<double>::quiet_NaN()).has_value());
  EXPECT_FALSE(
    InflationCostModel::create(distance_model(), std::numeric_limits<double>::infinity()).has_value());
}

TEST(CostModel, CostAtKeyDistances)
{
  const InflationCostModel inflation = model();
  EXPECT_EQ(inflation.cost_at_distance(0.0), INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(inflation.cost_at_distance(0.299), INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(inflation.cost_at_distance(0.3), MAX_NON_OBSTACLE);

  const std::uint8_t at_circumscribed = static_cast<std::uint8_t>(
    static_cast<double>(MAX_NON_OBSTACLE) * std::exp(-3.0 * 0.2));
  EXPECT_EQ(inflation.cost_at_distance(0.5), at_circumscribed);

  const std::uint8_t at_inflation = static_cast<std::uint8_t>(
    static_cast<double>(MAX_NON_OBSTACLE) * std::exp(-3.0 * 0.7));
  EXPECT_EQ(inflation.cost_at_distance(1.0), at_inflation);
  EXPECT_GT(at_inflation, FREE_SPACE);

  EXPECT_EQ(inflation.cost_at_distance(1.0001), FREE_SPACE);
  EXPECT_EQ(inflation.cost_at_distance(100.0), FREE_SPACE);
}

TEST(CostModel, CostIsMonotonicallyNonIncreasing)
{
  const InflationCostModel inflation = model();
  int previous = 256;
  for (int i = 0; i <= 15000; ++i) {
    const double distance = 0.0001 * static_cast<double>(i);
    const int cost = inflation.cost_at_distance(distance);
    ASSERT_LE(cost, previous) << "distance=" << distance;
    previous = cost;
  }
  EXPECT_EQ(previous, FREE_SPACE);
}

TEST(CostModel, CostNeverSaturatesAboveMaxNonObstacle)
{
  const InflationCostModel inflation = model(0.0);
  EXPECT_EQ(inflation.cost_at_distance(0.3), MAX_NON_OBSTACLE);
  EXPECT_EQ(inflation.cost_at_distance(1.0), MAX_NON_OBSTACLE);
  EXPECT_LT(inflation.cost_at_distance(0.3), INSCRIBED_INFLATED_OBSTACLE);
}

TEST(CostModel, CircumscribedThresholdFallsAsScalingFactorRises)
{
  const std::uint8_t gentle = model(1.0).circumscribed_cost();
  const std::uint8_t medium = model(3.0).circumscribed_cost();
  const std::uint8_t steep = model(10.0).circumscribed_cost();
  EXPECT_GT(gentle, medium);
  EXPECT_GT(medium, steep);
  EXPECT_EQ(model(3.0).circumscribed_cost(), model(3.0).cost_at_distance(0.5));
}

TEST(CostModel, CircumscribedThresholdIsClampedAboveZero)
{
  const InflationCostModel steep = model(200.0);
  EXPECT_EQ(steep.cost_at_distance(0.5), FREE_SPACE);
  EXPECT_EQ(steep.circumscribed_cost(), 1);

  const CostTraversabilityModel traversability(steep.circumscribed_cost());
  EXPECT_EQ(traversability.classify(FREE_SPACE), Traversability::Free);
}

TEST(CostModel, CircumscribedThresholdIsIndependentOfInflationRadius)
{
  const std::uint8_t near = model(3.0, distance_model(0.3, 0.5, 0.6)).circumscribed_cost();
  const std::uint8_t far = model(3.0, distance_model(0.3, 0.5, 4.0)).circumscribed_cost();
  EXPECT_EQ(near, far);
}

TEST(CostModel, CostThresholdShiftsWithInscribedRadius)
{
  const std::uint8_t small_robot = model(3.0, distance_model(0.1, 0.5, 1.0)).circumscribed_cost();
  const std::uint8_t large_robot = model(3.0, distance_model(0.4, 0.5, 1.0)).circumscribed_cost();
  EXPECT_LT(small_robot, large_robot);
}

TEST(CostModel, CostModelBoundaries)
{
  const std::uint8_t threshold = model().circumscribed_cost();
  ASSERT_GT(threshold, 1);
  ASSERT_LT(threshold, MAX_NON_OBSTACLE);
  const CostTraversabilityModel traversability(threshold);

  EXPECT_EQ(traversability.classify(LETHAL_OBSTACLE), Traversability::Inscribed);
  EXPECT_EQ(traversability.classify(INSCRIBED_INFLATED_OBSTACLE), Traversability::Inscribed);
  EXPECT_EQ(
    traversability.classify(INSCRIBED_INFLATED_OBSTACLE - 1), Traversability::Circumscribed);
  EXPECT_EQ(traversability.classify(threshold), Traversability::Circumscribed);
  EXPECT_EQ(
    traversability.classify(static_cast<std::uint8_t>(threshold - 1)), Traversability::Free);
  EXPECT_EQ(traversability.classify(FREE_SPACE), Traversability::Free);
}

TEST(CostModel, UnknownHandlingIsConfigurable)
{
  const std::uint8_t threshold = model().circumscribed_cost();
  const CostTraversabilityModel blocking(threshold, false);
  const CostTraversabilityModel permissive(threshold, true);
  EXPECT_EQ(blocking.classify(NO_INFORMATION), Traversability::Inscribed);
  EXPECT_EQ(permissive.classify(NO_INFORMATION), Traversability::Free);
  EXPECT_EQ(permissive.classify(LETHAL_OBSTACLE), Traversability::Inscribed);
}

TEST(CostModel, IsObstacleOnlySelectsLethalCells)
{
  const CostTraversabilityModel traversability(model().circumscribed_cost());

  EXPECT_TRUE(traversability.is_obstacle(LETHAL_OBSTACLE));
  EXPECT_FALSE(traversability.is_obstacle(INSCRIBED_INFLATED_OBSTACLE));
  EXPECT_FALSE(traversability.is_obstacle(MAX_NON_OBSTACLE));
  EXPECT_FALSE(traversability.is_obstacle(FREE_SPACE));
}

TEST(CostModel, IsObstacleFollowsTheUnknownPolicy)
{
  const std::uint8_t threshold = model().circumscribed_cost();
  EXPECT_TRUE(CostTraversabilityModel(threshold, false).is_obstacle(NO_INFORMATION));
  EXPECT_FALSE(CostTraversabilityModel(threshold, true).is_obstacle(NO_INFORMATION));
}

TEST(CostModel, InflatedCellsAreInscribedButNotObstacles)
{
  const CostTraversabilityModel traversability(model().circumscribed_cost());
  // The inflation layer writes at most INSCRIBED_INFLATED_OBSTACLE onto non-obstacle cells.
  EXPECT_EQ(traversability.classify(INSCRIBED_INFLATED_OBSTACLE), Traversability::Inscribed);
  EXPECT_FALSE(traversability.is_obstacle(INSCRIBED_INFLATED_OBSTACLE));
}
