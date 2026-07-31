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

#include <eltanin/collision/velocity_limiter.hpp>

#include <collision/collision_fixture.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{

using eltanin::Polygon2D;
using eltanin::Pose2D;
using eltanin::Twist2D;
using eltanin::collision::VelocityLimiter;
using eltanin::collision::VelocityLimiterParams;
using eltanin::collision::detail::limit_command;
using eltanin_test::default_footprint;
using eltanin_test::free_scenario;
using eltanin_test::make_limiter;
using eltanin_test::reversed_footprint;
using eltanin_test::CollisionScenario;
using eltanin_test::single_obstacle_scenario;
using eltanin_test::uninflated_scenario;
using eltanin_test::wall_scenario;
using Eigen::Vector2d;

constexpr double INFINITE_DISTANCE = std::numeric_limits<double>::infinity();

Twist2D twist(double v, double w) { return Twist2D{Vector2d{v, 0.0}, w}; }

/// Centre of cell (mx, my) of the 24 x 24 map at 0.05 m.
Pose2D pose_at_cell(int mx, int my, double yaw)
{
  return Pose2D{
    Vector2d{0.05 * static_cast<double>(mx) + 0.025, 0.05 * static_cast<double>(my) + 0.025}, yaw};
}

void expect_same_result(const VelocityLimiter::Result & a, const VelocityLimiter::Result & b)
{
  EXPECT_EQ(a.has_collision, b.has_collision);
  EXPECT_DOUBLE_EQ(a.collision_distance, b.collision_distance);
  EXPECT_DOUBLE_EQ(a.command.linear.x(), b.command.linear.x());
  EXPECT_DOUBLE_EQ(a.command.linear.y(), b.command.linear.y());
  EXPECT_DOUBLE_EQ(a.command.angular, b.command.angular);
  ASSERT_EQ(a.predicted_poses.size(), b.predicted_poses.size());
  for (std::size_t i = 0; i < a.predicted_poses.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.predicted_poses[i].position.x(), b.predicted_poses[i].position.x());
    EXPECT_DOUBLE_EQ(a.predicted_poses[i].position.y(), b.predicted_poses[i].position.y());
    EXPECT_DOUBLE_EQ(a.predicted_poses[i].yaw, b.predicted_poses[i].yaw);
  }
}

}  // namespace

TEST(VelocityLimiterCreate, AcceptsTheDefaultParameters)
{
  EXPECT_TRUE(VelocityLimiter::create(VelocityLimiterParams{}).has_value());
}

TEST(VelocityLimiterCreate, ChecksTheFootprintExactlyByDefault)
{
  EXPECT_TRUE(VelocityLimiterParams{}.exact_footprint_check);
  EXPECT_TRUE(make_limiter(VelocityLimiterParams{}).params().exact_footprint_check);
}

TEST(VelocityLimiterCreate, RejectsADegenerateFootprint)
{
  VelocityLimiterParams two_vertices;
  two_vertices.footprint = Polygon2D{Vector2d{-0.3, 0.0}, Vector2d{0.3, 0.0}};
  EXPECT_FALSE(VelocityLimiter::create(two_vertices).has_value());

  VelocityLimiterParams collinear;
  collinear.footprint =
    Polygon2D{Vector2d{-0.3, 0.0}, Vector2d{0.0, 0.0}, Vector2d{0.3, 0.0}};
  EXPECT_FALSE(VelocityLimiter::create(collinear).has_value());
}

TEST(VelocityLimiterCreate, RejectsAFootprintThatExcludesTheOrigin)
{
  VelocityLimiterParams params;
  params.footprint =
    Polygon2D{Vector2d{0.1, 0.1}, Vector2d{0.4, 0.1}, Vector2d{0.4, 0.4}, Vector2d{0.1, 0.4}};
  EXPECT_FALSE(VelocityLimiter::create(params).has_value());
}

TEST(VelocityLimiterCreate, RejectsANonConvexFootprint)
{
  VelocityLimiterParams params;
  params.footprint = Polygon2D{
    Vector2d{-0.3, -0.3}, Vector2d{0.3, -0.3}, Vector2d{0.3, 0.3}, Vector2d{0.05, 0.05},
    Vector2d{-0.3, 0.3}};
  EXPECT_FALSE(VelocityLimiter::create(params).has_value());
}

TEST(VelocityLimiterCreate, RejectsNonFiniteVertices)
{
  VelocityLimiterParams params;
  params.footprint = Polygon2D{
    Vector2d{-0.3, -0.3}, Vector2d{std::numeric_limits<double>::quiet_NaN(), -0.3},
    Vector2d{0.3, 0.3}, Vector2d{-0.3, 0.3}};
  EXPECT_FALSE(VelocityLimiter::create(params).has_value());
}

TEST(VelocityLimiterCreate, RejectsInvalidScalars)
{
  VelocityLimiterParams zero_steps;
  zero_steps.prediction_steps = 0;
  EXPECT_FALSE(VelocityLimiter::create(zero_steps).has_value());

  VelocityLimiterParams negative_time;
  negative_time.prediction_time = -1.0;
  EXPECT_FALSE(VelocityLimiter::create(negative_time).has_value());

  VelocityLimiterParams infinite_time;
  infinite_time.prediction_time = INFINITE_DISTANCE;
  EXPECT_FALSE(VelocityLimiter::create(infinite_time).has_value());

  VelocityLimiterParams negative_margin;
  negative_margin.collision_margin = -0.1;
  EXPECT_FALSE(VelocityLimiter::create(negative_margin).has_value());

  VelocityLimiterParams zero_deceleration;
  zero_deceleration.max_deceleration = 0.0;
  EXPECT_FALSE(VelocityLimiter::create(zero_deceleration).has_value());
}

TEST(VelocityLimiterCreate, NormalizesTheFootprintToCounterClockwise)
{
  VelocityLimiterParams clockwise;
  clockwise.footprint = reversed_footprint(default_footprint());
  ASSERT_LT(eltanin::signed_area(clockwise.footprint), 0.0);

  const VelocityLimiter limiter = make_limiter(clockwise);
  EXPECT_GT(eltanin::signed_area(limiter.footprint()), 0.0);

  const VelocityLimiter unchanged = make_limiter(VelocityLimiterParams{});
  EXPECT_GT(eltanin::signed_area(unchanged.footprint()), 0.0);
  EXPECT_DOUBLE_EQ(eltanin::signed_area(unchanged.footprint()), 0.36);
}

TEST(VelocityLimiterCreate, PredictionStepIsTheHorizonOverTheStepCount)
{
  EXPECT_DOUBLE_EQ(make_limiter(VelocityLimiterParams{}).prediction_dt(), 0.2);
}

TEST(LimitCommand, LimitsAReverseCommandByItsMagnitude)
{
  const VelocityLimiterParams params;
  const double v_in = -0.5;
  const Twist2D limited = limit_command(params, twist(v_in, 0.0), true, 0.25);

  const double v_max = std::sqrt(2.0 * params.max_deceleration * (0.25 - params.collision_margin));
  EXPECT_DOUBLE_EQ(limited.linear.x(), -v_max);
  EXPECT_DOUBLE_EQ(limited.linear.x(), -std::sqrt(0.05));
  EXPECT_LT(limited.linear.x(), 0.0);
  EXPECT_LT(std::abs(limited.linear.x()), std::abs(v_in));
  // navyu computed std::min(v_max, v_in), which for a reverse command returns v_in unchanged.
  EXPECT_DOUBLE_EQ(std::min(v_max, v_in), v_in);
}

TEST(LimitCommand, LimitsForwardAndReverseSymmetrically)
{
  const VelocityLimiterParams params;
  const Twist2D forward = limit_command(params, twist(0.5, 0.0), true, 0.4);
  const Twist2D reverse = limit_command(params, twist(-0.5, 0.0), true, 0.4);

  EXPECT_DOUBLE_EQ(forward.linear.x(), std::sqrt(0.2));
  EXPECT_DOUBLE_EQ(reverse.linear.x(), -forward.linear.x());
}

TEST(LimitCommand, KeepsTheCurvatureAndTheAngularSign)
{
  const VelocityLimiterParams params;
  for (const double w_in : {0.4, -0.4}) {
    const Twist2D cmd_in = twist(-0.5, w_in);
    const Twist2D limited = limit_command(params, cmd_in, true, 0.4);

    EXPECT_DOUBLE_EQ(
      limited.angular, w_in * (limited.linear.x() / cmd_in.linear.x()));
    EXPECT_DOUBLE_EQ(
      limited.angular / limited.linear.x(), cmd_in.angular / cmd_in.linear.x());
    EXPECT_GT(limited.angular * w_in, 0.0);
    EXPECT_LT(std::abs(limited.angular), std::abs(w_in));
  }
}

TEST(LimitCommand, StopsWhenTheCollisionIsInsideTheMargin)
{
  const VelocityLimiterParams params;
  for (const double v_in : {0.5, -0.5}) {
    const Twist2D limited = limit_command(params, twist(v_in, 0.3), true, 0.1);
    EXPECT_DOUBLE_EQ(std::abs(limited.linear.x()), 0.0);
    EXPECT_DOUBLE_EQ(limited.angular, 0.0);
  }
}

TEST(LimitCommand, PassesTheCommandThroughWhenThereIsNoCollision)
{
  const VelocityLimiterParams params;
  const Twist2D cmd_in = twist(0.05, 0.3);
  const Twist2D limited = limit_command(params, cmd_in, false, INFINITE_DISTANCE);

  EXPECT_DOUBLE_EQ(limited.linear.x(), cmd_in.linear.x());
  EXPECT_DOUBLE_EQ(limited.angular, cmd_in.angular);
}

TEST(LimitCommand, PassesInPlaceRotationThroughWhenThereIsNoCollision)
{
  const Twist2D limited =
    limit_command(VelocityLimiterParams{}, twist(0.0, 0.5), false, INFINITE_DISTANCE);

  EXPECT_DOUBLE_EQ(limited.linear.x(), 0.0);
  EXPECT_DOUBLE_EQ(limited.angular, 0.5);
}

TEST(LimitCommand, ZeroesInPlaceRotationOnACollision)
{
  const Twist2D limited = limit_command(VelocityLimiterParams{}, twist(0.0, 0.5), true, 0.0);

  EXPECT_DOUBLE_EQ(limited.linear.x(), 0.0);
  EXPECT_DOUBLE_EQ(limited.angular, 0.0);
}

TEST(LimitCommand, IgnoresTheLateralInputAndNeverEmitsIt)
{
  const Twist2D omni{Vector2d{-0.5, 0.7}, 0.4};
  const Twist2D limited = limit_command(VelocityLimiterParams{}, omni, true, 0.4);

  EXPECT_DOUBLE_EQ(limited.linear.y(), 0.0);
  EXPECT_DOUBLE_EQ(limited.linear.x(), -std::sqrt(0.2));
}

TEST(VelocityLimiterLimit, LimitsAReverseCommandInFrontOfAWall)
{
  const CollisionScenario scenario = wall_scenario(false);
  const VelocityLimiter limiter = make_limiter(VelocityLimiterParams{});
  const Twist2D cmd_in = twist(-0.5, 0.0);

  const VelocityLimiter::Result result =
    limiter.limit(scenario.map, scenario.model, pose_at_cell(15, 11, 0.0), cmd_in);

  EXPECT_TRUE(result.has_collision);
  EXPECT_DOUBLE_EQ(result.collision_distance, 0.4);
  EXPECT_DOUBLE_EQ(result.command.linear.x(), -std::sqrt(0.2));
  ASSERT_EQ(result.predicted_poses.size(), 6u);
  EXPECT_NEAR(result.predicted_poses.back().position.x(), 0.275, 1e-12);
  // navyu's std::min() would have passed the requested -0.5 straight through.
  EXPECT_LT(std::abs(result.command.linear.x()), std::abs(cmd_in.linear.x()));
}

TEST(VelocityLimiterLimit, LimitsAForwardCommandSymmetrically)
{
  const CollisionScenario scenario = wall_scenario(true);
  const VelocityLimiter limiter = make_limiter(VelocityLimiterParams{});

  const VelocityLimiter::Result result =
    limiter.limit(scenario.map, scenario.model, pose_at_cell(8, 11, 0.0), twist(0.5, 0.0));

  EXPECT_TRUE(result.has_collision);
  EXPECT_DOUBLE_EQ(result.collision_distance, 0.4);
  EXPECT_DOUBLE_EQ(result.command.linear.x(), std::sqrt(0.2));
  ASSERT_EQ(result.predicted_poses.size(), 6u);
}

TEST(VelocityLimiterLimit, PassesTheCommandThroughOnAFreeMap)
{
  const CollisionScenario scenario = free_scenario();
  const VelocityLimiter limiter = make_limiter(VelocityLimiterParams{});
  const Twist2D cmd_in = twist(0.05, 0.3);

  const VelocityLimiter::Result result =
    limiter.limit(scenario.map, scenario.model, pose_at_cell(11, 11, 0.0), cmd_in);

  EXPECT_FALSE(result.has_collision);
  EXPECT_EQ(result.collision_distance, INFINITE_DISTANCE);
  EXPECT_DOUBLE_EQ(result.command.linear.x(), cmd_in.linear.x());
  EXPECT_DOUBLE_EQ(result.command.angular, cmd_in.angular);
  EXPECT_EQ(result.predicted_poses.size(), 11u);
}

TEST(VelocityLimiterLimit, IsDeterministicAcrossCalls)
{
  const CollisionScenario scenario = wall_scenario(false);
  const VelocityLimiter limiter = make_limiter(VelocityLimiterParams{});
  const Pose2D robot = pose_at_cell(15, 11, 0.0);

  const VelocityLimiter::Result first =
    limiter.limit(scenario.map, scenario.model, robot, twist(-0.5, 0.2));
  const VelocityLimiter::Result second =
    limiter.limit(scenario.map, scenario.model, robot, twist(-0.5, 0.2));

  expect_same_result(first, second);
}

TEST(VelocityLimiterLimit, TruncatesThePredictionThatLeavesTheMap)
{
  const CollisionScenario scenario = free_scenario();
  const VelocityLimiter limiter = make_limiter(VelocityLimiterParams{});
  const Twist2D cmd_in = twist(0.5, 0.0);

  const VelocityLimiter::Result result =
    limiter.limit(scenario.map, scenario.model, pose_at_cell(23, 11, 0.0), cmd_in);

  EXPECT_FALSE(result.has_collision);
  EXPECT_LT(result.predicted_poses.size(), 11u);
  EXPECT_EQ(result.predicted_poses.size(), 1u);
  EXPECT_DOUBLE_EQ(result.command.linear.x(), cmd_in.linear.x());
}

TEST(VelocityLimiterLimit, ReportsACollisionForAZeroCommandOnAnOccupiedPose)
{
  const CollisionScenario scenario = single_obstacle_scenario(5, 0);
  const VelocityLimiter limiter = make_limiter(VelocityLimiterParams{});

  const VelocityLimiter::Result result =
    limiter.limit(scenario.map, scenario.model, pose_at_cell(11, 11, 0.0), Twist2D{});

  EXPECT_TRUE(result.has_collision);
  EXPECT_DOUBLE_EQ(result.collision_distance, 0.0);
  EXPECT_DOUBLE_EQ(result.command.linear.x(), 0.0);
  EXPECT_DOUBLE_EQ(result.command.angular, 0.0);
  EXPECT_EQ(result.predicted_poses.size(), 2u);
}

TEST(VelocityLimiterLimit, ZeroesInPlaceRotationTowardsAnObstacle)
{
  const CollisionScenario scenario = single_obstacle_scenario(5, 0);
  const VelocityLimiter limiter = make_limiter(VelocityLimiterParams{});

  const VelocityLimiter::Result result =
    limiter.limit(scenario.map, scenario.model, pose_at_cell(11, 11, 0.0), twist(0.0, 0.5));

  EXPECT_TRUE(result.has_collision);
  EXPECT_DOUBLE_EQ(result.command.linear.x(), 0.0);
  EXPECT_DOUBLE_EQ(result.command.angular, 0.0);
}

TEST(VelocityLimiterLimit, OnlyTheExactCheckStopsForARawObstacleAhead)
{
  const CollisionScenario scenario = uninflated_scenario(5, 0);
  const Pose2D robot = pose_at_cell(7, 11, 0.0);
  const Twist2D cmd_in = twist(0.5, 0.0);

  VelocityLimiterParams two_stage;
  two_stage.exact_footprint_check = false;

  const VelocityLimiter::Result exact =
    make_limiter(VelocityLimiterParams{}).limit(scenario.map, scenario.model, robot, cmd_in);
  const VelocityLimiter::Result lenient =
    make_limiter(two_stage).limit(scenario.map, scenario.model, robot, cmd_in);

  EXPECT_TRUE(exact.has_collision);
  EXPECT_DOUBLE_EQ(exact.collision_distance, 0.1);
  EXPECT_DOUBLE_EQ(exact.command.linear.x(), 0.0);
  EXPECT_EQ(exact.predicted_poses.size(), 3u);

  // No predicted pose lands on the lethal cell (16, 11), so the Free gate hides it every step.
  EXPECT_FALSE(lenient.has_collision);
  EXPECT_EQ(lenient.collision_distance, INFINITE_DISTANCE);
  EXPECT_DOUBLE_EQ(lenient.command.linear.x(), cmd_in.linear.x());
}

TEST(VelocityLimiterLimit, IsIndependentOfTheFootprintVertexOrder)
{
  const CollisionScenario scenario = wall_scenario(false);
  const Pose2D robot = pose_at_cell(15, 11, 0.0);
  const Twist2D cmd_in = twist(-0.5, 0.2);

  VelocityLimiterParams clockwise;
  clockwise.footprint = reversed_footprint(default_footprint());

  const VelocityLimiter::Result counter_clockwise =
    make_limiter(VelocityLimiterParams{}).limit(scenario.map, scenario.model, robot, cmd_in);
  const VelocityLimiter::Result reversed =
    make_limiter(clockwise).limit(scenario.map, scenario.model, robot, cmd_in);

  expect_same_result(counter_clockwise, reversed);
}
