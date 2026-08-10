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
#include <stdexcept>
#include <vector>

namespace
{

using eltanin::Polygon2D;
using eltanin::Pose2D;
using eltanin::Twist2D;
using eltanin::collision::VelocityLimiter;
using eltanin::collision::VelocityLimiterParams;
using eltanin::collision::detail::limit_command;
using eltanin::collision::detail::proximity_scale;
using eltanin_test::default_footprint;
using eltanin_test::distance_scenario;
using eltanin_test::DistanceScenario;
using eltanin_test::narrow_footprint;
using eltanin_test::raw_wall_map;
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

/// The narrow footprint keeps the braking law out of the way so the ramp can be observed alone.
VelocityLimiter narrow_limiter()
{
  VelocityLimiterParams params;
  params.footprint = narrow_footprint();
  return make_limiter(params);
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

  VelocityLimiterParams negative_reaction;
  negative_reaction.reaction_time = -1.0;
  EXPECT_FALSE(VelocityLimiter::create(negative_reaction).has_value());

  VelocityLimiterParams infinite_reaction;
  infinite_reaction.reaction_time = INFINITE_DISTANCE;
  EXPECT_FALSE(VelocityLimiter::create(infinite_reaction).has_value());

  VelocityLimiterParams negative_margin;
  negative_margin.collision_margin = -0.1;
  EXPECT_FALSE(VelocityLimiter::create(negative_margin).has_value());

  VelocityLimiterParams zero_deceleration;
  zero_deceleration.max_deceleration = 0.0;
  EXPECT_FALSE(VelocityLimiter::create(zero_deceleration).has_value());
}

TEST(VelocityLimiterCreate, RejectsAnUnusableProximityRamp)
{
  VelocityLimiterParams negative_stop;
  negative_stop.stop_clearance = -0.1;
  EXPECT_FALSE(VelocityLimiter::create(negative_stop).has_value());

  VelocityLimiterParams inverted;
  inverted.slow_down_clearance = inverted.stop_clearance;
  EXPECT_FALSE(VelocityLimiter::create(inverted).has_value());

  // A floor of zero would let the ramp stop the robot, which only the longitudinal laws may do.
  VelocityLimiterParams zero_floor;
  zero_floor.min_proximity_scale = 0.0;
  EXPECT_FALSE(VelocityLimiter::create(zero_floor).has_value());

  VelocityLimiterParams above_one;
  above_one.min_proximity_scale = 1.5;
  EXPECT_FALSE(VelocityLimiter::create(above_one).has_value());
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

TEST(VelocityLimiterCreate, PredictionStepIsTheDerivedHorizonOverTheStepCount)
{
  const VelocityLimiter limiter = make_limiter(VelocityLimiterParams{});
  const VelocityLimiterParams & params = limiter.params();

  EXPECT_DOUBLE_EQ(limiter.horizon(twist(0.0, 0.9)), params.reaction_time);
  EXPECT_DOUBLE_EQ(limiter.horizon(twist(0.5, 0.0)), 1.3);
  EXPECT_DOUBLE_EQ(limiter.horizon(twist(-0.5, 0.0)), limiter.horizon(twist(0.5, 0.0)));
  EXPECT_LT(limiter.horizon(twist(0.1, 0.0)), limiter.horizon(twist(0.5, 0.0)));
  EXPECT_DOUBLE_EQ(
    limiter.prediction_dt(twist(0.5, 0.0)),
    limiter.horizon(twist(0.5, 0.0)) / static_cast<double>(params.prediction_steps));
}

TEST(VelocityLimiterLimit, ReportsTheHorizonItRolledOut)
{
  const CollisionScenario scenario = free_scenario();
  const VelocityLimiter limiter = make_limiter(VelocityLimiterParams{});

  const VelocityLimiter::Result slow =
    limiter.limit(scenario.map, scenario.model, pose_at_cell(11, 11, 0.0), twist(0.1, 0.0));
  const VelocityLimiter::Result fast =
    limiter.limit(scenario.map, scenario.model, pose_at_cell(11, 11, 0.0), twist(0.3, 0.0));

  EXPECT_DOUBLE_EQ(slow.horizon, limiter.horizon(twist(0.1, 0.0)));
  EXPECT_DOUBLE_EQ(fast.horizon, limiter.horizon(twist(0.3, 0.0)));
  EXPECT_LT(slow.horizon, fast.horizon);
}

TEST(ProximityScale, RampsBetweenTheTwoClearancesAndNeverReachesZero)
{
  const VelocityLimiterParams params;

  EXPECT_DOUBLE_EQ(proximity_scale(params, INFINITE_DISTANCE), 1.0);
  EXPECT_DOUBLE_EQ(proximity_scale(params, params.slow_down_clearance), 1.0);
  EXPECT_DOUBLE_EQ(proximity_scale(params, 1.0), 1.0);
  EXPECT_DOUBLE_EQ(proximity_scale(params, params.stop_clearance), params.min_proximity_scale);
  EXPECT_DOUBLE_EQ(proximity_scale(params, 0.0), params.min_proximity_scale);
  EXPECT_DOUBLE_EQ(proximity_scale(params, -1.0), params.min_proximity_scale);

  const double middle =
    0.5 * (params.stop_clearance + params.slow_down_clearance);
  EXPECT_DOUBLE_EQ(
    proximity_scale(params, middle), 0.5 * (params.min_proximity_scale + 1.0));

  double previous = params.min_proximity_scale;
  for (int step = 0; step <= 20; ++step) {
    const double clearance = 0.05 * static_cast<double>(step);
    const double scale = proximity_scale(params, clearance);
    EXPECT_GE(scale, params.min_proximity_scale);
    EXPECT_LE(scale, 1.0);
    EXPECT_GE(scale, previous);
    previous = scale;
  }
}

TEST(VelocityLimiterLimit, ADistanceMapCarriesAClearanceAndAnOpenMapDoesNotScale)
{
  const DistanceScenario scenario = distance_scenario(raw_wall_map(false), narrow_footprint());
  const VelocityLimiter limiter = narrow_limiter();
  const Twist2D cmd_in = twist(0.3, 0.0);

  const VelocityLimiter::Result result =
    limiter.limit(scenario.map, scenario.model, pose_at_cell(20, 11, 0.0), cmd_in);

  ASSERT_TRUE(result.clearance.has_value());
  EXPECT_NEAR(
    *result.clearance, eltanin_test::CLEARANCE_MAX_DISTANCE - limiter.circumscribed_radius(), 1e-6);
  EXPECT_DOUBLE_EQ(result.proximity_scale, 1.0);
  EXPECT_DOUBLE_EQ(result.command.linear.x(), cmd_in.linear.x());
}

TEST(VelocityLimiterLimit, TheProximityRampSlowsDownWithoutStopping)
{
  const DistanceScenario scenario = distance_scenario(raw_wall_map(false), narrow_footprint());
  const VelocityLimiter limiter = narrow_limiter();
  const Twist2D cmd_in = twist(-0.3, 0.0);

  const VelocityLimiter::Result result =
    limiter.limit(scenario.map, scenario.model, pose_at_cell(9, 11, 0.0), cmd_in);

  ASSERT_FALSE(result.has_collision);
  ASSERT_TRUE(result.clearance.has_value());
  // The sweep runs one braking distance towards the wall, so it is closer than the robot itself.
  EXPECT_LT(*result.clearance, 0.45 - limiter.circumscribed_radius());
  EXPECT_LT(result.proximity_scale, 1.0);
  EXPECT_GT(result.proximity_scale, limiter.params().min_proximity_scale);
  EXPECT_DOUBLE_EQ(result.command.linear.x(), cmd_in.linear.x() * result.proximity_scale);
  EXPECT_LT(result.command.linear.x(), 0.0);
}

TEST(VelocityLimiterLimit, ACostMapCarriesNoClearanceAndKeepsTheLawItHad)
{
  const CollisionScenario scenario = wall_scenario(false);
  const VelocityLimiter limiter = make_limiter(VelocityLimiterParams{});

  const VelocityLimiter::Result result =
    limiter.limit(scenario.map, scenario.model, pose_at_cell(9, 11, 0.0), twist(-0.3, 0.0));

  EXPECT_FALSE(result.clearance.has_value());
  EXPECT_DOUBLE_EQ(result.proximity_scale, 1.0);
}

TEST(VelocityLimiterLimit, TheProximityRampKeepsTheSignAndTheCurvature)
{
  const DistanceScenario scenario = distance_scenario(raw_wall_map(false), narrow_footprint());
  const VelocityLimiter limiter = narrow_limiter();
  const Twist2D cmd_in = twist(-0.3, 0.4);

  const VelocityLimiter::Result result =
    limiter.limit(scenario.map, scenario.model, pose_at_cell(9, 11, 0.0), cmd_in);

  ASSERT_TRUE(result.clearance.has_value());
  ASSERT_LT(result.proximity_scale, 1.0);
  EXPECT_LT(result.command.linear.x(), 0.0);
  EXPECT_GT(result.command.angular, 0.0);
  EXPECT_DOUBLE_EQ(
    result.command.angular / result.command.linear.x(), cmd_in.angular / cmd_in.linear.x());
}

TEST(VelocityLimiterLimit, TheProximityRampSlowsInPlaceRotationWithoutBlockingIt)
{
  const DistanceScenario scenario = distance_scenario(raw_wall_map(false), narrow_footprint());
  const VelocityLimiter limiter = narrow_limiter();

  const VelocityLimiter::Result result =
    limiter.limit(scenario.map, scenario.model, pose_at_cell(9, 11, 0.0), twist(0.0, 0.5));

  ASSERT_FALSE(result.has_collision);
  ASSERT_LT(result.proximity_scale, 1.0);
  EXPECT_DOUBLE_EQ(result.command.angular, 0.5 * result.proximity_scale);
  EXPECT_GT(result.command.angular, 0.0);
}

TEST(VelocityLimiter, RejectsAnEmptyMap)
{
  const VelocityLimiter limiter = make_limiter(VelocityLimiterParams{});
  const CollisionScenario scenario = free_scenario();
  EXPECT_THROW(
    limiter.limit(eltanin::map::Costmap{}, scenario.model, Pose2D{}, twist(0.5, 0.0)),
    std::invalid_argument);
}

TEST(LimitCommand, LimitsAReverseCommandByItsMagnitude)
{
  const VelocityLimiterParams params;
  const double v_in = -0.5;
  const Twist2D limited = limit_command(params, twist(v_in, 0.0), true, 0.4, 1.0);

  const double v_max = std::sqrt(2.0 * params.max_deceleration * (0.4 - params.collision_margin));
  EXPECT_DOUBLE_EQ(limited.linear.x(), -v_max);
  EXPECT_DOUBLE_EQ(limited.linear.x(), -std::sqrt(0.2));
  EXPECT_LT(limited.linear.x(), 0.0);
  EXPECT_LT(std::abs(limited.linear.x()), std::abs(v_in));
  // navyu computed std::min(v_max, v_in), which for a reverse command returns v_in unchanged.
  EXPECT_DOUBLE_EQ(std::min(v_max, v_in), v_in);
}

TEST(LimitCommand, LimitsForwardAndReverseSymmetrically)
{
  const VelocityLimiterParams params;
  const Twist2D forward = limit_command(params, twist(0.5, 0.0), true, 0.4, 1.0);
  const Twist2D reverse = limit_command(params, twist(-0.5, 0.0), true, 0.4, 1.0);

  EXPECT_DOUBLE_EQ(forward.linear.x(), std::sqrt(0.2));
  EXPECT_DOUBLE_EQ(reverse.linear.x(), -forward.linear.x());
}

TEST(LimitCommand, KeepsTheCurvatureAndTheAngularSign)
{
  const VelocityLimiterParams params;
  for (const double w_in : {0.4, -0.4}) {
    const Twist2D cmd_in = twist(-0.5, w_in);
    const Twist2D limited = limit_command(params, cmd_in, true, 0.4, 1.0);

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
    const Twist2D limited = limit_command(params, twist(v_in, 0.3), true, 0.1, 1.0);
    EXPECT_DOUBLE_EQ(std::abs(limited.linear.x()), 0.0);
    EXPECT_DOUBLE_EQ(limited.angular, 0.0);
  }
}

TEST(LimitCommand, TheReactionTimeBindsInsideTheCrossover)
{
  const VelocityLimiterParams params;
  const double crossover =
    2.0 * params.max_deceleration * params.reaction_time * params.reaction_time;
  EXPECT_DOUBLE_EQ(crossover, 0.09);

  const Twist2D near = limit_command(params, twist(0.5, 0.0), true, params.collision_margin + 0.04, 1.0);
  EXPECT_DOUBLE_EQ(near.linear.x(), 0.04 / params.reaction_time);
  EXPECT_LT(near.linear.x(), std::sqrt(2.0 * params.max_deceleration * 0.04));

  const Twist2D at_crossover =
    limit_command(params, twist(0.5, 0.0), true, params.collision_margin + crossover, 1.0);
  EXPECT_DOUBLE_EQ(at_crossover.linear.x(), crossover / params.reaction_time);
  EXPECT_DOUBLE_EQ(
    at_crossover.linear.x(), std::sqrt(2.0 * params.max_deceleration * crossover));

  const Twist2D far = limit_command(params, twist(0.5, 0.0), true, params.collision_margin + 0.25, 1.0);
  EXPECT_DOUBLE_EQ(far.linear.x(), std::sqrt(2.0 * params.max_deceleration * 0.25));
  EXPECT_LT(far.linear.x(), 0.25 / params.reaction_time);
}

TEST(LimitCommand, TheReactionTimeCapKeepsTheSignAndTheCurvature)
{
  const VelocityLimiterParams params;
  const Twist2D cmd_in = twist(-0.5, 0.4);
  const Twist2D limited = limit_command(params, cmd_in, true, params.collision_margin + 0.04, 1.0);

  EXPECT_DOUBLE_EQ(limited.linear.x(), -0.04 / params.reaction_time);
  EXPECT_DOUBLE_EQ(limited.angular / limited.linear.x(), cmd_in.angular / cmd_in.linear.x());
}

TEST(LimitCommand, PassesTheCommandThroughWhenThereIsNoCollision)
{
  const VelocityLimiterParams params;
  const Twist2D cmd_in = twist(0.05, 0.3);
  const Twist2D limited = limit_command(params, cmd_in, false, INFINITE_DISTANCE, 1.0);

  EXPECT_DOUBLE_EQ(limited.linear.x(), cmd_in.linear.x());
  EXPECT_DOUBLE_EQ(limited.angular, cmd_in.angular);
}

TEST(LimitCommand, PassesInPlaceRotationThroughWhenThereIsNoCollision)
{
  const Twist2D limited =
    limit_command(VelocityLimiterParams{}, twist(0.0, 0.5), false, INFINITE_DISTANCE, 1.0);

  EXPECT_DOUBLE_EQ(limited.linear.x(), 0.0);
  EXPECT_DOUBLE_EQ(limited.angular, 0.5);
}

TEST(LimitCommand, ZeroesInPlaceRotationOnACollision)
{
  const Twist2D limited = limit_command(VelocityLimiterParams{}, twist(0.0, 0.5), true, 0.0, 1.0);

  EXPECT_DOUBLE_EQ(limited.linear.x(), 0.0);
  EXPECT_DOUBLE_EQ(limited.angular, 0.0);
}

TEST(LimitCommand, IgnoresTheLateralInputAndNeverEmitsIt)
{
  const Twist2D omni{Vector2d{-0.5, 0.7}, 0.4};
  const Twist2D limited = limit_command(VelocityLimiterParams{}, omni, true, 0.4, 1.0);

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
  EXPECT_DOUBLE_EQ(result.collision_distance, 0.446875);
  EXPECT_DOUBLE_EQ(result.command.linear.x(), -std::sqrt(0.246875));
  ASSERT_EQ(result.predicted_poses.size(), 8u);
  EXPECT_NEAR(result.predicted_poses.back().position.x(), 0.32, 1e-12);
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
  EXPECT_DOUBLE_EQ(result.collision_distance, 0.446875);
  EXPECT_DOUBLE_EQ(result.command.linear.x(), std::sqrt(0.246875));
  ASSERT_EQ(result.predicted_poses.size(), 8u);
}

TEST(VelocityLimiterLimit, RefinesTheCollisionDistanceInsideTheCollidingStep)
{
  const CollisionScenario scenario = wall_scenario(false);
  const VelocityLimiter limiter = make_limiter(VelocityLimiterParams{});
  const Twist2D cmd_in = twist(-0.5, 0.0);

  const VelocityLimiter::Result result =
    limiter.limit(scenario.map, scenario.model, pose_at_cell(15, 11, 0.0), cmd_in);

  ASSERT_TRUE(result.has_collision);
  const double step_arc = std::abs(cmd_in.linear.x()) * limiter.prediction_dt(cmd_in);
  const double steps = result.collision_distance / step_arc;
  EXPECT_NE(steps, std::floor(steps));
  // The wall cell centre enters the footprint exactly 0.45 m into the rollout.
  EXPECT_LE(result.collision_distance, 0.45);
  EXPECT_GT(result.collision_distance, 0.45 - step_arc / 16.0);
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

TEST(VelocityLimiterLimit, ReportsTheTimeToCollision)
{
  const CollisionScenario wall = wall_scenario(false);
  const VelocityLimiter limiter = make_limiter(VelocityLimiterParams{});
  const Twist2D cmd_in = twist(-0.5, 0.0);

  const VelocityLimiter::Result blocked =
    limiter.limit(wall.map, wall.model, pose_at_cell(15, 11, 0.0), cmd_in);
  EXPECT_DOUBLE_EQ(
    blocked.time_to_collision,
    (blocked.collision_distance - limiter.params().collision_margin) /
      std::abs(cmd_in.linear.x()));

  const CollisionScenario open = free_scenario();
  const VelocityLimiter::Result clear =
    limiter.limit(open.map, open.model, pose_at_cell(11, 11, 0.0), twist(0.05, 0.0));
  EXPECT_EQ(clear.time_to_collision, INFINITE_DISTANCE);

  const CollisionScenario obstacle = single_obstacle_scenario(5, 0);
  const VelocityLimiter::Result at_rest =
    limiter.limit(obstacle.map, obstacle.model, pose_at_cell(11, 11, 0.0), twist(0.0, 0.5));
  EXPECT_TRUE(at_rest.has_collision);
  EXPECT_EQ(at_rest.time_to_collision, INFINITE_DISTANCE);
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
  const CollisionScenario scenario = uninflated_scenario(5, 5);
  const Pose2D robot = pose_at_cell(7, 11, 0.0);
  const Twist2D cmd_in = twist(0.5, 0.0);

  VelocityLimiterParams two_stage;
  two_stage.exact_footprint_check = false;

  const VelocityLimiter::Result exact =
    make_limiter(VelocityLimiterParams{}).limit(scenario.map, scenario.model, robot, cmd_in);
  const VelocityLimiter::Result lenient =
    make_limiter(two_stage).limit(scenario.map, scenario.model, robot, cmd_in);

  EXPECT_TRUE(exact.has_collision);
  EXPECT_DOUBLE_EQ(exact.collision_distance, 0.14625);
  EXPECT_DOUBLE_EQ(exact.command.linear.x(), 0.0);
  EXPECT_EQ(exact.predicted_poses.size(), 4u);

  // No predicted pose lands on the lethal cell (16, 16), so the Free gate hides it every step.
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
