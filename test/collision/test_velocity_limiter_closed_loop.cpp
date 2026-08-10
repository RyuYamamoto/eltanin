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

#include <eltanin/collision/collision_checker.hpp>
#include <eltanin/sim/simple_simulator.hpp>
#include <collision/collision_fixture.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace
{

using eltanin::Pose2D;
using eltanin::Twist2D;
using eltanin::collision::check_footprint;
using eltanin::collision::CollisionCheck;
using eltanin::collision::VelocityLimiter;
using eltanin::collision::VelocityLimiterParams;
using eltanin::sim::SimpleSimulator;
using eltanin_test::default_footprint;
using eltanin_test::make_limiter;
using eltanin_test::CollisionScenario;
using eltanin_test::wall_scenario;
using Eigen::Vector2d;

constexpr double CONTROL_DT = 0.1;

constexpr int MAX_CYCLES = 40;
constexpr double HALF_FOOTPRINT = 0.3;

struct ClosedLoopTrace
{
  std::vector<double> commanded_velocities;
  Pose2D stop_pose;
  bool stopped{false};
};

/// Runs the limiter against the plant until the limited command reaches exactly zero.
ClosedLoopTrace run_closed_loop(
  const CollisionScenario & scenario, const VelocityLimiter & limiter, const Pose2D & start,
  const Twist2D & cmd_in)
{
  SimpleSimulator plant(start);
  ClosedLoopTrace trace;
  for (int cycle = 0; cycle < MAX_CYCLES; ++cycle) {
    const VelocityLimiter::Result result =
      limiter.limit(scenario.map, scenario.model, plant.pose(), cmd_in);
    trace.commanded_velocities.push_back(result.command.linear.x());
    if (result.command.linear.x() == 0.0) {
      trace.stopped = true;
      break;
    }
    plant.update(result.command, CONTROL_DT);
    // The pose the limited command actually reached must never be in collision.
    EXPECT_EQ(
      check_footprint(scenario.map, scenario.model, limiter.footprint(), plant.pose()),
      CollisionCheck::Free)
      << "cycle " << cycle;
  }
  trace.stop_pose = plant.pose();
  return trace;
}

/// Bisection quantum of the rollout; the stop can undershoot the margin by at most one of them.
double margin_overshoot(const VelocityLimiter & limiter, const Twist2D & cmd_in)
{
  const double step_arc = std::abs(cmd_in.linear.x()) * limiter.prediction_dt(cmd_in);
  return step_arc / 16.0;
}

Pose2D pose_at_cell(int mx, int my, double yaw)
{
  return Pose2D{
    Vector2d{0.05 * static_cast<double>(mx) + 0.025, 0.05 * static_cast<double>(my) + 0.025}, yaw};
}

}  // namespace

TEST(VelocityLimiterClosedLoop, StopsInFrontOfAWallWhenDrivingForward)
{
  const CollisionScenario scenario = wall_scenario(true);
  const VelocityLimiter limiter = make_limiter(VelocityLimiterParams{});
  const Twist2D cmd_in{Vector2d{0.5, 0.0}, 0.0};

  const ClosedLoopTrace trace =
    run_closed_loop(scenario, limiter, pose_at_cell(8, 11, 0.0), cmd_in);

  ASSERT_TRUE(trace.stopped);
  EXPECT_LE(trace.commanded_velocities.size(), static_cast<std::size_t>(MAX_CYCLES));

  // Clearance measured from the leading footprint edge to the centre of the lethal wall cell.
  const double wall_centre_x = 0.05 * 23.0 + 0.025;
  const double clearance = wall_centre_x - (trace.stop_pose.position.x() + HALF_FOOTPRINT);
  const VelocityLimiterParams & params = limiter.params();
  EXPECT_GE(clearance, params.collision_margin - margin_overshoot(limiter, cmd_in));
  EXPECT_LE(clearance, params.collision_margin + 2.0 * std::abs(cmd_in.linear.x()) * CONTROL_DT);
}

TEST(VelocityLimiterClosedLoop, StopsInFrontOfAWallWhenDrivingBackward)
{
  const CollisionScenario scenario = wall_scenario(false);
  const VelocityLimiter limiter = make_limiter(VelocityLimiterParams{});
  const Twist2D cmd_in{Vector2d{-0.5, 0.0}, 0.0};

  const ClosedLoopTrace trace =
    run_closed_loop(scenario, limiter, pose_at_cell(15, 11, 0.0), cmd_in);

  ASSERT_TRUE(trace.stopped);
  for (const double v_out : trace.commanded_velocities) {
    EXPECT_LE(v_out, 0.0);
    EXPECT_GE(v_out, cmd_in.linear.x());
  }

  const double wall_centre_x = 0.025;
  const double clearance = (trace.stop_pose.position.x() - HALF_FOOTPRINT) - wall_centre_x;
  const VelocityLimiterParams & params = limiter.params();
  EXPECT_GE(clearance, params.collision_margin - margin_overshoot(limiter, cmd_in));
  EXPECT_LE(clearance, params.collision_margin + 2.0 * std::abs(cmd_in.linear.x()) * CONTROL_DT);
}

TEST(VelocityLimiterClosedLoop, ForwardAndBackwardStopWithTheSameClearance)
{
  const VelocityLimiter limiter = make_limiter(VelocityLimiterParams{});
  const CollisionScenario forward_scenario = wall_scenario(true);
  const CollisionScenario backward_scenario = wall_scenario(false);

  const ClosedLoopTrace forward = run_closed_loop(
    forward_scenario, limiter, pose_at_cell(8, 11, 0.0), Twist2D{Vector2d{0.5, 0.0}, 0.0});
  const ClosedLoopTrace backward = run_closed_loop(
    backward_scenario, limiter, pose_at_cell(15, 11, 0.0), Twist2D{Vector2d{-0.5, 0.0}, 0.0});

  ASSERT_TRUE(forward.stopped);
  ASSERT_TRUE(backward.stopped);
  ASSERT_EQ(forward.commanded_velocities.size(), backward.commanded_velocities.size());
  for (std::size_t i = 0; i < forward.commanded_velocities.size(); ++i) {
    EXPECT_DOUBLE_EQ(forward.commanded_velocities[i], -backward.commanded_velocities[i]);
  }

  const double forward_clearance =
    (0.05 * 23.0 + 0.025) - (forward.stop_pose.position.x() + HALF_FOOTPRINT);
  const double backward_clearance =
    (backward.stop_pose.position.x() - HALF_FOOTPRINT) - 0.025;
  EXPECT_NEAR(forward_clearance, backward_clearance, 1e-12);
}

TEST(VelocityLimiterClosedLoop, TheStopPoseItselfIsCollisionFree)
{
  const CollisionScenario scenario = wall_scenario(false);
  const VelocityLimiter limiter = make_limiter(VelocityLimiterParams{});

  const ClosedLoopTrace trace = run_closed_loop(
    scenario, limiter, pose_at_cell(15, 11, 0.0), Twist2D{Vector2d{-0.5, 0.0}, 0.0});

  ASSERT_TRUE(trace.stopped);
  const VelocityLimiter::Result at_rest =
    limiter.limit(scenario.map, scenario.model, trace.stop_pose, Twist2D{});
  EXPECT_FALSE(at_rest.has_collision);
  EXPECT_EQ(
    check_footprint(scenario.map, scenario.model, default_footprint(), trace.stop_pose),
    CollisionCheck::Free);
}

TEST(VelocityLimiterClosedLoop, InPlaceRotationIsNotBlockedByTheWall)
{
  const CollisionScenario scenario = wall_scenario(false);
  const VelocityLimiter limiter = make_limiter(VelocityLimiterParams{});

  SimpleSimulator plant(pose_at_cell(15, 11, 0.0));
  for (int cycle = 0; cycle < MAX_CYCLES; ++cycle) {
    const VelocityLimiter::Result result =
      limiter.limit(scenario.map, scenario.model, plant.pose(), Twist2D{Vector2d::Zero(), 0.5});
    ASSERT_FALSE(result.has_collision) << "cycle " << cycle;
    EXPECT_DOUBLE_EQ(result.command.angular, 0.5);
    plant.update(result.command, CONTROL_DT);
  }
  EXPECT_DOUBLE_EQ(plant.pose().position.x(), pose_at_cell(15, 11, 0.0).position.x());
}
