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

#include <eltanin/control/mpc_follower.hpp>

#include <control/mpc_fixture.hpp>
#include <control/tracking_fixture.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/core/types.hpp>

#include <gtest/gtest.h>

#include <numbers>
#include <utility>

namespace
{

using eltanin::Path;
using eltanin::Pose2D;
using eltanin::control::MpcFollowerParams;
using eltanin_test::make_arc_path;
using eltanin_test::make_mpc;
using eltanin_test::make_straight_path;
using eltanin_test::MpcDriver;
using eltanin_test::simulate;
using eltanin_test::SIMULATION_DT;
using eltanin_test::TrackingResult;
using Eigen::Vector2d;

constexpr double kPi = std::numbers::pi;
constexpr double kSpacing = 0.05;
constexpr double kGate = 1.0;

// Every bound below is the Pure Pursuit measurement of docs/control-design.md 4.1, unchanged.
constexpr double kStraightOnPathTolerance = 1e-12;
constexpr double kStraightOffsetAfterGate = 0.01350;
constexpr double kStraightYawAfterGate = 0.00052;
constexpr double kArcAfterGate = 0.00351;

TrackingResult track(const Path & path, const Pose2D & start)
{
  return simulate(MpcDriver(make_mpc(), path), path, start, SIMULATION_DT, kGate);
}

void expect_command_limits(const TrackingResult & result)
{
  const MpcFollowerParams defaults;
  EXPECT_LE(result.max_linear_vel, defaults.max_linear_vel + 1e-12);
  EXPECT_LE(result.max_abs_angular_vel, defaults.max_angular_vel + 1e-12);
}

}  // namespace

TEST(MpcTracking, StraightOnPathHasNoLateralError)
{
  const Path path = make_straight_path(5.0, kSpacing);
  const TrackingResult result = track(path, Pose2D{Vector2d{0.0, 0.0}, 0.0});

  EXPECT_TRUE(result.reached);
  EXPECT_LE(result.max_lateral_error, kStraightOnPathTolerance)
    << "max lateral error " << result.max_lateral_error;
  expect_command_limits(result);
}

TEST(MpcTracking, StraightWithLateralOffsetConverges)
{
  const Path path = make_straight_path(5.0, kSpacing);
  const TrackingResult result = track(path, Pose2D{Vector2d{0.0, 0.2}, 0.0});

  EXPECT_TRUE(result.reached);
  EXPECT_LE(result.max_lateral_error_after_gate, kStraightOffsetAfterGate)
    << "after gate " << result.max_lateral_error_after_gate;
  expect_command_limits(result);
}

TEST(MpcTracking, StraightWithYawErrorConverges)
{
  const Path path = make_straight_path(5.0, kSpacing);
  const TrackingResult result = track(path, Pose2D{Vector2d{0.0, 0.0}, 0.5});

  EXPECT_TRUE(result.reached);
  EXPECT_LE(result.max_lateral_error_after_gate, kStraightYawAfterGate)
    << "after gate " << result.max_lateral_error_after_gate;
  expect_command_limits(result);
}

TEST(MpcTracking, ArcLeftTrackingError)
{
  const Path path = make_arc_path(2.0, 0.5 * kPi, kSpacing, 1.0);
  const TrackingResult result = track(path, Pose2D{Vector2d{0.0, 0.0}, 0.0});

  EXPECT_TRUE(result.reached);
  EXPECT_LE(result.max_lateral_error_after_gate, kArcAfterGate)
    << "after gate " << result.max_lateral_error_after_gate;
  expect_command_limits(result);
}

TEST(MpcTracking, ArcRightTrackingError)
{
  const Path path = make_arc_path(2.0, 0.5 * kPi, kSpacing, -1.0);
  const TrackingResult result = track(path, Pose2D{Vector2d{0.0, 0.0}, 0.0});

  EXPECT_TRUE(result.reached);
  EXPECT_LE(result.max_lateral_error_after_gate, kArcAfterGate)
    << "after gate " << result.max_lateral_error_after_gate;
  expect_command_limits(result);
}

TEST(MpcTracking, GoalReachedWithinHalfSpacing)
{
  const Path path = make_straight_path(5.0, kSpacing);
  const TrackingResult result = track(path, Pose2D{Vector2d{0.0, 0.0}, 0.0});

  ASSERT_TRUE(result.reached);
  const double remaining = (path[path.size() - 1].position - result.final_pose.position).norm();
  EXPECT_LE(remaining, 0.5 * kSpacing + 1e-9) << "remaining " << remaining;
}

TEST(MpcTracking, TheProfileKeepsTheBodyOnATightArcAtSpeed)
{
  MpcFollowerParams fast;
  fast.max_linear_vel = 0.8;
  MpcFollowerParams regulated = fast;
  eltanin::control::VelocityProfileParams profile;
  profile.max_linear_vel = fast.max_linear_vel;
  regulated.velocity_profile = profile;

  const Path path = make_arc_path(0.4, kPi, kSpacing, 1.0);
  const Pose2D start{path[0].position, path[0].yaw};
  const TrackingResult without =
    simulate(MpcDriver(make_mpc(fast), path), path, start, SIMULATION_DT, 0.2, 5000);
  const TrackingResult with =
    simulate(MpcDriver(make_mpc(regulated), path), path, start, SIMULATION_DT, 0.2, 5000);

  EXPECT_LT(with.max_lateral_error_after_gate, without.max_lateral_error_after_gate);
  EXPECT_LE(with.max_linear_vel, fast.max_linear_vel + 1e-12);
}
