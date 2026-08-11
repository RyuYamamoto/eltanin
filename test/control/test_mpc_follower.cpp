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
#include <eltanin/core/angle.hpp>
#include <eltanin/core/differential_drive.hpp>
#include <eltanin/core/path.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

using eltanin::integrate_differential_drive;
using eltanin::normalize_angle;
using eltanin::Path;
using eltanin::Pose2D;
using eltanin::Twist2D;
using eltanin::control::FollowerState;
using eltanin::control::FollowResult;
using eltanin::control::FollowStatus;
using eltanin::control::MpcFollower;
using eltanin::control::MpcFollowerParams;
using eltanin::control::VelocityProfileParams;
using eltanin_test::make_arc_path;
using eltanin_test::make_mpc;
using eltanin_test::make_straight_path;
using Eigen::Vector2d;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kDt = 0.05;

/// Drives the follower with its own commands and hands back every cycle it produced.
std::vector<FollowResult> drive(
  MpcFollower & follower, const Path & path, Pose2D pose, int cycles)
{
  std::vector<FollowResult> results;
  std::optional<Twist2D> measured;
  for (int i = 0; i < cycles; ++i) {
    const FollowResult result = follower.follow(FollowerState{pose, measured}, path, kDt);
    results.push_back(result);
    if (result.status == FollowStatus::GoalReached || result.status == FollowStatus::NoPath) {
      break;
    }
    measured = result.command;
    pose = integrate_differential_drive(pose, result.command, kDt);
  }
  return results;
}

}  // namespace

TEST(MpcFollower, CreateAcceptsDefaults)
{
  const auto follower = MpcFollower::create(MpcFollowerParams{});
  ASSERT_TRUE(follower.has_value());
  const MpcFollowerParams defaults;
  EXPECT_EQ(follower->params().prediction_horizon, defaults.prediction_horizon);
  EXPECT_DOUBLE_EQ(follower->params().prediction_dt, defaults.prediction_dt);
}

TEST(MpcFollower, CreateRejectsNonFiniteParams)
{
  for (const double bad : {kNan, kInf, -kInf}) {
    MpcFollowerParams params;
    params.prediction_dt = bad;
    EXPECT_FALSE(MpcFollower::create(params).has_value()) << "prediction_dt=" << bad;

    params = MpcFollowerParams{};
    params.max_linear_vel = bad;
    EXPECT_FALSE(MpcFollower::create(params).has_value()) << "max_linear_vel=" << bad;

    params = MpcFollowerParams{};
    params.max_angular_accel = bad;
    EXPECT_FALSE(MpcFollower::create(params).has_value()) << "max_angular_accel=" << bad;

    params = MpcFollowerParams{};
    params.weight_lateral = bad;
    EXPECT_FALSE(MpcFollower::create(params).has_value()) << "weight_lateral=" << bad;

    params = MpcFollowerParams{};
    params.weight_linear_vel = bad;
    EXPECT_FALSE(MpcFollower::create(params).has_value()) << "weight_linear_vel=" << bad;
  }
}

TEST(MpcFollower, CreateRejectsOutOfRangeParams)
{
  MpcFollowerParams params;
  params.prediction_horizon = 0;
  EXPECT_FALSE(MpcFollower::create(params).has_value());

  params = MpcFollowerParams{};
  params.prediction_dt = 0.0;
  EXPECT_FALSE(MpcFollower::create(params).has_value());

  params = MpcFollowerParams{};
  params.min_linear_vel = params.max_linear_vel + 1e-9;
  EXPECT_FALSE(MpcFollower::create(params).has_value());

  params = MpcFollowerParams{};
  params.max_angular_vel = 0.0;
  EXPECT_FALSE(MpcFollower::create(params).has_value());

  params = MpcFollowerParams{};
  params.max_linear_accel = 0.0;
  EXPECT_FALSE(MpcFollower::create(params).has_value());

  params = MpcFollowerParams{};
  params.weight_lateral = -1.0;
  EXPECT_FALSE(MpcFollower::create(params).has_value());

  params = MpcFollowerParams{};
  // A singular input weight leaves the QP without a unique minimiser.
  params.weight_angular_vel = 0.0;
  EXPECT_FALSE(MpcFollower::create(params).has_value());

  params = MpcFollowerParams{};
  params.yaw_tolerance = std::numbers::pi;
  EXPECT_FALSE(MpcFollower::create(params).has_value());

  params = MpcFollowerParams{};
  params.max_heading_error = params.yaw_tolerance - 1e-9;
  EXPECT_FALSE(MpcFollower::create(params).has_value());

  params = MpcFollowerParams{};
  params.solver.max_iterations = 0;
  EXPECT_FALSE(MpcFollower::create(params).has_value());

  params = MpcFollowerParams{};
  params.velocity_profile = VelocityProfileParams{};
  params.velocity_profile->max_decel = -1.0;
  EXPECT_FALSE(MpcFollower::create(params).has_value());
}

TEST(MpcFollower, DegeneratePathsComeFromTheBase)
{
  MpcFollower follower = make_mpc();
  const FollowResult empty = follower.follow(FollowerState{}, Path{}, kDt);
  EXPECT_EQ(empty.status, FollowStatus::NoPath);
  EXPECT_DOUBLE_EQ(empty.command.linear.x(), 0.0);

  const Path single{Pose2D{Vector2d{1.0, 1.0}, 0.0}};
  const FollowResult one = follower.follow(FollowerState{}, single, kDt);
  EXPECT_EQ(one.status, FollowStatus::GoalReached);

  EXPECT_THROW(
    static_cast<void>(follower.follow(FollowerState{}, make_straight_path(1.0, 0.05), 0.0)),
    std::invalid_argument);
}

TEST(MpcFollower, ItTurnsInPlaceBeforeTheLinearisationCanHold)
{
  MpcFollower follower = make_mpc();
  const Path path = make_straight_path(5.0, 0.05);
  const FollowResult result =
    follower.follow(FollowerState{Pose2D{Vector2d{0.0, 0.0}, 1.5}}, path, kDt);

  EXPECT_EQ(result.status, FollowStatus::Tracking);
  EXPECT_DOUBLE_EQ(result.command.linear.x(), 0.0);
  EXPECT_LT(result.command.angular, 0.0);
  EXPECT_GE(result.command.angular, -follower.params().max_angular_vel);
  EXPECT_NEAR(follower.prediction().yaw_error, 1.5, 1e-9);
}

TEST(MpcFollower, TheAlignmentLatchesAndThenTheSolverTakesOver)
{
  MpcFollower follower = make_mpc();
  const Path path = make_straight_path(5.0, 0.05);
  const std::vector<FollowResult> results =
    drive(follower, path, Pose2D{Vector2d{0.0, 0.0}, 1.5}, 200);

  ASSERT_FALSE(results.empty());
  bool moved = false;
  for (const FollowResult & result : results) {
    if (result.command.linear.x() > 0.0) {
      moved = true;
      break;
    }
  }
  EXPECT_TRUE(moved);
  EXPECT_FALSE(follower.prediction().predicted.empty());
  EXPECT_EQ(
    follower.prediction().predicted.size(),
    static_cast<std::size_t>(follower.params().prediction_horizon) + 1);
}

TEST(MpcFollower, TheCommandStaysInsideEveryBound)
{
  MpcFollower follower = make_mpc();
  const Path path = make_arc_path(0.6, std::numbers::pi, 0.05, -1.0);
  const MpcFollowerParams & params = follower.params();
  const std::vector<FollowResult> results =
    drive(follower, path, Pose2D{path[0].position, path[0].yaw}, 400);

  double previous_linear = 0.0;
  double previous_angular = 0.0;
  for (const FollowResult & result : results) {
    if (result.status != FollowStatus::Tracking) {
      continue;
    }
    EXPECT_GE(result.command.linear.x(), params.min_linear_vel - 1e-9);
    EXPECT_LE(result.command.linear.x(), params.max_linear_vel + 1e-9);
    EXPECT_LE(std::abs(result.command.angular), params.max_angular_vel + 1e-9);
    EXPECT_DOUBLE_EQ(result.command.linear.y(), 0.0);
    EXPECT_TRUE(std::isfinite(result.command.linear.x()));
    EXPECT_TRUE(std::isfinite(result.command.angular));
    previous_linear = result.command.linear.x();
    previous_angular = result.command.angular;
  }
  EXPECT_TRUE(std::isfinite(previous_linear));
  EXPECT_TRUE(std::isfinite(previous_angular));
}

TEST(MpcFollower, ItReachesTheGoalAndReportsIt)
{
  MpcFollower follower = make_mpc();
  const Path path = make_straight_path(2.0, 0.05);
  const std::vector<FollowResult> results =
    drive(follower, path, Pose2D{Vector2d{0.0, 0.0}, 0.0}, 400);

  ASSERT_FALSE(results.empty());
  EXPECT_EQ(results.back().status, FollowStatus::GoalReached);
  EXPECT_DOUBLE_EQ(results.back().command.linear.x(), 0.0);
  EXPECT_DOUBLE_EQ(results.back().command.angular, 0.0);
}

TEST(MpcFollower, TwoInstancesFollowTheSameSequenceIdentically)
{
  MpcFollower first = make_mpc();
  MpcFollower second = make_mpc();
  const Path path = make_arc_path(1.5, std::numbers::pi, 0.05, 1.0);
  const Pose2D start{Vector2d{0.05, -0.05}, 0.1};

  const std::vector<FollowResult> left = drive(first, path, start, 200);
  const std::vector<FollowResult> right = drive(second, path, start, 200);

  ASSERT_EQ(left.size(), right.size());
  for (std::size_t i = 0; i < left.size(); ++i) {
    EXPECT_EQ(left[i].status, right[i].status) << "cycle " << i;
    EXPECT_EQ(left[i].command.linear.x(), right[i].command.linear.x()) << "cycle " << i;
    EXPECT_EQ(left[i].command.angular, right[i].command.angular) << "cycle " << i;
  }
}

TEST(MpcFollower, ResetMakesItBehaveLikeANewInstance)
{
  MpcFollower reused = make_mpc();
  MpcFollower fresh = make_mpc();
  const Path warmup = make_arc_path(1.0, std::numbers::pi, 0.05, -1.0);
  const Path path = make_straight_path(3.0, 0.05);
  const Pose2D start{Vector2d{0.0, 0.1}, 0.0};

  static_cast<void>(drive(reused, warmup, Pose2D{warmup[0].position, warmup[0].yaw}, 50));
  reused.reset();

  const std::vector<FollowResult> after = drive(reused, path, start, 100);
  const std::vector<FollowResult> reference = drive(fresh, path, start, 100);

  ASSERT_EQ(after.size(), reference.size());
  for (std::size_t i = 0; i < after.size(); ++i) {
    EXPECT_EQ(after[i].command.linear.x(), reference[i].command.linear.x()) << "cycle " << i;
    EXPECT_EQ(after[i].command.angular, reference[i].command.angular) << "cycle " << i;
  }
}

TEST(MpcFollower, ThePathIsNotModified)
{
  MpcFollower follower = make_mpc();
  const Path path = make_arc_path(1.0, std::numbers::pi, 0.05, 1.0);
  Path copy = path;
  static_cast<void>(drive(follower, copy, Pose2D{Vector2d{0.0, 0.1}, 0.0}, 100));

  ASSERT_EQ(copy.size(), path.size());
  for (std::size_t i = 0; i < path.size(); ++i) {
    EXPECT_EQ(copy[i].position.x(), path[i].position.x()) << "pose " << i;
    EXPECT_EQ(copy[i].position.y(), path[i].position.y()) << "pose " << i;
    EXPECT_EQ(copy[i].yaw, path[i].yaw) << "pose " << i;
  }
}

TEST(MpcFollower, AnUnsolvableIterationCapDeceleratesAndThenStops)
{
  MpcFollowerParams params;
  // One iteration never converges, so every cycle takes the failure path.
  params.solver.max_iterations = 1;
  params.solver.eps_abs = 1e-14;
  params.solver.eps_rel = 1e-14;
  params.max_consecutive_failures = 2;
  MpcFollower follower = make_mpc(params);

  const Path path = make_straight_path(5.0, 0.05);
  const Pose2D pose{Vector2d{0.0, 0.0}, 0.0};
  std::optional<Twist2D> measured = Twist2D{Vector2d{0.5, 0.0}, 0.2};

  std::vector<FollowResult> results;
  for (int i = 0; i < 4; ++i) {
    const FollowResult result = follower.follow(FollowerState{pose, measured}, path, kDt);
    results.push_back(result);
    measured = result.command;
  }

  for (const FollowResult & result : results) {
    EXPECT_EQ(result.status, FollowStatus::SolverFailed);
  }
  EXPECT_LT(results[0].command.linear.x(), 0.5);
  EXPECT_GT(results[0].command.linear.x(), 0.0);
  EXPECT_LT(results[1].command.linear.x(), results[0].command.linear.x());
  EXPECT_DOUBLE_EQ(results[2].command.linear.x(), 0.0);
  EXPECT_DOUBLE_EQ(results[2].command.angular, 0.0);
  EXPECT_GE(follower.solver_stats().consecutive_failures, params.max_consecutive_failures);
}

TEST(MpcFollower, TheFallbackKeepsTheCommandedCurvature)
{
  MpcFollowerParams params;
  params.solver.max_iterations = 1;
  params.solver.eps_abs = 1e-14;
  params.solver.eps_rel = 1e-14;
  MpcFollower follower = make_mpc(params);

  const Path path = make_straight_path(5.0, 0.05);
  const Twist2D measured{Vector2d{0.5, 0.0}, 0.25};
  const FollowResult result =
    follower.follow(FollowerState{Pose2D{}, measured}, path, kDt);

  ASSERT_EQ(result.status, FollowStatus::SolverFailed);
  EXPECT_NEAR(
    result.command.angular / result.command.linear.x(), measured.angular / measured.linear.x(),
    1e-9);
}

TEST(MpcFollower, TheReferenceAndThePredictionAreReadableAfterASolve)
{
  MpcFollower follower = make_mpc();
  const Path path = make_straight_path(5.0, 0.05);
  static_cast<void>(follower.follow(FollowerState{Pose2D{Vector2d{1.0, 0.05}, 0.0}}, path, kDt));

  const auto horizon = static_cast<std::size_t>(follower.params().prediction_horizon) + 1;
  EXPECT_EQ(follower.prediction().reference.size(), horizon);
  EXPECT_EQ(follower.prediction().predicted.size(), horizon);
  EXPECT_NEAR(follower.prediction().lateral_error, 0.05, 1e-9);
  EXPECT_NEAR(follower.prediction().yaw_error, 0.0, 1e-9);
  EXPECT_NE(std::string(follower.solver_stats().status), "unsolved");
  EXPECT_GT(follower.solver_stats().iterations, 0);
  EXPECT_EQ(follower.solver_stats().consecutive_failures, 0);
}

TEST(MpcFollower, ItStaysFiniteOnAPathThatTurnsInPlace)
{
  MpcFollower follower = make_mpc();
  Path path;
  for (int i = 0; i < 8; ++i) {
    path.push_back(Pose2D{Vector2d{0.0, 0.0}, 0.2 * static_cast<double>(i)});
  }
  for (int i = 1; i <= 20; ++i) {
    const double travel = 0.05 * static_cast<double>(i);
    path.push_back(Pose2D{
      Vector2d{travel * std::cos(1.4), travel * std::sin(1.4)}, 1.4});
  }

  ASSERT_FALSE(path.has_reverse());
  const std::vector<FollowResult> results =
    drive(follower, path, Pose2D{Vector2d{0.0, 0.0}, 0.0}, 200);
  for (const FollowResult & result : results) {
    EXPECT_TRUE(std::isfinite(result.command.linear.x()));
    EXPECT_TRUE(std::isfinite(result.command.angular));
    EXPECT_GE(result.command.linear.x(), 0.0);
  }
}

TEST(MpcFollower, ItIsMovableSoOptionalCanCarryIt)
{
  std::optional<MpcFollower> follower = MpcFollower::create(MpcFollowerParams{});
  ASSERT_TRUE(follower.has_value());
  MpcFollower moved = std::move(*follower);
  const Path path = make_straight_path(1.0, 0.05);
  const FollowResult result = moved.follow(FollowerState{}, path, kDt);
  EXPECT_EQ(result.status, FollowStatus::Tracking);
}

TEST(MpcFollower, TheHeadingGateReopensWhenThePathJumpsAway)
{
  MpcFollower follower = make_mpc();
  const Path path = make_straight_path(5.0, 0.05);
  static_cast<void>(follower.follow(FollowerState{Pose2D{Vector2d{0.0, 0.0}, 0.0}}, path, kDt));

  const double beyond = follower.params().max_heading_error + 0.2;
  const FollowResult turned =
    follower.follow(FollowerState{Pose2D{Vector2d{0.5, 0.0}, beyond}}, path, kDt);
  EXPECT_EQ(turned.status, FollowStatus::Tracking);
  EXPECT_DOUBLE_EQ(turned.command.linear.x(), 0.0);
  EXPECT_LT(turned.command.angular, 0.0);
  EXPECT_NEAR(normalize_angle(follower.prediction().yaw_error), beyond, 1e-9);
}

namespace
{

/// Parameters that allow reversing at the same speed as driving forwards.
MpcFollowerParams reversing_params()
{
  MpcFollowerParams params;
  params.max_linear_vel = 0.3;
  params.min_linear_vel = -0.3;
  return params;
}

/// Largest number of poses any of these paths need to be driven end to end.
constexpr int kCycles = 800;

}  // namespace

TEST(MpcFollower, CreateAcceptsANegativeMinLinearVelDownToMinusMax)
{
  MpcFollowerParams params;
  params.max_linear_vel = 0.3;

  params.min_linear_vel = -0.3;
  EXPECT_TRUE(MpcFollower::create(params).has_value());
  params.min_linear_vel = -0.1;
  EXPECT_TRUE(MpcFollower::create(params).has_value());
  params.min_linear_vel = -0.3 - 1e-9;
  EXPECT_FALSE(MpcFollower::create(params).has_value());
  params.min_linear_vel = 0.4;
  EXPECT_FALSE(MpcFollower::create(params).has_value());
}

TEST(MpcFollower, AReversingPathIsRefusedWhenNothingCanExecuteIt)
{
  MpcFollower follower = make_mpc();
  ASSERT_DOUBLE_EQ(follower.params().min_linear_vel, 0.0);
  const Path path = eltanin_test::make_reverse_straight_path(1.0, 0.05);

  const FollowResult result =
    follower.follow(FollowerState{Pose2D{Vector2d{0.0, 0.0}, 0.0}}, path, kDt);
  EXPECT_EQ(result.status, FollowStatus::PathNotSupported);
  EXPECT_DOUBLE_EQ(result.command.linear.x(), 0.0);
}

TEST(MpcFollower, AllReverseIsDrivenBackwardsToTheGoal)
{
  MpcFollower follower = make_mpc(reversing_params());
  const Path path = eltanin_test::make_reverse_straight_path(1.0, 0.05);

  const std::vector<FollowResult> results =
    drive(follower, path, Pose2D{Vector2d{0.0, 0.0}, 0.0}, kCycles);

  ASSERT_FALSE(results.empty());
  EXPECT_EQ(results.back().status, FollowStatus::GoalReached);
  bool moved_backwards = false;
  for (const FollowResult & result : results) {
    EXPECT_LE(result.command.linear.x(), 1e-9);
    moved_backwards = moved_backwards || result.command.linear.x() < -0.05;
  }
  EXPECT_TRUE(moved_backwards);
}

TEST(MpcFollower, TheCommandNeverStepsAcrossZeroOnACuspPath)
{
  MpcFollower follower = make_mpc(reversing_params());
  const Path path = eltanin_test::make_one_cusp_path(1.0, 0.5, 0.05);

  const std::vector<FollowResult> results =
    drive(follower, path, Pose2D{Vector2d{0.0, 0.0}, 0.0}, kCycles);

  ASSERT_GE(results.size(), 2u);
  for (std::size_t i = 1; i < results.size(); ++i) {
    const double before = results[i - 1].command.linear.x();
    const double after = results[i].command.linear.x();
    EXPECT_GE(before * after, 0.0) << "cycle " << i << " went " << before << " -> " << after;
  }
}

TEST(MpcFollower, ItStopsAtTheCuspAndThenTakesUpTheReverseRun)
{
  MpcFollower follower = make_mpc(reversing_params());
  const Path path = eltanin_test::make_one_cusp_path(1.0, 0.5, 0.05);

  EXPECT_EQ(follower.run_progress().run_index, 0u);
  const std::vector<FollowResult> results =
    drive(follower, path, Pose2D{Vector2d{0.0, 0.0}, 0.0}, kCycles);

  ASSERT_FALSE(results.empty());
  EXPECT_EQ(results.back().status, FollowStatus::GoalReached);
  bool drove_forward = false;
  bool drove_backward = false;
  for (const FollowResult & result : results) {
    drove_forward = drove_forward || result.command.linear.x() > 0.05;
    drove_backward = drove_backward || result.command.linear.x() < -0.05;
  }
  EXPECT_TRUE(drove_forward);
  EXPECT_TRUE(drove_backward);
}

TEST(MpcFollower, TheRunProgressNamesTheDirectionAndTheCuspAhead)
{
  MpcFollower follower = make_mpc(reversing_params());
  const Path path = eltanin_test::make_one_cusp_path(1.0, 0.5, 0.05);
  const std::size_t cusp = 20;
  ASSERT_TRUE(path.is_cusp(cusp));

  static_cast<void>(follower.follow(FollowerState{Pose2D{Vector2d{0.0, 0.0}, 0.0}}, path, kDt));
  EXPECT_EQ(follower.run_progress().direction, eltanin::Direction::Forward);
  EXPECT_TRUE(follower.run_progress().has_cusp);
  EXPECT_EQ(follower.run_progress().cusp_index, cusp);

  std::optional<Twist2D> measured;
  Pose2D pose{Vector2d{0.0, 0.0}, 0.0};
  for (int i = 0; i < kCycles; ++i) {
    const FollowResult result = follower.follow(FollowerState{pose, measured}, path, kDt);
    if (follower.run_progress().run_index == 1) {
      break;
    }
    measured = result.command;
    pose = integrate_differential_drive(pose, result.command, kDt);
  }
  EXPECT_EQ(follower.run_progress().run_index, 1u);
  EXPECT_EQ(follower.run_progress().direction, eltanin::Direction::Reverse);
  EXPECT_FALSE(follower.run_progress().has_cusp);
}

TEST(MpcFollower, TwoCuspsAreExecutedOneRunAtATime)
{
  MpcFollower follower = make_mpc(reversing_params());
  const Path path = eltanin_test::make_two_cusp_path(1.0, 0.05);

  std::vector<FollowResult> results;
  std::size_t runs_seen = 0;
  std::optional<Twist2D> measured;
  Pose2D pose{Vector2d{0.0, 0.0}, 0.0};
  for (int i = 0; i < kCycles; ++i) {
    const FollowResult result = follower.follow(FollowerState{pose, measured}, path, kDt);
    results.push_back(result);
    runs_seen = std::max(runs_seen, follower.run_progress().run_index);
    if (result.status == FollowStatus::GoalReached) {
      break;
    }
    measured = result.command;
    pose = integrate_differential_drive(pose, result.command, kDt);
  }

  ASSERT_FALSE(results.empty());
  EXPECT_EQ(results.back().status, FollowStatus::GoalReached);
  EXPECT_EQ(runs_seen, 2u);
  for (std::size_t i = 1; i < results.size(); ++i) {
    EXPECT_GE(results[i - 1].command.linear.x() * results[i].command.linear.x(), 0.0)
      << "cycle " << i;
  }
}

TEST(MpcFollower, AReverseRunDoesNotSendItTurningOnTheSpot)
{
  MpcFollower follower = make_mpc(reversing_params());
  const Path path = eltanin_test::make_reverse_straight_path(1.0, 0.05);

  const std::vector<FollowResult> results =
    drive(follower, path, Pose2D{Vector2d{0.0, 0.0}, 0.0}, kCycles);

  ASSERT_GE(results.size(), 5u);
  // Alignment turns command exactly zero speed; a reverse run must not need any of them.
  std::size_t stationary = 0;
  for (const FollowResult & result : results) {
    stationary += result.command.linear.x() == 0.0 ? 1u : 0u;
  }
  EXPECT_LE(stationary, 2u);
  EXPECT_NEAR(follower.prediction().yaw_error, 0.0, 0.2);
}

namespace
{

/// Forward along x, a turn on the spot at the corner, then reverse back the way it came.
Path make_spin_between_runs(double spin)
{
  std::vector<Pose2D> poses;
  std::vector<eltanin::Direction> directions;
  for (std::size_t i = 0; i <= 10; ++i) {
    poses.push_back(Pose2D{Eigen::Vector2d{0.05 * static_cast<double>(i), 0.0}, 0.0});
    if (i > 0) {
      directions.push_back(eltanin::Direction::Forward);
    }
  }
  // The spin shares its position with the pose before it; only the yaw changes.
  poses.push_back(Pose2D{poses.back().position, spin});
  directions.push_back(eltanin::Direction::InPlace);
  for (std::size_t i = 1; i <= 10; ++i) {
    const double back = 0.5 - 0.05 * static_cast<double>(i);
    poses.push_back(Pose2D{Eigen::Vector2d{back * std::cos(spin), back * std::sin(spin)}, spin});
    directions.push_back(eltanin::Direction::Reverse);
  }
  return Path{std::move(poses), std::move(directions)};
}

}  // namespace

TEST(MpcFollowerInPlaceRun, ExecutesTheTurnOnTheSpotInsteadOfSteppingOverIt)
{
  // A turn on the spot is a run of zero arc length, which the arc-based cusp test reports as
  // reached the moment it is taken up. Stepping over it starts the next run a half turn off.
  const double spin = 1.2;
  const Path path = make_spin_between_runs(spin);
  eltanin::control::MpcFollowerParams params;
  params.min_linear_vel = -0.3;
  eltanin_test::MpcDriver drive(eltanin_test::make_mpc(params), path);

  Pose2D robot{Eigen::Vector2d{0.5, 0.0}, 0.0};
  bool turned = false;
  for (int cycle = 0; cycle < 200; ++cycle) {
    const std::optional<Twist2D> command = drive(robot, 0.05);
    if (!command.has_value()) {
      break;
    }
    robot = integrate_differential_drive(robot, *command, 0.05);
    if (std::abs(normalize_angle(robot.yaw - spin)) <= params.yaw_tolerance) {
      turned = true;
      break;
    }
  }

  EXPECT_TRUE(turned) << "the body never reached the yaw the in-place run asks for; it is at "
                      << robot.yaw << " instead of " << spin;
  EXPECT_NEAR(robot.position.x(), 0.5, 0.02);
  EXPECT_NEAR(robot.position.y(), 0.0, 0.02);
}

TEST(MpcFollowerInPlaceRun, TheCommandDuringTheTurnIsAPureRotation)
{
  const Path path = make_spin_between_runs(1.2);
  eltanin::control::MpcFollowerParams params;
  params.min_linear_vel = -0.3;
  eltanin_test::MpcDriver drive(eltanin_test::make_mpc(params), path);

  Pose2D robot{Eigen::Vector2d{0.5, 0.0}, 0.0};
  const std::optional<Twist2D> command = drive(robot, 0.05);

  ASSERT_TRUE(command.has_value());
  EXPECT_DOUBLE_EQ(command->linear.x(), 0.0);
  EXPECT_GT(std::abs(command->angular), 0.0);
}
