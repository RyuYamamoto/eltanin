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

#include <eltanin/control/pure_pursuit.hpp>

#include <control/tracking_fixture.hpp>
#include <eltanin/core/angle.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/core/types.hpp>

#include <gtest/gtest.h>

#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>

namespace
{

using eltanin::normalize_angle;
using eltanin::Path;
using eltanin::Pose2D;
using eltanin::control::FollowerState;
using eltanin::control::FollowStatus;
using eltanin::control::PurePursuit;
using eltanin::control::PurePursuitParams;
using eltanin::Twist2D;
using eltanin_test::ALIGNMENT_ANGULAR_VEL_RATIO;
using eltanin_test::LINEAR_VEL_GAIN;
using eltanin_test::make_straight_path;
using eltanin_test::SIMULATION_DT;
using Eigen::Vector2d;

constexpr double kPi = std::numbers::pi;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

PurePursuit make_tracker(const PurePursuitParams & params = PurePursuitParams{})
{
  const auto tracker = PurePursuit::create(params);
  assert(tracker.has_value());
  return *tracker;
}

/// FollowResult and the lookahead accessor read back together, as the old Result carried both.
struct Result
{
  Twist2D command{};
  FollowStatus status{FollowStatus::NoPath};
  std::size_t target_index{0};
  Vector2d lookahead_point{Vector2d::Zero()};
};

Result compute(
  PurePursuit & tracker, const Pose2D & robot, const Path & path, double dt,
  const std::optional<Twist2D> & twist = std::nullopt)
{
  const eltanin::control::FollowResult followed =
    tracker.follow(FollowerState{robot, twist}, path, dt);
  const PurePursuit::Lookahead & lookahead = tracker.lookahead();
  return Result{followed.command, followed.status, lookahead.target_index, lookahead.point};
}

/// Heading error the tracker saw, recovered from the reported lookahead point.
double alpha_of(const Result & result, const Pose2D & robot)
{
  const Vector2d delta = result.lookahead_point - robot.position;
  return normalize_angle(std::atan2(delta.y(), delta.x()) - robot.yaw);
}

void expect_zero_command(const Result & result)
{
  EXPECT_DOUBLE_EQ(result.command.linear.x(), 0.0);
  EXPECT_DOUBLE_EQ(result.command.linear.y(), 0.0);
  EXPECT_DOUBLE_EQ(result.command.angular, 0.0);
}

void expect_same_result(const Result & lhs, const Result & rhs)
{
  EXPECT_EQ(lhs.status, rhs.status);
  EXPECT_EQ(lhs.target_index, rhs.target_index);
  EXPECT_EQ(lhs.command.linear.x(), rhs.command.linear.x());
  EXPECT_EQ(lhs.command.linear.y(), rhs.command.linear.y());
  EXPECT_EQ(lhs.command.angular, rhs.command.angular);
  EXPECT_EQ(lhs.lookahead_point.x(), rhs.lookahead_point.x());
  EXPECT_EQ(lhs.lookahead_point.y(), rhs.lookahead_point.y());
}

/// Synthetic probe path: the target pose flips sides when the lookahead grows past 0.3162 m.
Path make_lookahead_probe_path()
{
  return Path{
    Pose2D{Vector2d{0.00, 0.00}, 0.0}, Pose2D{Vector2d{0.30, 0.10}, 0.0},
    Pose2D{Vector2d{0.36, -0.15}, 0.0}, Pose2D{Vector2d{0.80, -0.40}, 0.0},
    Pose2D{Vector2d{1.20, -0.70}, 0.0}};
}

}  // namespace

TEST(PurePursuit, CreateAcceptsDefaults)
{
  const auto tracker = PurePursuit::create(PurePursuitParams{});
  ASSERT_TRUE(tracker.has_value());
  const PurePursuitParams defaults;
  EXPECT_DOUBLE_EQ(tracker->params().desired_linear_vel, defaults.desired_linear_vel);
  EXPECT_DOUBLE_EQ(tracker->params().max_angular_vel, defaults.max_angular_vel);
  EXPECT_DOUBLE_EQ(tracker->params().yaw_tolerance, defaults.yaw_tolerance);
  EXPECT_DOUBLE_EQ(tracker->params().lookahead_time, defaults.lookahead_time);
  EXPECT_DOUBLE_EQ(tracker->params().min_lookahead_dist, defaults.min_lookahead_dist);
}

TEST(PurePursuit, CreateAcceptsZeroLookaheadTime)
{
  PurePursuitParams params;
  params.lookahead_time = 0.0;
  EXPECT_TRUE(PurePursuit::create(params).has_value());
}

TEST(PurePursuit, CreateRejectsNonFiniteParams)
{
  for (const double bad : {kNan, kInf, -kInf}) {
    PurePursuitParams params;
    params.desired_linear_vel = bad;
    EXPECT_FALSE(PurePursuit::create(params).has_value()) << "desired_linear_vel=" << bad;

    params = PurePursuitParams{};
    params.max_angular_vel = bad;
    EXPECT_FALSE(PurePursuit::create(params).has_value()) << "max_angular_vel=" << bad;

    params = PurePursuitParams{};
    params.yaw_tolerance = bad;
    EXPECT_FALSE(PurePursuit::create(params).has_value()) << "yaw_tolerance=" << bad;

    params = PurePursuitParams{};
    params.lookahead_time = bad;
    EXPECT_FALSE(PurePursuit::create(params).has_value()) << "lookahead_time=" << bad;

    params = PurePursuitParams{};
    params.min_lookahead_dist = bad;
    EXPECT_FALSE(PurePursuit::create(params).has_value()) << "min_lookahead_dist=" << bad;
  }
}

TEST(PurePursuit, CreateRejectsOutOfRangeParams)
{
  for (const double bad : {0.0, -0.5}) {
    PurePursuitParams params;
    params.desired_linear_vel = bad;
    EXPECT_FALSE(PurePursuit::create(params).has_value()) << "desired_linear_vel=" << bad;

    params = PurePursuitParams{};
    params.max_angular_vel = bad;
    EXPECT_FALSE(PurePursuit::create(params).has_value()) << "max_angular_vel=" << bad;

    params = PurePursuitParams{};
    params.yaw_tolerance = bad;
    EXPECT_FALSE(PurePursuit::create(params).has_value()) << "yaw_tolerance=" << bad;

    params = PurePursuitParams{};
    params.min_lookahead_dist = bad;
    EXPECT_FALSE(PurePursuit::create(params).has_value()) << "min_lookahead_dist=" << bad;
  }

  PurePursuitParams params;
  params.yaw_tolerance = kPi;
  EXPECT_FALSE(PurePursuit::create(params).has_value());

  params = PurePursuitParams{};
  params.yaw_tolerance = kPi + 0.1;
  EXPECT_FALSE(PurePursuit::create(params).has_value());

  params = PurePursuitParams{};
  params.lookahead_time = -1e-9;
  EXPECT_FALSE(PurePursuit::create(params).has_value());
}

TEST(PurePursuit, EmptyPathReturnsNoPathAndZeroCommand)
{
  PurePursuit tracker = make_tracker();
  const Path path;
  const Result result = compute(tracker, Pose2D{}, path, SIMULATION_DT);
  EXPECT_EQ(result.status, FollowStatus::NoPath);
  expect_zero_command(result);
}

TEST(PurePursuit, ComputeRejectsInvalidRuntimeInput)
{
  PurePursuit tracker = make_tracker();
  const Path path = make_straight_path(1.0, 0.05);
  EXPECT_THROW(compute(tracker, Pose2D{}, path, 0.0), std::invalid_argument);
  EXPECT_THROW(compute(tracker, Pose2D{}, path, kInf), std::invalid_argument);
  EXPECT_THROW(
    compute(tracker, Pose2D{Vector2d{kNan, 0.0}, 0.0}, path, SIMULATION_DT),
    std::invalid_argument);
}

TEST(PurePursuit, SinglePosePathReturnsGoalReached)
{
  PurePursuit tracker = make_tracker();
  const Path path{Pose2D{Vector2d{1.0, 2.0}, 0.3}};
  const Result result =
    compute(tracker, Pose2D{Vector2d{0.0, 0.0}, 0.0}, path, SIMULATION_DT);
  EXPECT_EQ(result.status, FollowStatus::GoalReached);
  expect_zero_command(result);
}

TEST(PurePursuit, GoalReachedIsIdempotent)
{
  PurePursuit tracker = make_tracker();
  const Path path = make_straight_path(1.0, 0.05);
  const Pose2D robot{Vector2d{1.0, 0.0}, 0.0};
  for (int i = 0; i < 3; ++i) {
    const Result result = compute(tracker, robot, path, SIMULATION_DT);
    EXPECT_EQ(result.status, FollowStatus::GoalReached) << "call " << i;
    expect_zero_command(result);
  }
}

TEST(PurePursuit, LastPoseNearestButOutsideTerminalToleranceKeepsTracking)
{
  PurePursuit tracker = make_tracker();
  const Path path{
    Pose2D{Vector2d{0.0, 0.0}, 0.0}, Pose2D{Vector2d{0.05, 0.0}, 0.0},
    Pose2D{Vector2d{0.10, 0.0}, 0.0}};
  const Pose2D robot{Vector2d{0.10, 0.25}, 0.0};

  const Result result = compute(tracker, robot, path, SIMULATION_DT);

  EXPECT_EQ(result.status, FollowStatus::Tracking);
  EXPECT_EQ(result.target_index, path.size() - 1);
  EXPECT_EQ(result.lookahead_point, path[path.size() - 1].position);
}

TEST(PurePursuit, RealignsInPlaceWhenTheCloseTerminalPointMovesToTheSide)
{
  PurePursuit tracker = make_tracker();
  const Path path{
    Pose2D{Vector2d{0.00, 0.0}, 0.0}, Pose2D{Vector2d{0.05, 0.0}, 0.0},
    Pose2D{Vector2d{0.10, 0.0}, 0.0}};

  const Result initial =
    compute(tracker, Pose2D{Vector2d{0.0, 0.0}, 0.0}, path, SIMULATION_DT);
  ASSERT_GT(initial.command.linear.x(), 0.0);

  const Result terminal =
    compute(tracker, Pose2D{Vector2d{0.10, 0.25}, 0.0}, path, SIMULATION_DT);

  EXPECT_EQ(terminal.status, FollowStatus::Tracking);
  EXPECT_EQ(terminal.target_index, path.size() - 1);
  EXPECT_DOUBLE_EQ(terminal.command.linear.x(), 0.0);
  EXPECT_LT(terminal.command.angular, 0.0);
}

TEST(PurePursuit, NearestPathProgressDoesNotRegressAndResetClearsIt)
{
  PurePursuit tracker = make_tracker();
  const Path path{
    Pose2D{Vector2d{0.0, 0.0}, 0.0}, Pose2D{Vector2d{1.0, 0.0}, 0.0},
    Pose2D{Vector2d{2.0, 0.0}, 0.0}, Pose2D{Vector2d{2.0, 1.0}, 0.0},
    Pose2D{Vector2d{1.0, 1.0}, 0.0}, Pose2D{Vector2d{0.0, 1.0}, 0.0}};

  const Result later =
    compute(tracker, Pose2D{Vector2d{1.0, 1.0}, std::numbers::pi}, path, SIMULATION_DT);
  ASSERT_EQ(later.target_index, 5u);

  const Result after_jump =
    compute(tracker, Pose2D{Vector2d{1.0, 0.0}, 0.0}, path, SIMULATION_DT);
  EXPECT_EQ(after_jump.target_index, 4u);

  tracker.reset();
  const Result after_reset =
    compute(tracker, Pose2D{Vector2d{1.0, 0.0}, 0.0}, path, SIMULATION_DT);
  EXPECT_EQ(after_reset.target_index, 2u);
}

TEST(PurePursuit, ResultObservablesAreZeroWhenNotTracking)
{
  PurePursuit tracker = make_tracker();
  const Path empty;
  const Result no_path = compute(tracker, Pose2D{}, empty, SIMULATION_DT);
  EXPECT_EQ(no_path.target_index, 0u);
  EXPECT_DOUBLE_EQ(no_path.lookahead_point.x(), 0.0);
  EXPECT_DOUBLE_EQ(no_path.lookahead_point.y(), 0.0);

  const Path path = make_straight_path(1.0, 0.05);
  const Result reached =
    compute(tracker, Pose2D{Vector2d{1.0, 0.0}, 0.0}, path, SIMULATION_DT);
  EXPECT_EQ(reached.status, FollowStatus::GoalReached);
  EXPECT_EQ(reached.target_index, 0u);
  EXPECT_DOUBLE_EQ(reached.lookahead_point.x(), 0.0);
  EXPECT_DOUBLE_EQ(reached.lookahead_point.y(), 0.0);
}

TEST(PurePursuit, PathIsNotModified)
{
  PurePursuit tracker = make_tracker();
  const Path path = make_straight_path(2.0, 0.05);
  const Path reference = path;

  Pose2D robot{Vector2d{0.0, 0.1}, 0.4};
  bool reached = false;
  for (std::size_t step = 0; step < 4000 && !reached; ++step) {
    const Result result = compute(tracker, robot, path, SIMULATION_DT);
    reached = result.status == FollowStatus::GoalReached;
    const double v = result.command.linear.x();
    robot.position.x() += v * std::cos(robot.yaw) * SIMULATION_DT;
    robot.position.y() += v * std::sin(robot.yaw) * SIMULATION_DT;
    robot.yaw = normalize_angle(robot.yaw + result.command.angular * SIMULATION_DT);
  }
  EXPECT_TRUE(reached);

  ASSERT_EQ(path.size(), reference.size());
  for (std::size_t i = 0; i < path.size(); ++i) {
    EXPECT_EQ(path[i].position.x(), reference[i].position.x()) << "pose " << i;
    EXPECT_EQ(path[i].position.y(), reference[i].position.y()) << "pose " << i;
    EXPECT_EQ(path[i].yaw, reference[i].yaw) << "pose " << i;
  }
}

TEST(PurePursuit, FirstCallIsDeterministicAcrossInstances)
{
  PurePursuit first = make_tracker();
  PurePursuit second = make_tracker();
  const Path path = make_lookahead_probe_path();
  const Pose2D robot{Vector2d{0.0, 0.0}, 0.0};
  expect_same_result(
    compute(first, robot, path, SIMULATION_DT), compute(second, robot, path, SIMULATION_DT));
}

TEST(PurePursuit, FirstCallLinearVelMatchesRampFromZero)
{
  PurePursuit tracker = make_tracker();
  const Path path = make_straight_path(2.0, 0.05);
  const Result result =
    compute(tracker, Pose2D{Vector2d{0.0, 0.0}, 0.0}, path, SIMULATION_DT);
  ASSERT_EQ(result.status, FollowStatus::Tracking);

  const PurePursuitParams defaults;
  const double expected =
    LINEAR_VEL_GAIN * (defaults.desired_linear_vel - 0.0) * SIMULATION_DT;
  EXPECT_DOUBLE_EQ(result.command.linear.x(), expected);
  EXPECT_DOUBLE_EQ(result.command.linear.y(), 0.0);
  EXPECT_DOUBLE_EQ(result.command.angular, 0.0);
}

TEST(PurePursuit, FirstCallLookaheadUsesZeroLinearVel)
{
  PurePursuit tracker = make_tracker();
  const Path path = make_lookahead_probe_path();
  const Result result =
    compute(tracker, Pose2D{Vector2d{0.0, 0.0}, 0.0}, path, SIMULATION_DT);

  ASSERT_EQ(result.status, FollowStatus::Tracking);
  const PurePursuitParams defaults;
  EXPECT_EQ(result.target_index, 1u);
  EXPECT_DOUBLE_EQ(result.command.linear.x(), 0.0);
  EXPECT_DOUBLE_EQ(
    result.command.angular, defaults.max_angular_vel * ALIGNMENT_ANGULAR_VEL_RATIO);
}

TEST(PurePursuit, ResetRestoresFirstCallResult)
{
  PurePursuit tracker = make_tracker();
  const Path path = make_lookahead_probe_path();
  const Pose2D robot{Vector2d{0.0, 0.0}, 0.0};
  const Result first = compute(tracker, robot, path, SIMULATION_DT);

  const Path straight = make_straight_path(2.0, 0.05);
  for (int i = 0; i < 100; ++i) {
    compute(tracker, Pose2D{Vector2d{0.2, 0.0}, 0.0}, straight, SIMULATION_DT);
  }
  tracker.reset();
  expect_same_result(compute(tracker, robot, path, SIMULATION_DT), first);
}

TEST(PurePursuit, NoPathResetsInternalState)
{
  PurePursuit tracker = make_tracker();
  const Path path = make_lookahead_probe_path();
  const Pose2D robot{Vector2d{0.0, 0.0}, 0.0};
  const Result first = compute(tracker, robot, path, SIMULATION_DT);

  const Path straight = make_straight_path(2.0, 0.05);
  for (int i = 0; i < 100; ++i) {
    compute(tracker, Pose2D{Vector2d{0.2, 0.0}, 0.0}, straight, SIMULATION_DT);
  }
  const Path empty;
  ASSERT_EQ(compute(tracker, robot, empty, SIMULATION_DT).status, FollowStatus::NoPath);
  expect_same_result(compute(tracker, robot, path, SIMULATION_DT), first);
}

TEST(PurePursuit, GoalReachedResetsInternalState)
{
  PurePursuit tracker = make_tracker();
  const Path path = make_lookahead_probe_path();
  const Pose2D robot{Vector2d{0.0, 0.0}, 0.0};
  const Result first = compute(tracker, robot, path, SIMULATION_DT);

  const Path straight = make_straight_path(2.0, 0.05);
  for (int i = 0; i < 100; ++i) {
    compute(tracker, Pose2D{Vector2d{0.2, 0.0}, 0.0}, straight, SIMULATION_DT);
  }
  ASSERT_EQ(
    compute(tracker, Pose2D{Vector2d{2.0, 0.0}, 0.0}, straight, SIMULATION_DT).status,
    FollowStatus::GoalReached);
  expect_same_result(compute(tracker, robot, path, SIMULATION_DT), first);
}

TEST(PurePursuit, AlignmentTurnsInPlaceAtFixedRate)
{
  PurePursuit tracker = make_tracker();
  const PurePursuitParams defaults;
  const Path path = make_straight_path(2.0, 0.05);
  Pose2D robot{Vector2d{0.0, 0.0}, 0.5 * kPi};

  std::size_t turning_steps = 0;
  for (std::size_t step = 0; step < 2000; ++step) {
    const Result result = compute(tracker, robot, path, SIMULATION_DT);
    ASSERT_EQ(result.status, FollowStatus::Tracking) << "step " << step;
    if (result.command.linear.x() > 0.0) {
      break;
    }
    EXPECT_DOUBLE_EQ(result.command.linear.x(), 0.0) << "step " << step;
    EXPECT_DOUBLE_EQ(result.command.linear.y(), 0.0) << "step " << step;
    EXPECT_DOUBLE_EQ(
      std::abs(result.command.angular),
      defaults.max_angular_vel * ALIGNMENT_ANGULAR_VEL_RATIO)
      << "step " << step;
    EXPECT_GE(std::abs(alpha_of(result, robot)), defaults.yaw_tolerance) << "step " << step;
    EXPECT_EQ(robot.position.x(), 0.0) << "step " << step;
    EXPECT_EQ(robot.position.y(), 0.0) << "step " << step;

    robot.yaw = normalize_angle(robot.yaw + result.command.angular * SIMULATION_DT);
    ++turning_steps;
  }
  EXPECT_GT(turning_steps, 0u);
  EXPECT_LT(turning_steps, 2000u);
}

TEST(PurePursuit, AlignmentSignFollowsAlpha)
{
  const Path path = make_straight_path(2.0, 0.05);
  for (const double yaw : {0.5 * kPi, -0.5 * kPi}) {
    PurePursuit tracker = make_tracker();
    const Pose2D robot{Vector2d{0.0, 0.0}, yaw};
    const Result result = compute(tracker, robot, path, SIMULATION_DT);
    ASSERT_EQ(result.status, FollowStatus::Tracking);
    EXPECT_DOUBLE_EQ(result.command.linear.x(), 0.0) << "yaw=" << yaw;
    EXPECT_GT(result.command.angular * alpha_of(result, robot), 0.0) << "yaw=" << yaw;
  }
}

TEST(PurePursuit, AlignmentCompletionDrivesSameCycle)
{
  PurePursuit tracker = make_tracker();
  const PurePursuitParams defaults;
  const Path path = make_straight_path(2.0, 0.05);
  Pose2D robot{Vector2d{0.0, 0.0}, 0.5 * kPi};

  bool driving = false;
  for (std::size_t step = 0; step < 2000 && !driving; ++step) {
    const Result result = compute(tracker, robot, path, SIMULATION_DT);
    ASSERT_EQ(result.status, FollowStatus::Tracking) << "step " << step;
    const double alpha = alpha_of(result, robot);
    if (std::abs(alpha) < defaults.yaw_tolerance) {
      // navyu turned once more here, and with alpha == 0 it turned the wrong way.
      EXPECT_GT(result.command.linear.x(), 0.0) << "step " << step;
      EXPECT_GE(result.command.angular * alpha, 0.0) << "step " << step;
      driving = true;
    }
    robot.position.x() += result.command.linear.x() * std::cos(robot.yaw) * SIMULATION_DT;
    robot.position.y() += result.command.linear.x() * std::sin(robot.yaw) * SIMULATION_DT;
    robot.yaw = normalize_angle(robot.yaw + result.command.angular * SIMULATION_DT);
  }
  EXPECT_TRUE(driving);
}

TEST(PurePursuit, OnceAlignedDoesNotRealignAwayFromTheEndpoint)
{
  PurePursuit tracker = make_tracker();
  const Path path = make_straight_path(2.0, 0.05);
  const Pose2D aligned{Vector2d{0.0, 0.0}, 0.0};
  ASSERT_GT(compute(tracker, aligned, path, SIMULATION_DT).command.linear.x(), 0.0);

  const Pose2D turned{Vector2d{0.2, 0.0}, 0.5 * kPi};
  const Result result = compute(tracker, turned, path, SIMULATION_DT);
  EXPECT_EQ(result.status, FollowStatus::Tracking);
  EXPECT_GT(result.command.linear.x(), 0.0);
}

TEST(PurePursuit, AlignmentConvergesAtLargeDt)
{
  // dt must stay below yaw_tolerance / (0.5 * max_angular_vel) = 0.14 s or the band is skipped.
  constexpr double dt = 0.1;
  PurePursuit tracker = make_tracker();
  const Path path = make_straight_path(2.0, 0.05);
  Pose2D robot{Vector2d{0.0, 0.0}, 0.5 * kPi};

  bool driving = false;
  for (std::size_t step = 0; step < 1000 && !driving; ++step) {
    const Result result = compute(tracker, robot, path, dt);
    ASSERT_EQ(result.status, FollowStatus::Tracking) << "step " << step;
    driving = result.command.linear.x() > 0.0;
    robot.position.x() += result.command.linear.x() * std::cos(robot.yaw) * dt;
    robot.position.y() += result.command.linear.x() * std::sin(robot.yaw) * dt;
    robot.yaw = normalize_angle(robot.yaw + result.command.angular * dt);
  }
  EXPECT_TRUE(driving);
}

TEST(PurePursuit, AngularVelocityIsClamped)
{
  PurePursuit tracker = make_tracker();
  const PurePursuitParams defaults;
  const Path straight = make_straight_path(2.0, 0.05);
  const Pose2D aligned{Vector2d{0.0, 0.0}, 0.0};
  for (int i = 0; i < 300; ++i) {
    compute(tracker, aligned, straight, SIMULATION_DT);
  }

  Path sideways;
  for (int i = 0; i <= 20; ++i) {
    sideways.push_back(Pose2D{Vector2d{0.0, 0.05 * static_cast<double>(i)}, 0.5 * kPi});
  }
  const Result result = compute(tracker, aligned, sideways, SIMULATION_DT);
  ASSERT_EQ(result.status, FollowStatus::Tracking);
  EXPECT_DOUBLE_EQ(result.command.angular, defaults.max_angular_vel);
}

TEST(PurePursuit, DuplicatePosesDoNotDivideByZero)
{
  PurePursuit tracker = make_tracker();
  const Path path{
    Pose2D{Vector2d{0.0, 0.0}, 0.0}, Pose2D{Vector2d{0.0, 0.0}, 0.0},
    Pose2D{Vector2d{0.0, 0.0}, 0.0}};
  const Result result =
    compute(tracker, Pose2D{Vector2d{0.0, 0.0}, 0.0}, path, SIMULATION_DT);

  ASSERT_EQ(result.status, FollowStatus::Tracking);
  EXPECT_TRUE(std::isfinite(result.command.linear.x()));
  EXPECT_TRUE(std::isfinite(result.command.angular));
  EXPECT_DOUBLE_EQ(result.command.angular, 0.0);
}

TEST(PurePursuit, TheMeasuredTwistIsIgnored)
{
  PurePursuit tracker = make_tracker();
  PurePursuit reference = make_tracker();
  const Path path = make_lookahead_probe_path();
  const Pose2D robot{Vector2d{0.0, 0.0}, 0.0};

  for (int i = 0; i < 5; ++i) {
    const Result with_twist =
      compute(tracker, robot, path, SIMULATION_DT, Twist2D{Vector2d{0.4, 0.0}, -0.9});
    const Result without = compute(reference, robot, path, SIMULATION_DT);
    expect_same_result(with_twist, without);
  }
}
