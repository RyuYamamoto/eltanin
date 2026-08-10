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

#include <eltanin/control/goal_approach.hpp>

#include <control/tracking_fixture.hpp>
#include <eltanin/control/pure_pursuit.hpp>
#include <eltanin/core/angle.hpp>
#include <eltanin/core/differential_drive.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/core/types.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <vector>

namespace
{

using eltanin::Path;
using eltanin::Pose2D;
using eltanin::Twist2D;
using eltanin::control::GoalApproach;
using eltanin::control::GoalApproachParams;
using eltanin::control::PurePursuit;
using eltanin::control::PurePursuitParams;
using eltanin::control::detail::apply_linear_limit;
using eltanin_test::make_straight_path;
using eltanin_test::simulate;
using eltanin_test::TrackingResult;
using Eigen::Vector2d;

constexpr double kPi = std::numbers::pi;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

/// Mirror of the private constant in src/control/goal_approach.cpp.
constexpr double YAW_ALIGN_GAIN = 2.0;

/// Alignment step matching the 20 Hz follower rate the ROS layer will run at [s].
constexpr double APPROACH_DT = 0.05;

constexpr std::array<double GoalApproachParams::*, 6> ALL_PARAMS{
  &GoalApproachParams::xy_goal_tolerance, &GoalApproachParams::yaw_goal_tolerance,
  &GoalApproachParams::approach_distance, &GoalApproachParams::approach_decel,
  &GoalApproachParams::yaw_align_timeout, &GoalApproachParams::max_angular_vel};

/// Every parameter but yaw_goal_tolerance, whose admissible range is checked separately.
constexpr std::array<double GoalApproachParams::*, 5> POSITIVE_PARAMS{
  &GoalApproachParams::xy_goal_tolerance, &GoalApproachParams::approach_distance,
  &GoalApproachParams::approach_decel, &GoalApproachParams::yaw_align_timeout,
  &GoalApproachParams::max_angular_vel};

GoalApproach make_approach(const GoalApproachParams & params = GoalApproachParams{})
{
  const auto approach = GoalApproach::create(params);
  assert(approach.has_value());
  return *approach;
}

void expect_zero_command(const GoalApproach::Result & result)
{
  EXPECT_DOUBLE_EQ(result.command.linear.x(), 0.0);
  EXPECT_DOUBLE_EQ(result.command.linear.y(), 0.0);
  EXPECT_DOUBLE_EQ(result.command.angular, 0.0);
}

/// Short path whose remaining arc always fits inside the approach band; goal yaw is `goal_yaw`.
Path make_goal_path(double goal_yaw)
{
  Path path = make_straight_path(0.2, 0.05);
  path[path.size() - 1].yaw = goal_yaw;
  return path;
}

/// Adapts GoalApproach to the simulate() command signature and records what it saw.
class AlignmentDriver
{
public:
  AlignmentDriver(GoalApproach & approach, const Path & path) : approach_(&approach), path_(&path)
  {
  }

  std::optional<Twist2D> operator()(const Pose2D & robot, double dt)
  {
    last_ = approach_->compute(robot, *path_, dt);
    yaw_errors_.push_back(last_.yaw_error);
    if (last_.state != GoalApproach::State::Aligning) {
      return std::nullopt;
    }
    angular_commands_.push_back(last_.command.angular);
    return last_.command;
  }

  const GoalApproach::Result & last() const noexcept { return last_; }
  const std::vector<double> & yaw_errors() const noexcept { return yaw_errors_; }
  const std::vector<double> & angular_commands() const noexcept { return angular_commands_; }

private:
  GoalApproach * approach_;
  const Path * path_;
  GoalApproach::Result last_{};
  std::vector<double> yaw_errors_{};
  std::vector<double> angular_commands_{};
};

TEST(GoalApproach, CreateAcceptsDefaults)
{
  const auto approach = GoalApproach::create(GoalApproachParams{});
  ASSERT_TRUE(approach.has_value());
  EXPECT_DOUBLE_EQ(approach->params().xy_goal_tolerance, 0.10);
  EXPECT_DOUBLE_EQ(approach->params().yaw_goal_tolerance, 0.10);
  EXPECT_DOUBLE_EQ(approach->params().approach_distance, 0.5);
  EXPECT_DOUBLE_EQ(approach->params().approach_decel, 0.5);
  EXPECT_DOUBLE_EQ(approach->params().yaw_align_timeout, 5.0);
  EXPECT_DOUBLE_EQ(approach->params().max_angular_vel, 1.0);
}

TEST(GoalApproach, ComputeRejectsInvalidRuntimeInput)
{
  GoalApproach approach = make_approach();
  const Path path = make_goal_path(0.0);
  EXPECT_THROW(approach.compute(Pose2D{}, path, 0.0), std::invalid_argument);
  EXPECT_THROW(approach.compute(Pose2D{}, path, kInf), std::invalid_argument);
  EXPECT_THROW(
    approach.compute(Pose2D{Vector2d::Zero(), kNan}, path, APPROACH_DT),
    std::invalid_argument);
}

TEST(GoalApproach, CreateRejectsNonFiniteParams)
{
  for (double GoalApproachParams::* const member : ALL_PARAMS) {
    for (const double bad : {kNan, kInf, -kInf}) {
      GoalApproachParams params;
      params.*member = bad;
      EXPECT_FALSE(GoalApproach::create(params).has_value());
    }
  }
}

TEST(GoalApproach, CreateRejectsNonPositiveParams)
{
  for (double GoalApproachParams::* const member : POSITIVE_PARAMS) {
    for (const double bad : {0.0, -1.0}) {
      GoalApproachParams params;
      params.*member = bad;
      EXPECT_FALSE(GoalApproach::create(params).has_value());
    }
  }
}

TEST(GoalApproach, CreateRejectsYawToleranceOutOfRange)
{
  for (const double bad : {0.0, -0.1, kPi, kPi + 0.1}) {
    GoalApproachParams params;
    params.yaw_goal_tolerance = bad;
    EXPECT_FALSE(GoalApproach::create(params).has_value());
  }
  GoalApproachParams params;
  params.yaw_goal_tolerance = std::nextafter(kPi, 0.0);
  EXPECT_TRUE(GoalApproach::create(params).has_value());
}

TEST(GoalApproach, CreateRejectsApproachDistanceBelowXyTolerance)
{
  GoalApproachParams narrow;
  narrow.xy_goal_tolerance = 0.10;
  narrow.approach_distance = 0.05;
  EXPECT_FALSE(GoalApproach::create(narrow).has_value());

  GoalApproachParams equal;
  equal.xy_goal_tolerance = 0.10;
  equal.approach_distance = 0.10;
  EXPECT_TRUE(GoalApproach::create(equal).has_value());
}

TEST(GoalApproach, FarFromGoalIsInactiveWithNoLimit)
{
  GoalApproach approach = make_approach();
  const Path path = make_straight_path(5.0, 0.05);
  const GoalApproach::Result result =
    approach.compute(Pose2D{Vector2d{2.0, 0.0}, 0.0}, path, APPROACH_DT);

  EXPECT_EQ(result.state, GoalApproach::State::Inactive);
  EXPECT_EQ(result.linear_vel_limit, kInf);
  EXPECT_NEAR(result.remaining_arc, 3.0, 1e-12);
  EXPECT_NEAR(result.position_error, 3.0, 1e-12);
  EXPECT_DOUBLE_EQ(result.align_elapsed, 0.0);
  expect_zero_command(result);
}

TEST(GoalApproach, ApproachLimitFollowsTheSquareRootLaw)
{
  GoalApproach approach = make_approach();
  const Path path = make_straight_path(5.0, 0.05);
  const GoalApproach::Result result =
    approach.compute(Pose2D{Vector2d{4.7, 0.0}, 0.0}, path, APPROACH_DT);

  ASSERT_EQ(result.state, GoalApproach::State::Approaching);
  EXPECT_NEAR(result.remaining_arc, 0.3, 1e-12);
  EXPECT_DOUBLE_EQ(
    result.linear_vel_limit,
    std::sqrt(2.0 * approach.params().approach_decel * result.remaining_arc));
  expect_zero_command(result);
}

TEST(GoalApproach, LimitAtBandEdgeExceedsDesiredLinearVel)
{
  GoalApproach approach = make_approach();
  const Path path = make_straight_path(5.0, 0.05);
  const GoalApproach::Result result =
    approach.compute(Pose2D{Vector2d{4.5, 0.0}, 0.0}, path, APPROACH_DT);

  ASSERT_EQ(result.state, GoalApproach::State::Approaching);
  EXPECT_DOUBLE_EQ(result.remaining_arc, approach.params().approach_distance);
  EXPECT_GT(result.linear_vel_limit, PurePursuitParams{}.desired_linear_vel);
}

TEST(GoalApproach, LimitIsMonotonicInRemainingArc)
{
  GoalApproach approach = make_approach();
  const Path path = make_straight_path(5.0, 0.05);

  double previous_arc = -1.0;
  double previous_limit = -1.0;
  std::size_t samples = 0;
  for (int step = 39; step >= 0; --step) {
    const double x = 4.5 + 0.01 * static_cast<double>(step);
    const GoalApproach::Result result =
      approach.compute(Pose2D{Vector2d{x, 0.0}, 0.0}, path, APPROACH_DT);
    if (result.state != GoalApproach::State::Approaching) {
      continue;
    }
    EXPECT_GE(result.remaining_arc, previous_arc);
    EXPECT_GE(result.linear_vel_limit, previous_limit);
    previous_arc = result.remaining_arc;
    previous_limit = result.linear_vel_limit;
    ++samples;
  }
  EXPECT_GT(samples, 30U);
}

TEST(GoalApproach, LateralDistanceFromLastPoseContributesToRemainingDistance)
{
  GoalApproach approach = make_approach();
  const Path path{Pose2D{Vector2d{0.0, 0.0}, 0.0}};
  const GoalApproach::Result result =
    approach.compute(Pose2D{Vector2d{0.3, 0.0}, 0.0}, path, APPROACH_DT);

  ASSERT_EQ(result.state, GoalApproach::State::Approaching);
  EXPECT_DOUBLE_EQ(result.remaining_arc, 0.3);
  EXPECT_DOUBLE_EQ(
    result.linear_vel_limit,
    std::sqrt(2.0 * approach.params().approach_decel * result.remaining_arc));
}

TEST(GoalApproach, TerminalStatesGiveZeroLimit)
{
  const Path path = make_goal_path(0.0);

  GoalApproach aligning = make_approach();
  const GoalApproach::Result align_result =
    aligning.compute(Pose2D{Vector2d{0.2, 0.0}, -0.5}, path, APPROACH_DT);
  ASSERT_EQ(align_result.state, GoalApproach::State::Aligning);
  EXPECT_DOUBLE_EQ(align_result.linear_vel_limit, 0.0);

  GoalApproach reached = make_approach();
  const GoalApproach::Result reached_result =
    reached.compute(Pose2D{Vector2d{0.2, 0.0}, 0.0}, path, APPROACH_DT);
  ASSERT_EQ(reached_result.state, GoalApproach::State::Reached);
  EXPECT_DOUBLE_EQ(reached_result.linear_vel_limit, 0.0);

  GoalApproach timed_out = make_approach();
  GoalApproach::Result timeout_result;
  for (int step = 0; step < 200; ++step) {
    timeout_result = timed_out.compute(Pose2D{Vector2d{0.2, 0.0}, -0.5}, path, APPROACH_DT);
  }
  ASSERT_EQ(timeout_result.state, GoalApproach::State::AlignmentTimeout);
  EXPECT_DOUBLE_EQ(timeout_result.linear_vel_limit, 0.0);
}

TEST(GoalApproach, CrossingTheApproachBandDropsTheLimit)
{
  GoalApproach approach = make_approach();
  const Path path = make_straight_path(5.0, 0.05);

  const GoalApproach::Result outside =
    approach.compute(Pose2D{Vector2d{4.4, 0.0}, 0.0}, path, APPROACH_DT);
  ASSERT_EQ(outside.state, GoalApproach::State::Inactive);
  EXPECT_EQ(outside.linear_vel_limit, kInf);

  const GoalApproach::Result inside =
    approach.compute(Pose2D{Vector2d{4.5, 0.0}, 0.0}, path, APPROACH_DT);
  ASSERT_EQ(inside.state, GoalApproach::State::Approaching);
  EXPECT_TRUE(std::isfinite(inside.linear_vel_limit));
}

TEST(GoalApproach, PathIsNotModified)
{
  GoalApproach approach = make_approach();
  const Path path = make_straight_path(5.0, 0.05);
  const Path original = path;

  approach.compute(Pose2D{Vector2d{4.7, 0.0}, 0.3}, path, APPROACH_DT);

  ASSERT_EQ(path.size(), original.size());
  for (std::size_t i = 0; i < path.size(); ++i) {
    EXPECT_EQ(path[i].position.x(), original[i].position.x());
    EXPECT_EQ(path[i].position.y(), original[i].position.y());
    EXPECT_EQ(path[i].yaw, original[i].yaw);
  }
}

TEST(GoalApproach, AlignmentConvergesFromLargeYawError)
{
  GoalApproach approach = make_approach();
  const Path path = make_goal_path(0.0);
  AlignmentDriver driver(approach, path);

  const TrackingResult run =
    simulate(driver, path, Pose2D{Vector2d{0.2, 0.0}, -3.0}, APPROACH_DT);

  EXPECT_TRUE(run.reached);
  EXPECT_EQ(driver.last().state, GoalApproach::State::Reached);
  EXPECT_LT(static_cast<double>(run.steps) * APPROACH_DT, approach.params().yaw_align_timeout);
  EXPECT_LE(std::abs(driver.last().yaw_error), approach.params().yaw_goal_tolerance);
}

TEST(GoalApproach, AlignmentConvergesFromOpposingYaw)
{
  GoalApproach approach = make_approach();
  const Path path = make_goal_path(0.0);
  AlignmentDriver driver(approach, path);

  const TrackingResult run = simulate(driver, path, Pose2D{Vector2d{0.2, 0.0}, kPi}, APPROACH_DT);

  EXPECT_TRUE(run.reached);
  EXPECT_EQ(driver.last().state, GoalApproach::State::Reached);
  EXPECT_LT(static_cast<double>(run.steps) * APPROACH_DT, 2.0 * 3.45);
}

TEST(GoalApproach, OpposingYawTurnsCounterClockwise)
{
  GoalApproach approach = make_approach();
  const Path path = make_goal_path(0.0);
  const GoalApproach::Result result =
    approach.compute(Pose2D{Vector2d{0.2, 0.0}, kPi}, path, APPROACH_DT);

  ASSERT_EQ(result.state, GoalApproach::State::Aligning);
  EXPECT_DOUBLE_EQ(result.yaw_error, kPi);
  EXPECT_GT(result.command.angular, 0.0);
}

TEST(GoalApproach, YawErrorMagnitudeNeverIncreases)
{
  GoalApproach approach = make_approach();
  const Path path = make_goal_path(0.0);
  AlignmentDriver driver(approach, path);

  const TrackingResult run = simulate(driver, path, Pose2D{Vector2d{0.2, 0.0}, kPi}, APPROACH_DT);
  ASSERT_TRUE(run.reached);
  ASSERT_GT(driver.yaw_errors().size(), 1U);

  std::size_t sign_flips = 0;
  for (std::size_t i = 1; i < driver.yaw_errors().size(); ++i) {
    EXPECT_LE(std::abs(driver.yaw_errors()[i]), std::abs(driver.yaw_errors()[i - 1]));
    if (driver.yaw_errors()[i] * driver.yaw_errors()[i - 1] < 0.0) {
      ++sign_flips;
    }
  }
  EXPECT_EQ(sign_flips, 0U);
}

TEST(GoalApproach, AlignmentRateFollowsTheProportionalLaw)
{
  const Path path = make_goal_path(0.0);

  for (const double robot_yaw : {-0.3, 0.3, -1.0, 1.0}) {
    GoalApproach approach = make_approach();
    const GoalApproach::Result result =
      approach.compute(Pose2D{Vector2d{0.2, 0.0}, robot_yaw}, path, APPROACH_DT);
    ASSERT_EQ(result.state, GoalApproach::State::Aligning);
    const double expected = std::clamp(
      YAW_ALIGN_GAIN * result.yaw_error, -approach.params().max_angular_vel,
      approach.params().max_angular_vel);
    EXPECT_DOUBLE_EQ(result.command.angular, expected);
  }

  GoalApproach unclamped = make_approach();
  const GoalApproach::Result small =
    unclamped.compute(Pose2D{Vector2d{0.2, 0.0}, -0.3}, path, APPROACH_DT);
  EXPECT_DOUBLE_EQ(small.command.angular, YAW_ALIGN_GAIN * 0.3);

  GoalApproach clamped = make_approach();
  const GoalApproach::Result large =
    clamped.compute(Pose2D{Vector2d{0.2, 0.0}, -1.0}, path, APPROACH_DT);
  EXPECT_DOUBLE_EQ(large.command.angular, clamped.params().max_angular_vel);
}

TEST(GoalApproach, AlignmentCommandHasNoLinearComponent)
{
  GoalApproach approach = make_approach();
  const Path path = make_goal_path(0.0);
  const GoalApproach::Result result =
    approach.compute(Pose2D{Vector2d{0.2, 0.0}, -1.5}, path, APPROACH_DT);

  ASSERT_EQ(result.state, GoalApproach::State::Aligning);
  EXPECT_EQ(result.command.linear.x(), 0.0);
  EXPECT_EQ(result.command.linear.y(), 0.0);
}

TEST(GoalApproach, YawErrorReadsTheGoalPoseYaw)
{
  GoalApproach approach = make_approach();
  const Pose2D robot{Vector2d{0.2, 0.0}, 0.0};

  const GoalApproach::Result shallow =
    approach.compute(robot, make_goal_path(0.4), APPROACH_DT);
  EXPECT_DOUBLE_EQ(shallow.yaw_error, 0.4);

  const GoalApproach::Result deep = approach.compute(robot, make_goal_path(0.9), APPROACH_DT);
  EXPECT_DOUBLE_EQ(deep.yaw_error, 0.9);

  Path midway_turned = make_goal_path(0.4);
  midway_turned[1].yaw = -1.2;
  const GoalApproach::Result ignored = approach.compute(robot, midway_turned, APPROACH_DT);
  EXPECT_DOUBLE_EQ(ignored.yaw_error, 0.4);
}

TEST(GoalApproach, AlignElapsedAccumulatesOnlyWhileAligning)
{
  GoalApproach approach = make_approach();
  const Path path = make_straight_path(5.0, 0.05);
  const Pose2D at_goal{Vector2d{5.0, 0.0}, -0.5};

  for (int step = 1; step <= 3; ++step) {
    const GoalApproach::Result result = approach.compute(at_goal, path, APPROACH_DT);
    ASSERT_EQ(result.state, GoalApproach::State::Aligning);
    EXPECT_DOUBLE_EQ(result.align_elapsed, static_cast<double>(step) * APPROACH_DT);
  }

  const GoalApproach::Result inactive =
    approach.compute(Pose2D{Vector2d{2.0, 0.0}, -0.5}, path, APPROACH_DT);
  ASSERT_EQ(inactive.state, GoalApproach::State::Inactive);
  EXPECT_DOUBLE_EQ(inactive.align_elapsed, 0.0);

  const GoalApproach::Result restarted = approach.compute(at_goal, path, APPROACH_DT);
  ASSERT_EQ(restarted.state, GoalApproach::State::Aligning);
  EXPECT_DOUBLE_EQ(restarted.align_elapsed, APPROACH_DT);

  const GoalApproach::Result approaching =
    approach.compute(Pose2D{Vector2d{4.7, 0.0}, -0.5}, path, APPROACH_DT);
  ASSERT_EQ(approaching.state, GoalApproach::State::Approaching);
  EXPECT_DOUBLE_EQ(approaching.align_elapsed, 0.0);
}

TEST(GoalApproach, AlignmentTimesOutWhenItNeverConverges)
{
  GoalApproach approach = make_approach();
  const Path path = make_goal_path(0.0);
  const Pose2D stuck{Vector2d{0.2, 0.0}, -0.5};

  bool timed_out = false;
  for (int step = 0; step < 200 && !timed_out; ++step) {
    timed_out =
      approach.compute(stuck, path, APPROACH_DT).state == GoalApproach::State::AlignmentTimeout;
  }
  EXPECT_TRUE(timed_out);
}

TEST(GoalApproach, AlignmentTimeoutBoundaryIsStrict)
{
  GoalApproach approach = make_approach();
  const Path path = make_goal_path(0.0);
  const Pose2D stuck{Vector2d{0.2, 0.0}, -0.5};

  GoalApproach::Result result;
  for (int step = 0; step < 100; ++step) {
    result = approach.compute(stuck, path, APPROACH_DT);
    ASSERT_EQ(result.state, GoalApproach::State::Aligning) << "cycle " << step + 1;
  }
  EXPECT_LE(result.align_elapsed, approach.params().yaw_align_timeout);

  const GoalApproach::Result overrun = approach.compute(stuck, path, APPROACH_DT);
  EXPECT_EQ(overrun.state, GoalApproach::State::AlignmentTimeout);
  EXPECT_GT(overrun.align_elapsed, approach.params().yaw_align_timeout);
}

TEST(GoalApproach, AlignmentTimeoutLatchesAndCommandsZero)
{
  GoalApproach approach = make_approach();
  const Path path = make_goal_path(0.0);
  const Pose2D stuck{Vector2d{0.2, 0.0}, -0.5};

  for (int step = 0; step < 101; ++step) {
    approach.compute(stuck, path, APPROACH_DT);
  }

  const GoalApproach::Result aligned =
    approach.compute(Pose2D{Vector2d{0.2, 0.0}, 0.0}, path, APPROACH_DT);
  EXPECT_EQ(aligned.state, GoalApproach::State::AlignmentTimeout);
  EXPECT_DOUBLE_EQ(aligned.linear_vel_limit, 0.0);
  expect_zero_command(aligned);
}

TEST(GoalApproach, ResetClearsTheTimeoutLatch)
{
  GoalApproach approach = make_approach();
  const Path path = make_goal_path(0.0);
  const Pose2D stuck{Vector2d{0.2, 0.0}, -0.5};

  for (int step = 0; step < 101; ++step) {
    approach.compute(stuck, path, APPROACH_DT);
  }
  ASSERT_EQ(
    approach.compute(stuck, path, APPROACH_DT).state, GoalApproach::State::AlignmentTimeout);

  approach.reset();
  EXPECT_EQ(approach.compute(stuck, path, APPROACH_DT).state, GoalApproach::State::Aligning);
}

TEST(GoalApproach, ResetClearsTheElapsedTime)
{
  GoalApproach approach = make_approach();
  const Path path = make_goal_path(0.0);
  const Pose2D stuck{Vector2d{0.2, 0.0}, -0.5};

  for (int step = 0; step < 5; ++step) {
    approach.compute(stuck, path, APPROACH_DT);
  }
  approach.reset();

  const GoalApproach::Result restarted = approach.compute(stuck, path, APPROACH_DT);
  ASSERT_EQ(restarted.state, GoalApproach::State::Aligning);
  EXPECT_DOUBLE_EQ(restarted.align_elapsed, APPROACH_DT);
}

TEST(GoalApproachIntegration, TerminalApproachDoesNotOrbitAfterTheGoalMovesToTheSide)
{
  GoalApproach approach = make_approach();
  const std::optional<PurePursuit> created = PurePursuit::create(PurePursuitParams{});
  ASSERT_TRUE(created.has_value());
  PurePursuit tracker = *created;
  const Path path{
    Pose2D{Vector2d{0.00, 0.0}, 0.0}, Pose2D{Vector2d{0.05, 0.0}, 0.0},
    Pose2D{Vector2d{0.10, 0.0}, 0.0}};

  // Reproduce the nav run: normal tracking has already latched its initial heading alignment.
  ASSERT_GT(
    tracker.follow(eltanin::control::FollowerState{Pose2D{Vector2d{0.0, 0.0}, 0.0}}, path, APPROACH_DT)
      .command.linear.x(),
    0.0);

  Pose2D robot{Vector2d{0.10, 0.25}, 0.0};
  double maximum_goal_distance = (path[path.size() - 1].position - robot.position).norm();
  bool reached = false;
  std::size_t steps = 0;
  for (; steps < 200; ++steps) {
    const GoalApproach::Result goal = approach.compute(robot, path, APPROACH_DT);
    if (goal.state == GoalApproach::State::Reached) {
      reached = true;
      break;
    }

    Twist2D command;
    if (goal.state == GoalApproach::State::Aligning) {
      command = goal.command;
    } else {
      const eltanin::control::FollowResult tracking =
        tracker.follow(eltanin::control::FollowerState{robot}, path, APPROACH_DT);
      ASSERT_EQ(tracking.status, eltanin::control::FollowStatus::Tracking);
      command = apply_linear_limit(tracking.command, goal.linear_vel_limit);
    }
    robot = eltanin::integrate_differential_drive(robot, command, APPROACH_DT);
    maximum_goal_distance = std::max(
      maximum_goal_distance, (path[path.size() - 1].position - robot.position).norm());
  }

  EXPECT_TRUE(reached);
  EXPECT_LT(steps, 200u);
  EXPECT_LE(maximum_goal_distance, 0.25 + 1e-12);
}

TEST(GoalApproach, EmptyPathIsInactiveWithNoLimit)
{
  GoalApproach approach = make_approach();
  const GoalApproach::Result result =
    approach.compute(Pose2D{Vector2d{1.0, 2.0}, 0.7}, Path{}, APPROACH_DT);

  EXPECT_EQ(result.state, GoalApproach::State::Inactive);
  EXPECT_EQ(result.linear_vel_limit, kInf);
  EXPECT_EQ(result.remaining_arc, kInf);
  EXPECT_EQ(result.position_error, kInf);
  EXPECT_DOUBLE_EQ(result.yaw_error, 0.0);
  EXPECT_DOUBLE_EQ(result.align_elapsed, 0.0);
  expect_zero_command(result);
}

TEST(GoalApproach, SinglePosePathUsesGoalDistanceAsRemainingArc)
{
  GoalApproach approach = make_approach();
  const Path path{Pose2D{Vector2d{1.0, 2.0}, 0.3}};
  const GoalApproach::Result result =
    approach.compute(Pose2D{Vector2d{1.2, 2.0}, 0.3}, path, APPROACH_DT);

  EXPECT_EQ(result.state, GoalApproach::State::Approaching);
  EXPECT_NEAR(result.remaining_arc, 0.2, 1e-12);
  EXPECT_DOUBLE_EQ(result.position_error, 0.2);
  EXPECT_DOUBLE_EQ(result.yaw_error, 0.0);
}

TEST(GoalApproach, StartingOnTheGoalReachesImmediately)
{
  GoalApproach approach = make_approach();
  const Path path{Pose2D{Vector2d{1.0, 2.0}, 0.3}};
  const GoalApproach::Result result =
    approach.compute(Pose2D{Vector2d{1.0, 2.0}, 0.3}, path, APPROACH_DT);

  EXPECT_EQ(result.state, GoalApproach::State::Reached);
  EXPECT_DOUBLE_EQ(result.linear_vel_limit, 0.0);
  EXPECT_DOUBLE_EQ(result.align_elapsed, 0.0);
  expect_zero_command(result);
}

TEST(GoalApproach, StartingOnTheGoalWithYawErrorAlignsFirst)
{
  GoalApproach approach = make_approach();
  const Path path{Pose2D{Vector2d{1.0, 2.0}, 0.3}};
  const GoalApproach::Result result =
    approach.compute(Pose2D{Vector2d{1.0, 2.0}, 0.8}, path, APPROACH_DT);

  EXPECT_EQ(result.state, GoalApproach::State::Aligning);
  EXPECT_DOUBLE_EQ(result.yaw_error, 0.3 - 0.8);
  EXPECT_LT(result.command.angular, 0.0);
}

TEST(GoalApproach, ReachedLatchesAgainstDisturbance)
{
  GoalApproach approach = make_approach();
  const Path path = make_straight_path(5.0, 0.05);
  ASSERT_EQ(
    approach.compute(Pose2D{Vector2d{5.0, 0.0}, 0.0}, path, APPROACH_DT).state,
    GoalApproach::State::Reached);

  const GoalApproach::Result pushed =
    approach.compute(Pose2D{Vector2d{2.0, 0.0}, 1.5}, path, APPROACH_DT);
  EXPECT_EQ(pushed.state, GoalApproach::State::Reached);
  EXPECT_DOUBLE_EQ(pushed.linear_vel_limit, 0.0);
  EXPECT_GT(pushed.remaining_arc, approach.params().approach_distance);
  expect_zero_command(pushed);
}

TEST(GoalApproach, ReachedLatchSurvivesAnEmptyPath)
{
  GoalApproach approach = make_approach();
  const Path path = make_straight_path(5.0, 0.05);
  ASSERT_EQ(
    approach.compute(Pose2D{Vector2d{5.0, 0.0}, 0.0}, path, APPROACH_DT).state,
    GoalApproach::State::Reached);

  const GoalApproach::Result gone =
    approach.compute(Pose2D{Vector2d{5.0, 0.0}, 0.0}, Path{}, APPROACH_DT);
  EXPECT_EQ(gone.state, GoalApproach::State::Reached);
  EXPECT_DOUBLE_EQ(gone.linear_vel_limit, 0.0);
}

TEST(GoalApproach, FirstCallIsDeterministicAcrossInstances)
{
  const Path path = make_straight_path(5.0, 0.05);
  const Pose2D robot{Vector2d{4.7, 0.0}, 0.2};

  GoalApproach first = make_approach();
  GoalApproach second = make_approach();
  const GoalApproach::Result lhs = first.compute(robot, path, APPROACH_DT);
  const GoalApproach::Result rhs = second.compute(robot, path, APPROACH_DT);

  EXPECT_EQ(lhs.state, rhs.state);
  EXPECT_EQ(lhs.linear_vel_limit, rhs.linear_vel_limit);
  EXPECT_EQ(lhs.remaining_arc, rhs.remaining_arc);
  EXPECT_EQ(lhs.position_error, rhs.position_error);
  EXPECT_EQ(lhs.yaw_error, rhs.yaw_error);
  EXPECT_EQ(lhs.align_elapsed, rhs.align_elapsed);
  EXPECT_EQ(lhs.command.linear.x(), rhs.command.linear.x());
  EXPECT_EQ(lhs.command.angular, rhs.command.angular);
}

TEST(GoalApproach, ApplyLinearLimitIsIdentityAtInfinity)
{
  const Twist2D cmd_in{Vector2d{0.4, 0.0}, 0.7};
  const Twist2D cmd_out = apply_linear_limit(cmd_in, kInf);

  EXPECT_EQ(cmd_out.linear.x(), cmd_in.linear.x());
  EXPECT_EQ(cmd_out.linear.y(), 0.0);
  EXPECT_EQ(cmd_out.angular, cmd_in.angular);
}

TEST(GoalApproach, ApplyLinearLimitScalesAngularByTheSameRatio)
{
  const Twist2D cmd_in{Vector2d{0.5, 0.0}, 0.8};
  const Twist2D cmd_out = apply_linear_limit(cmd_in, 0.25);

  EXPECT_DOUBLE_EQ(cmd_out.linear.x(), 0.25);
  EXPECT_DOUBLE_EQ(cmd_out.angular, 0.4);
  EXPECT_DOUBLE_EQ(cmd_out.angular / cmd_out.linear.x(), cmd_in.angular / cmd_in.linear.x());
}

TEST(GoalApproach, ApplyLinearLimitZeroesTheWholeCommand)
{
  const Twist2D cmd_out = apply_linear_limit(Twist2D{Vector2d{0.5, 0.0}, 0.8}, 0.0);

  EXPECT_DOUBLE_EQ(cmd_out.linear.x(), 0.0);
  EXPECT_DOUBLE_EQ(cmd_out.angular, 0.0);
}

TEST(GoalApproach, ApplyLinearLimitKeepsInPlaceRotation)
{
  const Twist2D cmd_out = apply_linear_limit(Twist2D{Vector2d{0.0, 0.0}, 0.6}, 0.3);

  EXPECT_DOUBLE_EQ(cmd_out.linear.x(), 0.0);
  EXPECT_DOUBLE_EQ(cmd_out.angular, 0.6);
}

TEST(GoalApproach, ApplyLinearLimitDoesNotRaiseTheCommand)
{
  const Twist2D cmd_in{Vector2d{0.2, 0.0}, 0.3};
  const Twist2D cmd_out = apply_linear_limit(cmd_in, 0.9);

  EXPECT_EQ(cmd_out.linear.x(), cmd_in.linear.x());
  EXPECT_EQ(cmd_out.angular, cmd_in.angular);
}

TEST(GoalApproach, ApplyLinearLimitCapsTheMagnitudeOfAReversingCommand)
{
  const Twist2D cmd_in{Vector2d{-0.5, 0.0}, 0.8};
  const Twist2D cmd_out = apply_linear_limit(cmd_in, 0.25);

  EXPECT_DOUBLE_EQ(cmd_out.linear.x(), -0.25);
  EXPECT_DOUBLE_EQ(cmd_out.angular, 0.4);
  EXPECT_DOUBLE_EQ(cmd_out.angular / cmd_out.linear.x(), cmd_in.angular / cmd_in.linear.x());
}

TEST(GoalApproach, ApplyLinearLimitIsExactlyMirroredBetweenTheTwoDirections)
{
  for (const double limit : {0.0, 0.1, 0.25, kInf}) {
    const Twist2D forward = apply_linear_limit(Twist2D{Vector2d{0.5, 0.0}, 0.8}, limit);
    const Twist2D backward = apply_linear_limit(Twist2D{Vector2d{-0.5, 0.0}, 0.8}, limit);
    EXPECT_DOUBLE_EQ(backward.linear.x(), -forward.linear.x()) << "limit " << limit;
    EXPECT_DOUBLE_EQ(backward.angular, forward.angular) << "limit " << limit;
  }
}

TEST(GoalApproach, ApplyLinearLimitLeavesASlowReversingCommandAlone)
{
  const Twist2D cmd_in{Vector2d{-0.2, 0.0}, 0.3};
  const Twist2D cmd_out = apply_linear_limit(cmd_in, 0.9);

  EXPECT_EQ(cmd_out.linear.x(), cmd_in.linear.x());
  EXPECT_EQ(cmd_out.angular, cmd_in.angular);
}

TEST(GoalApproach, TheRemainingArcDoesNotJumpWhenThePathDoublesBack)
{
  GoalApproach approach = make_approach();
  const Path path = eltanin_test::make_one_cusp_path(1.0, 0.5, 0.05);
  const std::size_t cusp = 20;
  ASSERT_TRUE(path.is_cusp(cusp));

  // Walking the forward run, then the reverse run, exactly as the body would.
  double previous = kInf;
  for (std::size_t i = 0; i <= cusp; ++i) {
    const GoalApproach::Result result = approach.compute(path[i], path, APPROACH_DT);
    EXPECT_LE(result.remaining_arc, previous + 1e-9) << "forward pose " << i;
    previous = result.remaining_arc;
  }
  for (std::size_t i = cusp + 1; i < path.size(); ++i) {
    const GoalApproach::Result result = approach.compute(path[i], path, APPROACH_DT);
    EXPECT_LE(result.remaining_arc, previous + 1e-9) << "reverse pose " << i;
    previous = result.remaining_arc;
  }
  EXPECT_LE(previous, GoalApproachParams{}.xy_goal_tolerance);
}

TEST(GoalApproach, WithoutMonotoneProgressTheArcWouldReadTheNearSideOfTheCusp)
{
  GoalApproach approach = make_approach();
  const Path path = eltanin_test::make_one_cusp_path(1.0, 0.5, 0.05);
  const std::size_t cusp = 20;

  // Halfway out on the forward run the body sits on top of the reverse run's last poses.
  const GoalApproach::Result early = approach.compute(path[10], path, APPROACH_DT);
  // Both legs still have to be driven, so the arc is the whole 1.5 m less what is behind.
  EXPECT_NEAR(early.remaining_arc, 1.0, 1e-9);

  const GoalApproach::Result at_cusp = approach.compute(path[cusp], path, APPROACH_DT);
  EXPECT_NEAR(at_cusp.remaining_arc, 0.5, 1e-9);
}

TEST(GoalApproach, AGoalReachedBackwardsStillDeceleratesAndLatches)
{
  GoalApproachParams params;
  params.approach_distance = 0.4;
  GoalApproach approach = make_approach(params);
  const Path path = eltanin_test::make_reverse_straight_path(1.0, 0.05);

  bool decelerated = false;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const GoalApproach::Result result = approach.compute(path[i], path, APPROACH_DT);
    if (result.state == GoalApproach::State::Approaching) {
      decelerated = true;
      EXPECT_GE(result.linear_vel_limit, 0.0);
      EXPECT_LE(
        result.linear_vel_limit,
        std::sqrt(2.0 * params.approach_decel * params.approach_distance) + 1e-9);
    }
  }
  EXPECT_TRUE(decelerated);
  const GoalApproach::Result final_result =
    approach.compute(path[path.size() - 1], path, APPROACH_DT);
  EXPECT_EQ(final_result.state, GoalApproach::State::Reached);
}

namespace
{

/// Out along +x and back 5 mm to the side: the shape a parking manoeuvre really has on the robot.
Path make_retracing_cusp_path()
{
  Path path;
  path.push_back(Pose2D{Vector2d{0.0, 0.0}, 0.0});
  for (int i = 1; i <= 20; ++i) {
    path.push_back(Pose2D{Vector2d{0.05 * i, 0.0}, 0.0}, eltanin::Direction::Forward);
  }
  for (int i = 1; i <= 20; ++i) {
    path.push_back(Pose2D{Vector2d{1.0 - 0.05 * i, 0.005}, 0.0}, eltanin::Direction::Reverse);
  }
  return path;
}

/// Tracking error puts the body nearer the return leg than the outbound one it is actually on.
Pose2D on_outbound_leg(double x)
{
  return Pose2D{Vector2d{x, 0.004}, 0.0};
}

}  // namespace

namespace
{

/// A parking wiggle whose whole extent is inside xy_goal_tolerance: two cusps in 18 cm.
Path make_tiny_cusp_path()
{
  Path path;
  path.push_back(Pose2D{Vector2d{0.0, 0.0}, 0.0});
  for (int i = 1; i <= 12; ++i) {
    path.push_back(Pose2D{Vector2d{0.01 * i, 0.0}, 0.0}, eltanin::Direction::Forward);
  }
  for (int i = 1; i <= 8; ++i) {
    path.push_back(Pose2D{Vector2d{0.12 - 0.01 * i, 0.0}, 0.5}, eltanin::Direction::Reverse);
  }
  for (int i = 1; i <= 6; ++i) {
    path.push_back(Pose2D{Vector2d{0.04 + 0.01 * i, 0.0}, 1.0}, eltanin::Direction::Forward);
  }
  return path;
}

}  // namespace

TEST(GoalApproach, ArrivalWaitsWhileACuspIsStillAhead)
{
  GoalApproach approach = make_approach();
  const Path path = make_tiny_cusp_path();
  ASSERT_TRUE(path.is_cusp(12));
  ASSERT_TRUE(path.is_cusp(20));

  // Standing on the first cusp: 2 cm from the goal and already at its heading, but two runs are
  // still to be driven. Judging on distance alone latches Reached and zeroes the command here.
  const GoalApproach::Result at_cusp =
    approach.compute(Pose2D{Vector2d{0.12, 0.0}, 1.0}, path, APPROACH_DT);
  ASSERT_LT(at_cusp.position_error, GoalApproachParams{}.xy_goal_tolerance);
  EXPECT_EQ(at_cusp.state, GoalApproach::State::Approaching);
  EXPECT_GT(at_cusp.linear_vel_limit, 0.0);
}

TEST(GoalApproach, ArrivalIsDeclaredOnceTheLastRunIsDriven)
{
  GoalApproach approach = make_approach();
  const Path path = make_tiny_cusp_path();

  for (std::size_t i = 0; i < path.size(); ++i) {
    static_cast<void>(approach.compute(path[i], path, APPROACH_DT));
  }
  const GoalApproach::Result result =
    approach.compute(path[path.size() - 1], path, APPROACH_DT);
  EXPECT_EQ(result.state, GoalApproach::State::Reached);
}

TEST(GoalApproach, ProgressDoesNotJumpOntoTheReturnLegOfACusp)
{
  GoalApproach approach = make_approach();
  const Path path = make_retracing_cusp_path();
  ASSERT_TRUE(path.is_cusp(20));

  const GoalApproach::Result result = approach.compute(on_outbound_leg(0.5), path, APPROACH_DT);

  // Half the outbound leg plus the whole return leg is still to be driven, not just half of one.
  EXPECT_NEAR(result.remaining_arc, 1.5, 0.02);
  EXPECT_EQ(result.state, GoalApproach::State::Inactive);
}

TEST(GoalApproach, TheOutboundLegIsNeverMistakenForTheApproach)
{
  GoalApproach approach = make_approach();
  const Path path = make_retracing_cusp_path();

  for (int i = 0; i <= 20; ++i) {
    const GoalApproach::Result result =
      approach.compute(on_outbound_leg(0.05 * i), path, APPROACH_DT);
    EXPECT_GE(result.remaining_arc, 1.0 - 1e-9) << "outbound pose " << i;
    EXPECT_EQ(result.state, GoalApproach::State::Inactive) << "outbound pose " << i;
  }
}

TEST(GoalApproach, TheProgressCrossesTheCuspOnceTheBodyReachesIt)
{
  GoalApproach approach = make_approach();
  const Path path = make_retracing_cusp_path();

  double previous = kInf;
  for (int i = 0; i <= 20; ++i) {
    const GoalApproach::Result result =
      approach.compute(on_outbound_leg(0.05 * i), path, APPROACH_DT);
    EXPECT_LE(result.remaining_arc, previous + 1e-9) << "outbound pose " << i;
    previous = result.remaining_arc;
  }
  for (std::size_t i = 21; i < path.size(); ++i) {
    const GoalApproach::Result result = approach.compute(path[i], path, APPROACH_DT);
    EXPECT_LE(result.remaining_arc, previous + 1e-9) << "return pose " << i;
    previous = result.remaining_arc;
  }
  EXPECT_LE(previous, GoalApproachParams{}.xy_goal_tolerance);
}

}  // namespace
