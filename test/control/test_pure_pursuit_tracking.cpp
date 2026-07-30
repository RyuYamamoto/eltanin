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
#include <eltanin/core/path.hpp>
#include <eltanin/core/types.hpp>

#include <gtest/gtest.h>

#include <cassert>
#include <numbers>

namespace
{

using eltanin::Path;
using eltanin::Pose2D;
using eltanin::control::PurePursuit;
using eltanin::control::PurePursuitParams;
using eltanin_test::make_arc_path;
using eltanin_test::make_straight_path;
using eltanin_test::NavyuFormReference;
using eltanin_test::PurePursuitDriver;
using eltanin_test::SIMULATION_DT;
using eltanin_test::simulate;
using eltanin_test::TrackingResult;
using Eigen::Vector2d;

constexpr double kPi = std::numbers::pi;

/// Path point spacing used by every tracking case; the A* + smoother output is around this [m].
constexpr double kSpacing = 0.05;

/// Travel distance after which the transient of the initial alignment is over [m].
constexpr double kGate = 1.0;

/// Thresholds fixed from the measured C++ values; see docs/control-design.md.
constexpr double kStraightOnPathTolerance = 1e-12;
constexpr double kStraightOffsetAfterGate = 0.020;
constexpr double kStraightOffsetFinal = 1e-3;
constexpr double kStraightYawAfterGate = 0.002;
constexpr double kStraightYawOverall = 0.012;
constexpr double kArcAfterGate = 0.006;
constexpr double kNavyuArcFloor = 0.020;
constexpr double kTextbookToNavyuRatio = 1.0 / 3.0;

PurePursuit make_tracker()
{
  const auto tracker = PurePursuit::create(PurePursuitParams{});
  assert(tracker.has_value());
  return *tracker;
}

TrackingResult track(const Path & path, const Pose2D & start)
{
  return simulate(PurePursuitDriver(make_tracker(), path), path, start, SIMULATION_DT, kGate);
}

void expect_command_limits(const TrackingResult & result)
{
  const PurePursuitParams defaults;
  EXPECT_LE(result.max_linear_vel, defaults.desired_linear_vel);
  EXPECT_LE(result.max_abs_angular_vel, defaults.max_angular_vel);
}

}  // namespace

TEST(PurePursuitTracking, StraightOnPathHasNoLateralError)
{
  const Path path = make_straight_path(5.0, kSpacing);
  const TrackingResult result = track(path, Pose2D{Vector2d{0.0, 0.0}, 0.0});

  EXPECT_TRUE(result.reached);
  EXPECT_LE(result.max_lateral_error, kStraightOnPathTolerance)
    << "max lateral error " << result.max_lateral_error;
  expect_command_limits(result);
}

TEST(PurePursuitTracking, StraightWithLateralOffsetConverges)
{
  const Path path = make_straight_path(5.0, kSpacing);
  const TrackingResult result = track(path, Pose2D{Vector2d{0.0, 0.2}, 0.0});

  EXPECT_TRUE(result.reached);
  EXPECT_LE(result.max_lateral_error_after_gate, kStraightOffsetAfterGate)
    << "after gate " << result.max_lateral_error_after_gate;
  EXPECT_LE(result.final_lateral_error, kStraightOffsetFinal)
    << "final " << result.final_lateral_error;
  expect_command_limits(result);
}

TEST(PurePursuitTracking, StraightWithYawErrorConverges)
{
  const Path path = make_straight_path(5.0, kSpacing);
  const TrackingResult result = track(path, Pose2D{Vector2d{0.0, 0.0}, 0.5});

  EXPECT_TRUE(result.reached);
  EXPECT_LE(result.max_lateral_error_after_gate, kStraightYawAfterGate)
    << "after gate " << result.max_lateral_error_after_gate;
  EXPECT_LE(result.max_lateral_error, kStraightYawOverall)
    << "overall " << result.max_lateral_error;
  expect_command_limits(result);
}

TEST(PurePursuitTracking, ArcLeftTrackingError)
{
  const Path path = make_arc_path(2.0, 0.5 * kPi, kSpacing, 1.0);
  const TrackingResult result = track(path, Pose2D{Vector2d{0.0, 0.0}, 0.0});

  EXPECT_TRUE(result.reached);
  EXPECT_LE(result.max_lateral_error_after_gate, kArcAfterGate)
    << "after gate " << result.max_lateral_error_after_gate;
  expect_command_limits(result);
}

TEST(PurePursuitTracking, ArcRightTrackingError)
{
  const Path path = make_arc_path(2.0, 0.5 * kPi, kSpacing, -1.0);
  const TrackingResult result = track(path, Pose2D{Vector2d{0.0, 0.0}, 0.0});

  EXPECT_TRUE(result.reached);
  EXPECT_LE(result.max_lateral_error_after_gate, kArcAfterGate)
    << "after gate " << result.max_lateral_error_after_gate;
  expect_command_limits(result);
}

TEST(PurePursuitTracking, ArcBeatsNavyuForm)
{
  const Path path = make_arc_path(2.0, 0.5 * kPi, kSpacing, 1.0);
  const Pose2D start{Vector2d{0.0, 0.0}, 0.0};

  const TrackingResult textbook = track(path, start);
  const TrackingResult navyu = simulate(
    NavyuFormReference(PurePursuitParams{}, path), path, start, SIMULATION_DT, kGate);

  ASSERT_TRUE(textbook.reached);
  ASSERT_TRUE(navyu.reached);
  // The defect is reproduced, so the comparison is against a measurement and not a magic number.
  EXPECT_GT(navyu.max_lateral_error_after_gate, kNavyuArcFloor)
    << "navyu after gate " << navyu.max_lateral_error_after_gate;
  EXPECT_LT(
    textbook.max_lateral_error_after_gate,
    kTextbookToNavyuRatio * navyu.max_lateral_error_after_gate)
    << "textbook " << textbook.max_lateral_error_after_gate << " navyu "
    << navyu.max_lateral_error_after_gate;
}

TEST(PurePursuitTracking, GoalReachedWithinHalfSpacing)
{
  const Path path = make_straight_path(5.0, kSpacing);
  const TrackingResult result = track(path, Pose2D{Vector2d{0.0, 0.0}, 0.0});

  ASSERT_TRUE(result.reached);
  const double remaining = (path[path.size() - 1].position - result.final_pose.position).norm();
  EXPECT_LE(remaining, 0.5 * kSpacing + 1e-9) << "remaining " << remaining;
}
