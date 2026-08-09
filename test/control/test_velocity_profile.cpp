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

#include <eltanin/control/velocity_profile.hpp>

#include <control/tracking_fixture.hpp>
#include <eltanin/core/path.hpp>

#include <gtest/gtest.h>

#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <vector>

namespace
{

using eltanin::cumulative_arc_length;
using eltanin::Path;
using eltanin::Pose2D;
using eltanin::control::VelocityProfile;
using eltanin::control::VelocityProfileParams;
using eltanin_test::make_arc_path;
using eltanin_test::make_straight_path;
using Eigen::Vector2d;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kTol = 1e-9;

VelocityProfile make_profile(const VelocityProfileParams & params)
{
  const auto profile = VelocityProfile::create(params);
  assert(profile.has_value());
  return *profile;
}

/// Backward pass neutralised, so the stored bound is exactly the per-pose one.
VelocityProfileParams instantaneous_only()
{
  VelocityProfileParams params;
  params.terminal_linear_vel = params.max_linear_vel;
  return params;
}

/// A 3 m straight run into a tight left arc long enough for the curvature window to fit inside it.
Path make_straight_into_corner(double radius)
{
  const Path straight = make_straight_path(3.0, 0.05);
  const Path arc = make_arc_path(radius, std::numbers::pi, 0.05, 1.0);
  Path path = straight;
  const Vector2d offset = straight[straight.size() - 1].position;
  for (std::size_t i = 1; i < arc.size(); ++i) {
    path.push_back(Pose2D{arc[i].position + offset, arc[i].yaw});
  }
  return path;
}

double corner_bound_of(const VelocityProfileParams & params, double radius)
{
  const double curvature = 1.0 / radius;
  return std::min(
    {params.max_linear_vel, params.max_angular_vel / curvature,
     std::sqrt(params.max_lateral_accel / curvature)});
}

/// How far ahead of the corner the profile leaves max_linear_vel [m]; the run-up length.
double run_up_length(const VelocityProfile & profile, double junction_arc, double max_linear_vel)
{
  const std::vector<double> & limits = profile.limits();
  for (std::size_t i = 0; i < limits.size(); ++i) {
    if (limits[i] < max_linear_vel - 1e-12) {
      return junction_arc - profile.arc_lengths()[i];
    }
  }
  return 0.0;
}

/// The corner step has to be sharp for the backward pass alone to explain the run-up.
VelocityProfileParams sharp_curvature()
{
  VelocityProfileParams params;
  params.curvature_window = 0.0;
  return params;
}

}  // namespace

TEST(VelocityProfile, CreateAcceptsDefaults)
{
  EXPECT_TRUE(VelocityProfile::create(VelocityProfileParams{}).has_value());
}

TEST(VelocityProfile, CreateRejectsNonFiniteParams)
{
  for (const double bad : {kNan, kInf, -kInf}) {
    VelocityProfileParams params;
    params.max_linear_vel = bad;
    EXPECT_FALSE(VelocityProfile::create(params).has_value()) << "max_linear_vel=" << bad;

    params = VelocityProfileParams{};
    params.max_angular_vel = bad;
    EXPECT_FALSE(VelocityProfile::create(params).has_value()) << "max_angular_vel=" << bad;

    params = VelocityProfileParams{};
    params.max_lateral_accel = bad;
    EXPECT_FALSE(VelocityProfile::create(params).has_value()) << "max_lateral_accel=" << bad;

    params = VelocityProfileParams{};
    params.max_decel = bad;
    EXPECT_FALSE(VelocityProfile::create(params).has_value()) << "max_decel=" << bad;

    params = VelocityProfileParams{};
    params.curvature_window = bad;
    EXPECT_FALSE(VelocityProfile::create(params).has_value()) << "curvature_window=" << bad;

    params = VelocityProfileParams{};
    params.min_linear_vel = bad;
    EXPECT_FALSE(VelocityProfile::create(params).has_value()) << "min_linear_vel=" << bad;

    params = VelocityProfileParams{};
    params.terminal_linear_vel = bad;
    EXPECT_FALSE(VelocityProfile::create(params).has_value()) << "terminal_linear_vel=" << bad;
  }
}

TEST(VelocityProfile, CreateRejectsOutOfRangeParams)
{
  for (const double bad : {0.0, -0.5}) {
    VelocityProfileParams params;
    params.max_linear_vel = bad;
    EXPECT_FALSE(VelocityProfile::create(params).has_value()) << "max_linear_vel=" << bad;

    params = VelocityProfileParams{};
    params.max_angular_vel = bad;
    EXPECT_FALSE(VelocityProfile::create(params).has_value()) << "max_angular_vel=" << bad;

    params = VelocityProfileParams{};
    params.max_lateral_accel = bad;
    EXPECT_FALSE(VelocityProfile::create(params).has_value()) << "max_lateral_accel=" << bad;

    params = VelocityProfileParams{};
    params.max_decel = bad;
    EXPECT_FALSE(VelocityProfile::create(params).has_value()) << "max_decel=" << bad;
  }

  VelocityProfileParams params;
  params.curvature_window = -1e-9;
  EXPECT_FALSE(VelocityProfile::create(params).has_value());

  params = VelocityProfileParams{};
  params.min_linear_vel = params.max_linear_vel + 1e-9;
  EXPECT_FALSE(VelocityProfile::create(params).has_value());

  params = VelocityProfileParams{};
  params.terminal_linear_vel = params.max_linear_vel + 1e-9;
  EXPECT_FALSE(VelocityProfile::create(params).has_value());

  params = VelocityProfileParams{};
  params.min_linear_vel = -1e-9;
  EXPECT_FALSE(VelocityProfile::create(params).has_value());
}

TEST(VelocityProfile, ReportsNoLimitBeforeTheFirstBuild)
{
  const VelocityProfile profile = make_profile(VelocityProfileParams{});
  EXPECT_FALSE(profile.built());
  EXPECT_EQ(profile.at_index(0), kInf);
  EXPECT_EQ(profile.at_arc(0.0), kInf);
  EXPECT_EQ(profile.at_arc(5.0), kInf);
}

TEST(VelocityProfile, AStraightPathIsCappedOnlyByTheLinearBound)
{
  VelocityProfile profile = make_profile(instantaneous_only());
  const Path path = make_straight_path(5.0, 0.05);
  profile.build(path);

  ASSERT_EQ(profile.limits().size(), path.size());
  for (std::size_t i = 0; i < path.size(); ++i) {
    EXPECT_TRUE(std::isfinite(profile.at_index(i))) << "index " << i;
    EXPECT_NEAR(profile.at_index(i), VelocityProfileParams{}.max_linear_vel, kTol) << "index " << i;
  }
}

TEST(VelocityProfile, ACircularArcSitsOnTheAngularAndLateralBounds)
{
  const VelocityProfileParams params = instantaneous_only();
  for (const double radius : {0.3, 0.5, 1.0, 2.0}) {
    VelocityProfile profile = make_profile(params);
    const Path path = make_arc_path(radius, std::numbers::pi, 0.05, 1.0);
    profile.build(path);

    const std::vector<double> arc = cumulative_arc_length(path);
    const double curvature = 1.0 / radius;
    const double expected = std::min(
      {params.max_linear_vel, params.max_angular_vel / curvature,
       std::sqrt(params.max_lateral_accel / curvature)});

    std::size_t checked = 0;
    for (std::size_t i = 0; i < path.size(); ++i) {
      if (arc[i] < params.curvature_window || arc.back() - arc[i] < params.curvature_window) {
        continue;
      }
      EXPECT_NEAR(profile.at_index(i), expected, 1e-9) << "radius " << radius << " index " << i;
      EXPECT_LE(profile.at_index(i), params.max_angular_vel / curvature + kTol);
      EXPECT_LE(profile.at_index(i), std::sqrt(params.max_lateral_accel / curvature) + kTol);
      ++checked;
    }
    EXPECT_GT(checked, 0u);
  }
}

TEST(VelocityProfile, TheCreepFloorAppliesToTheCurvatureBoundOnly)
{
  VelocityProfileParams params;
  params.min_linear_vel = 0.12;
  VelocityProfile profile = make_profile(params);
  // A 6 cm radius is far below anything the body can drive, so the raw bound lands under the floor.
  const Path path = make_arc_path(0.06, std::numbers::pi, 0.01, 1.0);
  profile.build(path);

  const std::vector<double> arc = cumulative_arc_length(path);
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (arc.back() - arc[i] < 0.2) {
      continue;
    }
    EXPECT_GE(profile.at_index(i), params.min_linear_vel - kTol) << "index " << i;
  }
  EXPECT_DOUBLE_EQ(profile.limits().back(), params.terminal_linear_vel);
}

TEST(VelocityProfile, TheProfileEndsAtTheTerminalSpeed)
{
  VelocityProfile profile = make_profile(VelocityProfileParams{});
  const Path path = make_straight_path(5.0, 0.05);
  profile.build(path);
  EXPECT_DOUBLE_EQ(profile.limits().back(), 0.0);
}

TEST(VelocityProfile, EveryStepIsReachableUnderTheDeceleration)
{
  const VelocityProfileParams params;
  VelocityProfile profile = make_profile(params);
  const Path path = make_straight_into_corner(0.3);
  profile.build(path);

  const std::vector<double> & arc = profile.arc_lengths();
  const std::vector<double> & limits = profile.limits();
  for (std::size_t i = 0; i + 1 < limits.size(); ++i) {
    const double span = arc[i + 1] - arc[i];
    const double reachable =
      std::sqrt(limits[i + 1] * limits[i + 1] + 2.0 * params.max_decel * span);
    EXPECT_LE(limits[i], reachable + kTol) << "index " << i;
  }
}

TEST(VelocityProfile, TheBackwardPassIsTheBrakingLawItself)
{
  const VelocityProfileParams params = sharp_curvature();
  VelocityProfile profile = make_profile(params);
  const Path path = make_straight_into_corner(0.3);
  profile.build(path);

  const std::vector<double> & arc = profile.arc_lengths();
  const std::vector<double> & limits = profile.limits();
  // Only where the curvature imposes nothing does the stored value have to be the braking law.
  std::size_t checked = 0;
  for (std::size_t i = 1; i + 1 < limits.size(); ++i) {
    if (arc[i] > 2.9) {
      break;
    }
    const double span = arc[i + 1] - arc[i];
    const double reachable =
      std::sqrt(limits[i + 1] * limits[i + 1] + 2.0 * params.max_decel * span);
    EXPECT_DOUBLE_EQ(limits[i], std::min(params.max_linear_vel, reachable)) << "index " << i;
    ++checked;
  }
  EXPECT_GT(checked, 0u);
}

TEST(VelocityProfile, TheSpeedIsAlreadyDownWhenTheCornerStarts)
{
  const VelocityProfileParams params = sharp_curvature();
  VelocityProfile profile = make_profile(params);
  profile.build(make_straight_into_corner(0.3));

  const double corner_bound = corner_bound_of(params, 0.3);
  // The straight run is 3 m long, so the junction sits at that arc length.
  EXPECT_LT(profile.at_arc(3.0), params.max_linear_vel);
  EXPECT_LE(profile.at_arc(3.2), corner_bound + 1e-6);

  const double expected_run_up =
    (params.max_linear_vel * params.max_linear_vel - corner_bound * corner_bound) /
    (2.0 * params.max_decel);
  const double run_up = run_up_length(profile, 3.0, params.max_linear_vel);
  EXPECT_GT(run_up, 0.5 * expected_run_up);
  EXPECT_LT(run_up, 2.0 * expected_run_up);
  EXPECT_NEAR(profile.at_arc(3.0 - 2.0 * expected_run_up), params.max_linear_vel, kTol);
}

TEST(VelocityProfile, DecelerationLookaheadIsWhatBringsTheSpeedDownEarly)
{
  const VelocityProfileParams gentle = sharp_curvature();
  VelocityProfileParams abrupt = gentle;
  // A deceleration this large needs no run-up, so the backward pass stops reaching upstream.
  abrupt.max_decel = 1e4;

  VelocityProfile gentle_profile = make_profile(gentle);
  VelocityProfile abrupt_profile = make_profile(abrupt);
  const Path path = make_straight_into_corner(0.3);
  gentle_profile.build(path);
  abrupt_profile.build(path);

  EXPECT_GT(run_up_length(gentle_profile, 3.0, gentle.max_linear_vel), 0.10);
  EXPECT_LT(run_up_length(abrupt_profile, 3.0, abrupt.max_linear_vel), 0.02);
  EXPECT_LT(gentle_profile.at_arc(2.95), abrupt_profile.at_arc(2.95));
}

TEST(VelocityProfile, TheBoundIsInterpolatedAndClampedInArcLength)
{
  VelocityProfile profile = make_profile(instantaneous_only());
  const Path path = make_straight_into_corner(0.3);
  profile.build(path);

  const std::vector<double> & arc = profile.arc_lengths();
  const std::vector<double> & limits = profile.limits();
  EXPECT_DOUBLE_EQ(profile.at_arc(-1.0), limits.front());
  EXPECT_DOUBLE_EQ(profile.at_arc(arc.back() + 1.0), limits.back());
  EXPECT_DOUBLE_EQ(profile.at_arc(arc.front()), limits.front());
  EXPECT_DOUBLE_EQ(profile.at_arc(arc.back()), limits.back());

  const std::size_t middle = limits.size() / 2;
  const double halfway = 0.5 * (arc[middle] + arc[middle + 1]);
  EXPECT_NEAR(
    profile.at_arc(halfway), 0.5 * (limits[middle] + limits[middle + 1]), kTol);
}

TEST(VelocityProfile, IndexReadsAreClampedToTheLastPose)
{
  VelocityProfile profile = make_profile(VelocityProfileParams{});
  const Path path = make_straight_path(1.0, 0.05);
  profile.build(path);
  EXPECT_DOUBLE_EQ(profile.at_index(path.size()), profile.limits().back());
  EXPECT_DOUBLE_EQ(profile.at_index(1000), profile.limits().back());
}

TEST(VelocityProfile, DegeneratePathsProduceNoLimit)
{
  VelocityProfile profile = make_profile(VelocityProfileParams{});
  profile.build(Path{});
  EXPECT_FALSE(profile.built());
  EXPECT_EQ(profile.at_index(0), kInf);

  const Path single{Pose2D{Vector2d{1.0, 2.0}, 0.0}};
  profile.build(single);
  ASSERT_TRUE(profile.built());
  EXPECT_DOUBLE_EQ(profile.at_index(0), 0.0);
}

TEST(VelocityProfile, RepeatedBuildsOverwriteTheOldPath)
{
  VelocityProfile profile = make_profile(instantaneous_only());
  profile.build(make_arc_path(0.3, std::numbers::pi, 0.05, 1.0));
  const std::size_t arc_poses = profile.limits().size();

  const Path straight = make_straight_path(5.0, 0.05);
  profile.build(straight);
  EXPECT_EQ(profile.limits().size(), straight.size());
  EXPECT_NE(profile.limits().size(), arc_poses);
  EXPECT_NEAR(profile.at_index(straight.size() / 2), VelocityProfileParams{}.max_linear_vel, kTol);
}

TEST(VelocityProfile, ClearDropsTheBound)
{
  VelocityProfile profile = make_profile(VelocityProfileParams{});
  profile.build(make_straight_path(1.0, 0.05));
  ASSERT_TRUE(profile.built());
  profile.clear();
  EXPECT_FALSE(profile.built());
  EXPECT_EQ(profile.at_index(0), kInf);
}
