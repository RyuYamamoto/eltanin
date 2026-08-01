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

#include <eltanin/core/differential_drive.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace
{

using eltanin::ANGULAR_VEL_EPSILON;
using eltanin::integrate_differential_drive;
using eltanin::Pose2D;
using eltanin::Twist2D;
using Eigen::Vector2d;

constexpr double kPi = std::numbers::pi;

Twist2D twist(double v, double w) { return Twist2D{Vector2d{v, 0.0}, w}; }

}  // namespace

TEST(DifferentialDrive, RejectsInvalidRuntimeInput)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(integrate_differential_drive(Pose2D{}, twist(0.5, 0.0), 0.0), std::invalid_argument);
  EXPECT_THROW(
    integrate_differential_drive(Pose2D{Vector2d{nan, 0.0}, 0.0}, twist(0.5, 0.0), 0.1),
    std::invalid_argument);
  EXPECT_THROW(
    integrate_differential_drive(Pose2D{}, twist(0.5, nan), 0.1), std::invalid_argument);
}

TEST(DifferentialDrive, StraightMotionFollowsTheHeading)
{
  const Pose2D start{Vector2d{1.0, 2.0}, kPi / 3.0};
  const Pose2D next = integrate_differential_drive(start, twist(0.5, 0.0), 0.2);

  EXPECT_DOUBLE_EQ(next.position.x(), 1.0 + 0.5 * 0.2 * std::cos(kPi / 3.0));
  EXPECT_DOUBLE_EQ(next.position.y(), 2.0 + 0.5 * 0.2 * std::sin(kPi / 3.0));
  EXPECT_DOUBLE_EQ(next.yaw, kPi / 3.0);
}

TEST(DifferentialDrive, InPlaceRotationKeepsThePosition)
{
  const Pose2D start{Vector2d{1.0, 2.0}, 0.0};
  const Pose2D next = integrate_differential_drive(start, twist(0.0, 0.5), 0.2);

  EXPECT_DOUBLE_EQ(next.position.x(), 1.0);
  EXPECT_DOUBLE_EQ(next.position.y(), 2.0);
  EXPECT_DOUBLE_EQ(next.yaw, 0.1);
}

TEST(DifferentialDrive, FullCircleReturnsToTheStart)
{
  const Pose2D start{Vector2d{1.0, 2.0}, 0.3};
  constexpr double dt = 0.001;
  Pose2D pose = start;
  for (int i = 0; i < 1000; ++i) {
    pose = integrate_differential_drive(pose, twist(0.5, 2.0 * kPi), dt);
  }

  EXPECT_NEAR(pose.position.x(), start.position.x(), 1e-9);
  EXPECT_NEAR(pose.position.y(), start.position.y(), 1e-9);
  EXPECT_NEAR(pose.yaw, start.yaw, 1e-9);
}

TEST(DifferentialDrive, ArcStaysOnTheInstantaneousCircle)
{
  const Pose2D start{Vector2d{1.0, 2.0}, 0.3};
  constexpr double linear_velocity = 0.6;
  constexpr double angular_velocity = 0.4;
  const double radius = linear_velocity / angular_velocity;
  const Vector2d center =
    start.position + radius * Vector2d{std::cos(start.yaw + kPi / 2.0), std::sin(start.yaw + kPi / 2.0)};

  Pose2D pose = start;
  for (int i = 0; i < 5; ++i) {
    pose = integrate_differential_drive(pose, twist(linear_velocity, angular_velocity), 0.2);
    EXPECT_NEAR((pose.position - center).norm(), std::abs(radius), 1e-12);
  }
}

TEST(DifferentialDrive, BothSidesOfTheBranchAgreeWithinTheTruncationBound)
{
  const Pose2D start{Vector2d{1.0, 2.0}, 1.0};
  constexpr double linear_velocity = 0.5;
  constexpr double dt = 0.2;
  const double arc_side = ANGULAR_VEL_EPSILON;
  const double straight_side = ANGULAR_VEL_EPSILON * (1.0 - 1e-12);

  const Pose2D arc = integrate_differential_drive(start, twist(linear_velocity, arc_side), dt);
  const Pose2D straight =
    integrate_differential_drive(start, twist(linear_velocity, straight_side), dt);

  const double bound = linear_velocity * arc_side * dt * dt;
  EXPECT_LE((arc.position - straight.position).norm(), bound);
  EXPECT_NEAR(arc.yaw, straight.yaw, 1e-15);
}

TEST(DifferentialDrive, ZeroAngularVelocityIsTheLimitOfSmallAngularVelocity)
{
  const Pose2D start{Vector2d{1.0, 2.0}, 1.0};
  constexpr double linear_velocity = 0.5;
  constexpr double dt = 0.2;
  constexpr double angular_velocity = 1e-6;

  const Pose2D zero = integrate_differential_drive(start, twist(linear_velocity, 0.0), dt);
  const Pose2D small =
    integrate_differential_drive(start, twist(linear_velocity, angular_velocity), dt);

  EXPECT_LE(
    (zero.position - small.position).norm(), linear_velocity * angular_velocity * dt * dt);
  ASSERT_TRUE(std::isfinite(small.position.x()));
  ASSERT_TRUE(std::isfinite(small.position.y()));
}

TEST(DifferentialDrive, YawStaysNormalizedAcrossManyTurns)
{
  Pose2D pose{Vector2d::Zero(), 0.0};
  for (int i = 0; i < 200; ++i) {
    pose = integrate_differential_drive(pose, twist(0.1, 2.0), 0.5);
    EXPECT_GT(pose.yaw, -kPi);
    EXPECT_LE(pose.yaw, kPi);
  }
}

TEST(DifferentialDrive, LateralVelocityIsIgnored)
{
  const Pose2D start{Vector2d{1.0, 2.0}, 0.4};
  const Pose2D without = integrate_differential_drive(start, Twist2D{Vector2d{0.5, 0.0}, 0.3}, 0.2);
  const Pose2D with = integrate_differential_drive(start, Twist2D{Vector2d{0.5, 7.0}, 0.3}, 0.2);

  EXPECT_DOUBLE_EQ(with.position.x(), without.position.x());
  EXPECT_DOUBLE_EQ(with.position.y(), without.position.y());
  EXPECT_DOUBLE_EQ(with.yaw, without.yaw);
}

TEST(DifferentialDrive, ReverseArcMirrorsTheForwardArc)
{
  const Pose2D start{Vector2d::Zero(), 0.0};
  const Pose2D forward = integrate_differential_drive(start, twist(0.5, 0.4), 0.2);
  const Pose2D reverse = integrate_differential_drive(start, twist(-0.5, -0.4), 0.2);

  EXPECT_DOUBLE_EQ(reverse.position.x(), -forward.position.x());
  EXPECT_DOUBLE_EQ(reverse.position.y(), forward.position.y());
  EXPECT_DOUBLE_EQ(reverse.yaw, -forward.yaw);
}
