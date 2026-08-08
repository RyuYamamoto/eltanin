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

#include <eltanin/core/angle.hpp>
#include <eltanin/core/types.hpp>
#include <eltanin/planner/dubins_path.hpp>
#include <eltanin/planner/reeds_shepp_path.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <random>

namespace
{

using Eigen::Vector2d;
using eltanin::Pose2D;
using eltanin::Transform2D;
using eltanin::shortest_angular_distance;
using eltanin::planner::ReedsSheppSegmentType;
using eltanin::planner::solve_dubins_path;
using eltanin::planner::solve_reeds_shepp_path;

constexpr double TOLERANCE = 1e-9;

void expect_pose_near(const Pose2D & actual, const Pose2D & expected, double tolerance)
{
  EXPECT_NEAR((actual.position - expected.position).norm(), 0.0, tolerance);
  EXPECT_NEAR(shortest_angular_distance(actual.yaw, expected.yaw), 0.0, tolerance);
}

/// Poses spread over position and heading, used by every sweep below.
std::vector<std::pair<Pose2D, Pose2D>> sample_pairs(int count, double spread)
{
  std::mt19937 rng(20260809);
  std::uniform_real_distribution<double> position(-spread, spread);
  std::uniform_real_distribution<double> heading(-std::numbers::pi, std::numbers::pi);
  std::vector<std::pair<Pose2D, Pose2D>> pairs;
  pairs.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    pairs.emplace_back(
      Pose2D{Vector2d{position(rng), position(rng)}, heading(rng)},
      Pose2D{Vector2d{position(rng), position(rng)}, heading(rng)});
  }
  return pairs;
}

}  // namespace

TEST(ReedsSheppPath, StraightLineHasTheExpectedLength)
{
  const Pose2D start{Vector2d{1.0, -2.0}, 0.0};
  const Pose2D goal{Vector2d{5.0, -2.0}, 0.0};

  const auto path = solve_reeds_shepp_path(start, goal, 0.5);

  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path->length(), 4.0, TOLERANCE);
  EXPECT_EQ(path->direction_changes(), 0);
  expect_pose_near(path->sample(0.0), start, TOLERANCE);
  expect_pose_near(path->sample(path->length()), goal, TOLERANCE);
}

TEST(ReedsSheppPath, ReversesStraightBackwardsInsteadOfTurningRound)
{
  const Pose2D start{Vector2d{0.0, 0.0}, 0.0};
  const Pose2D goal{Vector2d{-2.0, 0.0}, 0.0};

  const auto path = solve_reeds_shepp_path(start, goal, 1.0);

  ASSERT_TRUE(path.has_value());
  // Backing up is two metres; the shortest forward-only alternative is far longer.
  EXPECT_NEAR(path->length(), 2.0, TOLERANCE);
  EXPECT_TRUE(path->reverse_at(1.0));
  EXPECT_EQ(path->direction_changes(), 0);
  expect_pose_near(path->sample(path->length()), goal, TOLERANCE);
}

TEST(ReedsSheppPath, QuarterCircleHasTheExpectedLength)
{
  const Pose2D start{Vector2d{0.0, 0.0}, 0.0};
  const Pose2D goal{Vector2d{1.0, 1.0}, std::numbers::pi / 2.0};

  const auto path = solve_reeds_shepp_path(start, goal, 1.0);

  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path->length(), std::numbers::pi / 2.0, TOLERANCE);
  EXPECT_EQ(path->segments()[0].type, ReedsSheppSegmentType::Left);
  EXPECT_EQ(path->direction_changes(), 0);
}

TEST(ReedsSheppPath, SamplesReachManyGoalConfigurations)
{
  constexpr std::array<double, 5> yaws{-std::numbers::pi, -1.1, 0.0, 0.8, std::numbers::pi};
  const Pose2D start{Vector2d{-0.4, 0.7}, -0.35};

  for (const double x : {-2.0, -0.1, 1.5, 3.0}) {
    for (const double y : {-1.5, 0.2, 2.4}) {
      for (const double yaw : yaws) {
        const Pose2D goal{Vector2d{x, y}, yaw};
        const auto path = solve_reeds_shepp_path(start, goal, 0.6);
        ASSERT_TRUE(path.has_value()) << x << ", " << y << ", " << yaw;
        ASSERT_TRUE(std::isfinite(path->length()));
        ASSERT_GE(path->length(), 0.0);
        for (const auto & segment : path->segments()) {
          EXPECT_TRUE(std::isfinite(segment.length));
        }

        const double before_end = std::max(0.0, path->length() - 1e-8);
        expect_pose_near(path->sample(before_end), goal, 2e-8);
        expect_pose_near(path->sample(path->length()), goal, TOLERANCE);
      }
    }
  }
}

TEST(ReedsSheppPath, IsNeverLongerThanTheForwardOnlyDubinsPath)
{
  constexpr double radius = 0.9;
  for (const auto & [start, goal] : sample_pairs(2000, 5.0)) {
    const auto reeds_shepp = solve_reeds_shepp_path(start, goal, radius);
    const auto dubins = solve_dubins_path(start, goal, radius);
    ASSERT_TRUE(reeds_shepp.has_value());
    ASSERT_TRUE(dubins.has_value());
    // Every Dubins path is also a Reeds-Shepp path, so the optimum can only be shorter.
    EXPECT_LE(reeds_shepp->length(), dubins->length() + 1e-9);
    EXPECT_GE(reeds_shepp->length(), (goal.position - start.position).norm() - 1e-9);
  }
}

TEST(ReedsSheppPath, IsASymmetricDistanceUnlikeDubins)
{
  constexpr double radius = 1.3;
  for (const auto & [start, goal] : sample_pairs(2000, 5.0)) {
    const auto forward = solve_reeds_shepp_path(start, goal, radius);
    // Driving the same word backwards connects the poses the other way round at the same cost.
    const auto backward = solve_reeds_shepp_path(goal, start, radius);
    ASSERT_TRUE(forward.has_value());
    ASSERT_TRUE(backward.has_value());
    EXPECT_NEAR(forward->length(), backward->length(), 1e-9);
  }
}

TEST(ReedsSheppPath, IsInvariantUnderRigidTransform)
{
  const Pose2D start{Vector2d{-0.5, 0.8}, -0.7};
  const Pose2D goal{Vector2d{2.1, -1.3}, 1.2};
  const Transform2D transform{Vector2d{4.0, -3.0}, 0.9};

  const auto original = solve_reeds_shepp_path(start, goal, 0.75);
  const auto transformed = solve_reeds_shepp_path(transform * start, transform * goal, 0.75);

  ASSERT_TRUE(original.has_value());
  ASSERT_TRUE(transformed.has_value());
  EXPECT_NEAR(original->length(), transformed->length(), TOLERANCE);
  for (int i = 0; i <= 40; ++i) {
    const double s = original->length() * static_cast<double>(i) / 40.0;
    expect_pose_near(transformed->sample(s), transform * original->sample(s), 1e-8);
  }
}

TEST(ReedsSheppPath, CurvatureStaysInsideTheLimitAlongEverySegment)
{
  constexpr double radius = 1.3;
  for (const auto & [start, goal] : sample_pairs(300, 5.0)) {
    const auto path = solve_reeds_shepp_path(start, goal, radius);
    ASSERT_TRUE(path.has_value());

    // Sampling one segment at a time keeps a pair from straddling a cusp, where ds goes to zero.
    double base = 0.0;
    for (const auto & segment : path->segments()) {
      const double span = std::abs(segment.length);
      if (span == 0.0) {
        continue;
      }
      constexpr int steps = 40;
      Pose2D previous = path->sample(base);
      for (int i = 1; i <= steps; ++i) {
        const Pose2D current = path->sample(base + span * static_cast<double>(i) / steps);
        const double travelled = (current.position - previous.position).norm();
        const double turned = std::abs(shortest_angular_distance(previous.yaw, current.yaw));
        if (travelled > 1e-12) {
          EXPECT_LE(turned / travelled, 1.0 / radius + 1e-4);
        }
        previous = current;
      }
      base += span;
    }
  }
}

TEST(ReedsSheppPath, ReportsWhereItDrivesInReverse)
{
  const Pose2D start{Vector2d{0.0, 0.0}, 0.0};
  const Pose2D goal{Vector2d{-1.0, 0.6}, 0.4};

  const auto path = solve_reeds_shepp_path(start, goal, 0.5);

  ASSERT_TRUE(path.has_value());
  int measured_changes = 0;
  bool previous = path->reverse_at(0.0);
  constexpr int steps = 2000;
  for (int i = 1; i <= steps; ++i) {
    const bool current = path->reverse_at(path->length() * static_cast<double>(i) / steps);
    if (current != previous) {
      ++measured_changes;
    }
    previous = current;
  }
  EXPECT_EQ(measured_changes, path->direction_changes());
}

TEST(ReedsSheppPath, RejectsInvalidInput)
{
  const Pose2D start{Vector2d{0.0, 0.0}, 0.0};
  const Pose2D goal{Vector2d{1.0, 1.0}, 0.0};
  const double nan = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(solve_reeds_shepp_path(start, goal, 0.0).has_value());
  EXPECT_FALSE(solve_reeds_shepp_path(start, goal, -1.0).has_value());
  EXPECT_FALSE(solve_reeds_shepp_path(Pose2D{Vector2d{nan, 0.0}, 0.0}, goal, 1.0).has_value());
  EXPECT_FALSE(solve_reeds_shepp_path(start, Pose2D{Vector2d{1.0, 1.0}, nan}, 1.0).has_value());
}
