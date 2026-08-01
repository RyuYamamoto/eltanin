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

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <numbers>

namespace
{

using Eigen::Vector2d;
using eltanin::Pose2D;
using eltanin::Transform2D;
using eltanin::shortest_angular_distance;
using eltanin::planner::DubinsSegmentType;
using eltanin::planner::solve_dubins_path;

constexpr double TOLERANCE = 1e-9;

void expect_pose_near(const Pose2D & actual, const Pose2D & expected, double tolerance)
{
  EXPECT_NEAR((actual.position - expected.position).norm(), 0.0, tolerance);
  EXPECT_NEAR(shortest_angular_distance(actual.yaw, expected.yaw), 0.0, tolerance);
}

}  // namespace

TEST(DubinsPath, StraightLineHasTheExpectedLength)
{
  const Pose2D start{Vector2d{1.0, -2.0}, 0.0};
  const Pose2D goal{Vector2d{5.0, -2.0}, 0.0};

  const auto path = solve_dubins_path(start, goal, 0.5);

  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path->length(), 4.0, TOLERANCE);
  expect_pose_near(path->sample(0.0), start, TOLERANCE);
  expect_pose_near(path->sample(path->length()), goal, TOLERANCE);
}

TEST(DubinsPath, QuarterCircleHasTheExpectedLength)
{
  const Pose2D start{Vector2d{0.0, 0.0}, 0.0};
  const Pose2D goal{Vector2d{1.0, 1.0}, std::numbers::pi / 2.0};

  const auto path = solve_dubins_path(start, goal, 1.0);

  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path->length(), std::numbers::pi / 2.0, TOLERANCE);
  EXPECT_EQ(path->segments()[0].type, DubinsSegmentType::Left);
}

TEST(DubinsPath, SamplesReachManyGoalConfigurations)
{
  constexpr std::array<double, 5> yaws{
    -std::numbers::pi, -1.1, 0.0, 0.8, std::numbers::pi};
  const Pose2D start{Vector2d{-0.4, 0.7}, -0.35};

  for (double x : {-2.0, -0.1, 1.5, 3.0}) {
    for (double y : {-1.5, 0.2, 2.4}) {
      for (const double yaw : yaws) {
        const Pose2D goal{Vector2d{x, y}, yaw};
        const auto path = solve_dubins_path(start, goal, 0.6);
        ASSERT_TRUE(path.has_value()) << x << ", " << y << ", " << yaw;
        ASSERT_TRUE(std::isfinite(path->length()));
        ASSERT_GE(path->length(), 0.0);
        for (const auto & segment : path->segments()) {
          EXPECT_TRUE(std::isfinite(segment.length));
          EXPECT_GE(segment.length, 0.0);
        }

        const double before_end = std::max(0.0, path->length() - 1e-8);
        expect_pose_near(path->sample(before_end), goal, 2e-8);
        expect_pose_near(path->sample(path->length()), goal, TOLERANCE);
      }
    }
  }
}

TEST(DubinsPath, IsInvariantUnderRigidTransform)
{
  const Pose2D start{Vector2d{-0.5, 0.8}, -0.7};
  const Pose2D goal{Vector2d{2.1, -1.3}, 1.2};
  const Transform2D transform{Vector2d{4.0, -3.0}, 0.9};

  const auto original = solve_dubins_path(start, goal, 0.75);
  const auto transformed = solve_dubins_path(transform * start, transform * goal, 0.75);

  ASSERT_TRUE(original.has_value());
  ASSERT_TRUE(transformed.has_value());
  EXPECT_NEAR(original->length(), transformed->length(), TOLERANCE);
  for (int i = 0; i <= 40; ++i) {
    const double s = original->length() * static_cast<double>(i) / 40.0;
    expect_pose_near(transformed->sample(s), transform * original->sample(s), 1e-8);
  }
}

TEST(DubinsPath, SampledCurvatureDoesNotExceedTheLimit)
{
  constexpr double radius = 0.7;
  const auto path = solve_dubins_path(
    Pose2D{Vector2d{0.0, 0.0}, -0.4}, Pose2D{Vector2d{3.0, 2.0}, 2.1}, radius);
  ASSERT_TRUE(path.has_value());

  constexpr double step = 1e-4;
  for (double s = step; s + step < path->length(); s += 0.01) {
    const double delta_yaw = std::abs(shortest_angular_distance(
      path->sample(s - step).yaw, path->sample(s + step).yaw));
    EXPECT_LE(delta_yaw / (2.0 * step), 1.0 / radius + 1e-6);
  }
}

TEST(DubinsPath, RejectsInvalidInput)
{
  const Pose2D start{Vector2d{0.0, 0.0}, 0.0};
  const Pose2D goal{Vector2d{1.0, 1.0}, 0.0};
  const double nan = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(solve_dubins_path(start, goal, 0.0).has_value());
  EXPECT_FALSE(solve_dubins_path(start, goal, -1.0).has_value());
  EXPECT_FALSE(solve_dubins_path(Pose2D{Vector2d{nan, 0.0}, 0.0}, goal, 1.0).has_value());
  EXPECT_FALSE(solve_dubins_path(start, Pose2D{Vector2d{1.0, 1.0}, nan}, 1.0).has_value());
}
