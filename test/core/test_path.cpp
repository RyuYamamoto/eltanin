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

#include <eltanin/core/path.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <numbers>
#include <utility>
#include <vector>

namespace
{

using eltanin::cumulative_arc_length;
using eltanin::Direction;
using eltanin::Path;
using eltanin::path_length;
using eltanin::Pose2D;
using Eigen::Vector2d;

constexpr double kTol = 1e-12;

}  // namespace

TEST(Path, DefaultIsEmpty)
{
  const Path path;
  EXPECT_TRUE(path.empty());
  EXPECT_EQ(path.size(), 0u);
  EXPECT_NEAR(path_length(path), 0.0, kTol);
}

TEST(Path, SinglePoseHasZeroLength)
{
  const Path path{Pose2D{Vector2d{1.0, 2.0}, 0.5}};
  EXPECT_FALSE(path.empty());
  EXPECT_EQ(path.size(), 1u);
  EXPECT_NEAR(path_length(path), 0.0, kTol);
}

TEST(Path, StraightLineLength)
{
  const Path path{
    Pose2D{Vector2d{0.0, 0.0}, 0.0}, Pose2D{Vector2d{3.0, 0.0}, 0.0},
    Pose2D{Vector2d{5.0, 0.0}, 0.0}};
  EXPECT_NEAR(path_length(path), 5.0, kTol);
}

TEST(Path, PolylineLength)
{
  const Path path{
    Pose2D{Vector2d{0.0, 0.0}, 0.0}, Pose2D{Vector2d{3.0, 4.0}, 0.0},
    Pose2D{Vector2d{3.0, 0.0}, 0.0}};
  EXPECT_NEAR(path_length(path), 9.0, kTol);
}

TEST(Path, PushBackAndIndexing)
{
  Path path;
  path.push_back(Pose2D{Vector2d{0.0, 0.0}, 0.0});
  path.push_back(Pose2D{Vector2d{1.0, 0.0}, 0.25});
  ASSERT_EQ(path.size(), 2u);
  EXPECT_NEAR(path[1].position.x(), 1.0, kTol);
  EXPECT_NEAR(path[1].yaw, 0.25, kTol);

  path[1].yaw = 0.5;
  EXPECT_NEAR(path[1].yaw, 0.5, kTol);

  path.clear();
  EXPECT_TRUE(path.empty());
}

TEST(Path, IterationVisitsEveryPose)
{
  const Path path{
    Pose2D{Vector2d{0.0, 0.0}, 0.0}, Pose2D{Vector2d{1.0, 0.0}, 0.0},
    Pose2D{Vector2d{2.0, 0.0}, 0.0}};
  double sum_x = 0.0;
  for (const Pose2D & pose : path) {
    sum_x += pose.position.x();
  }
  EXPECT_NEAR(sum_x, 3.0, kTol);
}

TEST(Path, CumulativeArcLengthOfEmptyPathIsEmpty)
{
  const Path path;
  EXPECT_TRUE(cumulative_arc_length(path).empty());
}

TEST(Path, CumulativeArcLengthOfSinglePoseIsZero)
{
  const Path path{Pose2D{Vector2d{1.0, 2.0}, 0.5}};
  const std::vector<double> lengths = cumulative_arc_length(path);
  ASSERT_EQ(lengths.size(), 1u);
  EXPECT_DOUBLE_EQ(lengths[0], 0.0);
}

TEST(Path, CumulativeArcLengthOnEvenlySpacedLine)
{
  constexpr double spacing = 0.25;
  constexpr std::size_t count = 9;
  Path path;
  for (std::size_t i = 0; i < count; ++i) {
    path.push_back(Pose2D{Vector2d{spacing * static_cast<double>(i), 0.0}, 0.0});
  }

  const std::vector<double> lengths = cumulative_arc_length(path);
  ASSERT_EQ(lengths.size(), count);
  for (std::size_t i = 0; i < count; ++i) {
    EXPECT_DOUBLE_EQ(lengths[i], spacing * static_cast<double>(i)) << "index " << i;
  }
}

TEST(Path, CumulativeArcLengthOnPolyline)
{
  const Path path{
    Pose2D{Vector2d{0.0, 0.0}, 0.0}, Pose2D{Vector2d{3.0, 4.0}, 0.0},
    Pose2D{Vector2d{3.0, 0.0}, 0.0}, Pose2D{Vector2d{-3.0, 0.0}, 0.0}};

  const std::vector<double> lengths = cumulative_arc_length(path);
  ASSERT_EQ(lengths.size(), 4u);
  EXPECT_DOUBLE_EQ(lengths[0], 0.0);
  EXPECT_DOUBLE_EQ(lengths[1], 5.0);
  EXPECT_DOUBLE_EQ(lengths[2], 9.0);
  EXPECT_DOUBLE_EQ(lengths[3], 15.0);
}

TEST(Path, CumulativeArcLengthIsNonDecreasing)
{
  Path path;
  for (int i = 0; i < 40; ++i) {
    const double angle = 0.31 * static_cast<double>(i);
    path.push_back(Pose2D{Vector2d{std::cos(angle), 0.5 * std::sin(angle)}, 0.0});
  }

  const std::vector<double> lengths = cumulative_arc_length(path);
  ASSERT_EQ(lengths.size(), path.size());
  for (std::size_t i = 1; i < lengths.size(); ++i) {
    EXPECT_GE(lengths[i], lengths[i - 1]) << "index " << i;
  }
}

TEST(Path, CumulativeArcLengthBackMatchesPathLength)
{
  Path path;
  for (int i = 0; i < 40; ++i) {
    const double angle = 0.31 * static_cast<double>(i);
    path.push_back(Pose2D{Vector2d{std::cos(angle), 0.5 * std::sin(angle)}, 0.0});
  }
  EXPECT_DOUBLE_EQ(cumulative_arc_length(path).back(), path_length(path));

  const Path single{Pose2D{Vector2d{1.0, 2.0}, 0.0}};
  EXPECT_DOUBLE_EQ(cumulative_arc_length(single).back(), path_length(single));
}

TEST(Path, CumulativeArcLengthKeepsDuplicatePosesFlat)
{
  const Path path{
    Pose2D{Vector2d{0.0, 0.0}, 0.0}, Pose2D{Vector2d{1.0, 0.0}, 0.0},
    Pose2D{Vector2d{1.0, 0.0}, 0.5}, Pose2D{Vector2d{2.0, 0.0}, 0.0}};

  const std::vector<double> lengths = cumulative_arc_length(path);
  ASSERT_EQ(lengths.size(), 4u);
  EXPECT_DOUBLE_EQ(lengths[1], 1.0);
  EXPECT_DOUBLE_EQ(lengths[2], lengths[1]);
  EXPECT_DOUBLE_EQ(lengths[3], 2.0);
}

TEST(Path, ConstructionFromVector)
{
  std::vector<Pose2D> poses{Pose2D{Vector2d{0.0, 0.0}, 0.0}, Pose2D{Vector2d{0.0, 2.0}, 0.0}};
  const Path path(std::move(poses));
  EXPECT_EQ(path.size(), 2u);
  EXPECT_NEAR(path_length(path), 2.0, kTol);
  EXPECT_EQ(path.poses().size(), 2u);
}

TEST(PathDirections, AnUndirectedPathIsAllForward)
{
  const Path path{
    Pose2D{Vector2d{0.0, 0.0}, 0.0}, Pose2D{Vector2d{1.0, 0.0}, 0.0},
    Pose2D{Vector2d{2.0, 0.0}, 0.0}};
  EXPECT_FALSE(path.has_directions());
  EXPECT_TRUE(path.directions().empty());
  EXPECT_FALSE(path.has_reverse());
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    EXPECT_EQ(path.direction_of(i), Direction::Forward);
    EXPECT_FALSE(path.is_cusp(i));
  }
  EXPECT_EQ(path.run_bounds(1), std::make_pair(std::size_t{0}, std::size_t{2}));
}

TEST(PathDirections, ConstructionCarriesOneEntryPerSegment)
{
  std::vector<Pose2D> poses{
    Pose2D{Vector2d{0.0, 0.0}, 0.0}, Pose2D{Vector2d{1.0, 0.0}, 0.0},
    Pose2D{Vector2d{2.0, 0.0}, 0.0}};
  std::vector<Direction> directions{Direction::Forward, Direction::Reverse};
  const Path path(std::move(poses), std::move(directions));
  EXPECT_TRUE(path.has_directions());
  EXPECT_EQ(path.directions().size(), path.size() - 1);
  EXPECT_EQ(path.direction_of(0), Direction::Forward);
  EXPECT_EQ(path.direction_of(1), Direction::Reverse);
  EXPECT_TRUE(path.has_reverse());
}

TEST(PathDirections, DirectedPushBackPromotesAnUndirectedPath)
{
  Path path;
  path.push_back(Pose2D{Vector2d{0.0, 0.0}, 0.0});
  path.push_back(Pose2D{Vector2d{1.0, 0.0}, 0.0});
  EXPECT_FALSE(path.has_directions());
  path.push_back(Pose2D{Vector2d{0.5, 0.0}, 0.0}, Direction::Reverse);
  EXPECT_TRUE(path.has_directions());
  EXPECT_EQ(path.directions().size(), 2u);
  EXPECT_EQ(path.direction_of(0), Direction::Forward);
  EXPECT_EQ(path.direction_of(1), Direction::Reverse);
}

TEST(PathDirections, UndirectedPushBackKeepsTheArrayAsLongAsTheSegments)
{
  Path path;
  path.push_back(Pose2D{Vector2d{0.0, 0.0}, 0.0});
  path.push_back(Pose2D{Vector2d{1.0, 0.0}, 0.0}, Direction::Reverse);
  path.push_back(Pose2D{Vector2d{2.0, 0.0}, 0.0});
  EXPECT_EQ(path.directions().size(), path.size() - 1);
  EXPECT_EQ(path.direction_of(1), Direction::Forward);
}

TEST(PathDirections, ClearDropsTheDirections)
{
  Path path;
  path.push_back(Pose2D{Vector2d{0.0, 0.0}, 0.0});
  path.push_back(Pose2D{Vector2d{1.0, 0.0}, 0.0}, Direction::Reverse);
  path.clear();
  EXPECT_TRUE(path.empty());
  EXPECT_FALSE(path.has_directions());
}

TEST(PathDirections, ACuspIsThePoseWhoseSegmentsDisagree)
{
  std::vector<Pose2D> poses{
    Pose2D{Vector2d{0.0, 0.0}, 0.0}, Pose2D{Vector2d{1.0, 0.0}, 0.0},
    Pose2D{Vector2d{2.0, 0.0}, 0.0}, Pose2D{Vector2d{1.5, 0.0}, 0.0},
    Pose2D{Vector2d{1.0, 0.0}, 0.0}};
  std::vector<Direction> directions{
    Direction::Forward, Direction::Forward, Direction::Reverse, Direction::Reverse};
  const Path path(std::move(poses), std::move(directions));
  EXPECT_FALSE(path.is_cusp(0));
  EXPECT_FALSE(path.is_cusp(1));
  EXPECT_TRUE(path.is_cusp(2));
  EXPECT_FALSE(path.is_cusp(3));
  EXPECT_FALSE(path.is_cusp(4));
}

TEST(PathDirections, RunBoundsStopAtCusps)
{
  std::vector<Pose2D> poses{
    Pose2D{Vector2d{0.0, 0.0}, 0.0}, Pose2D{Vector2d{1.0, 0.0}, 0.0},
    Pose2D{Vector2d{2.0, 0.0}, 0.0}, Pose2D{Vector2d{1.5, 0.0}, 0.0},
    Pose2D{Vector2d{1.0, 0.0}, 0.0}};
  std::vector<Direction> directions{
    Direction::Forward, Direction::Forward, Direction::Reverse, Direction::Reverse};
  const Path path(std::move(poses), std::move(directions));
  EXPECT_EQ(path.run_bounds(0), std::make_pair(std::size_t{0}, std::size_t{2}));
  EXPECT_EQ(path.run_bounds(1), std::make_pair(std::size_t{0}, std::size_t{2}));
  EXPECT_EQ(path.run_bounds(2), std::make_pair(std::size_t{2}, std::size_t{4}));
  EXPECT_EQ(path.run_bounds(4), std::make_pair(std::size_t{2}, std::size_t{4}));
}

TEST(PathDirections, RunBoundsOnDegeneratePaths)
{
  const Path empty;
  EXPECT_EQ(empty.run_bounds(0), std::make_pair(std::size_t{0}, std::size_t{0}));
  const Path single{Pose2D{Vector2d{0.0, 0.0}, 0.0}};
  EXPECT_EQ(single.run_bounds(7), std::make_pair(std::size_t{0}, std::size_t{0}));
}

TEST(PathDirections, InPlaceIsItsOwnRun)
{
  std::vector<Pose2D> poses{
    Pose2D{Vector2d{0.0, 0.0}, 0.0}, Pose2D{Vector2d{1.0, 0.0}, 0.0},
    Pose2D{Vector2d{1.0, 0.0}, 1.0}, Pose2D{Vector2d{1.0, 1.0}, 1.0}};
  std::vector<Direction> directions{Direction::Forward, Direction::InPlace, Direction::Forward};
  const Path path(std::move(poses), std::move(directions));
  EXPECT_FALSE(path.has_reverse());
  EXPECT_TRUE(path.is_cusp(1));
  EXPECT_TRUE(path.is_cusp(2));
  EXPECT_EQ(path.run_bounds(1), std::make_pair(std::size_t{1}, std::size_t{2}));
}

namespace
{

Path make_arc(double radius, double sweep, double spacing, bool counter_clockwise)
{
  const double sign = counter_clockwise ? 1.0 : -1.0;
  const auto count = static_cast<std::size_t>(std::round(radius * sweep / spacing)) + 1;
  Path path;
  for (std::size_t i = 0; i < count; ++i) {
    const double angle = sign * static_cast<double>(i) * spacing / radius;
    path.push_back(
      Pose2D{Vector2d{radius * std::sin(std::abs(angle)), sign * radius * (1.0 - std::cos(angle))},
             angle});
  }
  return path;
}

}  // namespace

TEST(PathCurvature, StraightLineIsZero)
{
  Path path;
  for (int i = 0; i < 40; ++i) {
    path.push_back(Pose2D{Vector2d{0.05 * static_cast<double>(i), 0.0}, 0.0});
  }

  for (const double window : {0.0, 0.05, 0.3, 1.0}) {
    const std::vector<double> curvature = eltanin::path_curvature(path, window);
    ASSERT_EQ(curvature.size(), path.size());
    for (std::size_t i = 0; i < curvature.size(); ++i) {
      EXPECT_NEAR(curvature[i], 0.0, kTol) << "window " << window << " index " << i;
    }
  }
}

TEST(PathCurvature, CircularArcMatchesInverseRadius)
{
  for (const double radius : {0.3, 0.5, 1.0, 2.0}) {
    const Path path = make_arc(radius, std::numbers::pi, 0.05, true);
    const std::vector<double> arc = cumulative_arc_length(path);
    const double window = 0.2;
    const std::vector<double> curvature = eltanin::path_curvature(path, window);

    std::size_t checked = 0;
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
      if (arc[i] < window || arc.back() - arc[i] < window) {
        continue;
      }
      EXPECT_NEAR(curvature[i], 1.0 / radius, 1e-9) << "radius " << radius << " index " << i;
      ++checked;
    }
    EXPECT_GT(checked, 0u);
  }
}

TEST(PathCurvature, WindowDoesNotBiasACircularArc)
{
  const double radius = 0.5;
  const Path path = make_arc(radius, std::numbers::pi, 0.05, true);
  const std::vector<double> arc = cumulative_arc_length(path);

  for (const double window : {0.0, 0.1, 0.3, 0.5}) {
    const std::vector<double> curvature = eltanin::path_curvature(path, window);
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
      if (arc[i] < window || arc.back() - arc[i] < window) {
        continue;
      }
      EXPECT_NEAR(curvature[i], 1.0 / radius, 1e-9) << "window " << window << " index " << i;
    }
  }
}

TEST(PathCurvature, TurningRightIsNegative)
{
  const Path path = make_arc(1.0, 0.5 * std::numbers::pi, 0.05, false);
  const std::vector<double> curvature = eltanin::path_curvature(path, 0.2);
  EXPECT_NEAR(curvature[path.size() / 2], -1.0, 1e-9);
}

TEST(PathCurvature, EndsAreZero)
{
  const Path path = make_arc(1.0, 0.5 * std::numbers::pi, 0.05, true);
  const std::vector<double> curvature = eltanin::path_curvature(path, 0.2);
  EXPECT_DOUBLE_EQ(curvature.front(), 0.0);
  EXPECT_DOUBLE_EQ(curvature.back(), 0.0);
}

TEST(PathCurvature, ShortPathsAreZero)
{
  EXPECT_TRUE(eltanin::path_curvature(Path{}, 0.3).empty());

  const Path single{Pose2D{Vector2d{0.0, 0.0}, 0.0}};
  EXPECT_EQ(eltanin::path_curvature(single, 0.3), std::vector<double>{0.0});

  const Path pair{Pose2D{Vector2d{0.0, 0.0}, 0.0}, Pose2D{Vector2d{1.0, 0.0}, 0.0}};
  EXPECT_EQ(eltanin::path_curvature(pair, 0.3), (std::vector<double>{0.0, 0.0}));
}

TEST(PathCurvature, DuplicatePosesStayFinite)
{
  const Path path{
    Pose2D{Vector2d{0.0, 0.0}, 0.0}, Pose2D{Vector2d{0.0, 0.0}, 0.0},
    Pose2D{Vector2d{0.0, 0.0}, 0.5}, Pose2D{Vector2d{1.0, 0.0}, 0.0}};

  const std::vector<double> curvature = eltanin::path_curvature(path, 0.0);
  ASSERT_EQ(curvature.size(), 4u);
  for (const double value : curvature) {
    EXPECT_DOUBLE_EQ(value, 0.0);
  }
}

TEST(PathCurvature, WindowSuppressesAnIsolatedKink)
{
  const double bend = 7.0 * std::numbers::pi / 180.0;
  Path path;
  for (int i = -10; i <= 0; ++i) {
    path.push_back(Pose2D{Vector2d{0.05 * static_cast<double>(i), 0.0}, 0.0});
  }
  for (int i = 1; i <= 10; ++i) {
    const double distance = 0.05 * static_cast<double>(i);
    path.push_back(
      Pose2D{Vector2d{distance * std::cos(bend), distance * std::sin(bend)}, bend});
  }

  const std::vector<double> narrow = eltanin::path_curvature(path, 0.0);
  const std::vector<double> wide = eltanin::path_curvature(path, 0.28);
  const double per_point = 2.0 * std::sin(0.5 * bend) / 0.05;

  EXPECT_NEAR(narrow[10], per_point, 1e-9);
  EXPECT_NEAR(wide[10], 2.0 * std::sin(0.5 * bend) / 0.30, 1e-9);
  EXPECT_LT(wide[10], 0.2 * per_point);
}
