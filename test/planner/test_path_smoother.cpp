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
#include <eltanin/map/cost_values.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/planner/astar_planner.hpp>
#include <eltanin/planner/path_smoother.hpp>

#include <planner/planner_fixture.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{

using Eigen::Vector2d;
using eltanin::Path;
using eltanin::Pose2D;
using eltanin::map::Costmap;
using eltanin::map::FREE_SPACE;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::MapGeometry;
using eltanin::planner::SmootherParams;
using eltanin::planner::smooth;
using eltanin_test::CIRCUMSCRIBED_BAND_COST;
using eltanin_test::expect_all_free;
using eltanin_test::expect_same_path;
using eltanin_test::make_cost_model;
using eltanin_test::smoother_energy;
using eltanin_test::smoothness_cost;

constexpr double RESOLUTION = 0.1;

Costmap open_map(int size_x, int size_y, double resolution = RESOLUTION)
{
  return Costmap(MapGeometry(size_x, size_y, resolution, Vector2d::Zero()), FREE_SPACE);
}

Path make_path(const std::vector<Vector2d> & positions, double last_yaw = 0.0)
{
  Path path;
  for (const Vector2d & position : positions) {
    path.push_back(Pose2D{position, 0.0});
  }
  if (!path.empty()) {
    path[path.size() - 1].yaw = last_yaw;
  }
  return path;
}

/// Zigzag in y with a uniform x spacing, so only the y coordinates are ever smoothed.
Path zigzag_path(std::size_t count, double x0, double y0, double amplitude)
{
  std::vector<Vector2d> positions;
  for (std::size_t i = 0; i < count; ++i) {
    const double offset = (i % 2 == 0) ? 0.0 : amplitude;
    positions.push_back(Vector2d{x0 + 0.1 * static_cast<double>(i), y0 + offset});
  }
  return make_path(positions);
}

/// A lethal wall from the top edge down to my = 10, wrapped in a Circumscribed band.
Costmap wall_with_band_map()
{
  Costmap map = open_map(40, 40);
  for (int my = 9; my < 40; ++my) {
    for (int mx = 17; mx <= 22; ++mx) {
      map(mx, my) = CIRCUMSCRIBED_BAND_COST;
    }
  }
  for (int my = 10; my < 40; ++my) {
    for (int mx = 18; mx <= 21; ++mx) {
      map(mx, my) = LETHAL_OBSTACLE;
    }
  }
  return map;
}

}  // namespace

TEST(PathSmoother, KeepsBothEndPointsBitIdentical)
{
  const Costmap map = open_map(40, 40);
  const Path input = zigzag_path(15, 1.0, 2.0, 0.2);
  const Path output = smooth(input, map, make_cost_model(), SmootherParams{0.5, 0.3, 0.0, 10000});

  ASSERT_EQ(output.size(), input.size());
  EXPECT_EQ(output[0].position.x(), input[0].position.x());
  EXPECT_EQ(output[0].position.y(), input[0].position.y());
  const std::size_t last = input.size() - 1;
  EXPECT_EQ(output[last].position.x(), input[last].position.x());
  EXPECT_EQ(output[last].position.y(), input[last].position.y());
}

TEST(PathSmoother, RejectsInvalidParameters)
{
  const Costmap map = open_map(10, 10);
  const Path input = make_path({Vector2d{0.1, 0.1}, Vector2d{0.2, 0.2}});

  SmootherParams params;
  params.weight_data = -0.1;
  EXPECT_THROW(smooth(input, map, make_cost_model(), params), std::invalid_argument);
  params = SmootherParams{};
  params.weight_smooth = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(smooth(input, map, make_cost_model(), params), std::invalid_argument);
  params = SmootherParams{};
  params.weight_data = 1.0;
  params.weight_smooth = 0.25;
  EXPECT_THROW(smooth(input, map, make_cost_model(), params), std::invalid_argument);
  params = SmootherParams{};
  params.tolerance = -1.0;
  EXPECT_THROW(smooth(input, map, make_cost_model(), params), std::invalid_argument);
  params = SmootherParams{};
  params.max_iterations = -1;
  EXPECT_THROW(smooth(input, map, make_cost_model(), params), std::invalid_argument);
}

TEST(PathSmoother, KeepsTheTerminalYaw)
{
  const Costmap map = open_map(40, 40);
  Path input = zigzag_path(11, 1.0, 2.0, 0.2);
  input[input.size() - 1].yaw = -1.75;

  const Path output = smooth(input, map, make_cost_model());
  EXPECT_EQ(output[output.size() - 1].yaw, -1.75);
}

TEST(PathSmoother, LeavesAnEvenlySpacedCollinearPathInPlace)
{
  const Costmap map = open_map(40, 40);
  std::vector<Vector2d> positions;
  for (int i = 0; i < 12; ++i) {
    positions.push_back(Vector2d{1.0 + 0.1 * i, 2.0});
  }
  const Path input = make_path(positions);
  const Path output = smooth(input, map, make_cost_model(), SmootherParams{0.5, 0.3, 0.0, 500});

  ASSERT_EQ(output.size(), input.size());
  for (std::size_t i = 0; i < input.size(); ++i) {
    EXPECT_NEAR(output[i].position.x(), input[i].position.x(), 1e-12) << "pose " << i;
    EXPECT_NEAR(output[i].position.y(), input[i].position.y(), 1e-12) << "pose " << i;
  }
}

TEST(PathSmoother, KeepsAnUnevenlySpacedCollinearPathCollinear)
{
  const Costmap map = open_map(40, 40);
  const Path input = make_path(
    {Vector2d{1.0, 2.0}, Vector2d{1.2, 2.0}, Vector2d{1.3, 2.0}, Vector2d{1.7, 2.0},
     Vector2d{1.8, 2.0}, Vector2d{2.4, 2.0}});
  const Path output = smooth(input, map, make_cost_model(), SmootherParams{0.5, 0.3, 0.0, 500});

  for (std::size_t i = 0; i < output.size(); ++i) {
    EXPECT_NEAR(output[i].position.y(), 2.0, 1e-12) << "pose " << i;
  }
}

TEST(PathSmoother, ReducesTheSmoothnessCost)
{
  const Costmap map = open_map(40, 40);
  const Path input = zigzag_path(21, 1.0, 2.0, 0.2);
  const Path output = smooth(input, map, make_cost_model());
  EXPECT_LT(smoothness_cost(output), smoothness_cost(input));
}

TEST(PathSmoother, SmoothnessCostStaysBelowTheInputAtEveryIterationCount)
{
  const Costmap map = open_map(40, 40);
  const Path input = zigzag_path(21, 1.0, 2.0, 0.2);
  const double reference = smoothness_cost(input);

  for (int iterations = 1; iterations <= 40; ++iterations) {
    const Path output =
      smooth(input, map, make_cost_model(), SmootherParams{0.3, 0.3, 0.0, iterations});
    EXPECT_LT(smoothness_cost(output), reference) << "iterations " << iterations;
  }
}

TEST(PathSmoother, SmoothnessCostConvergesAsIterationsGrow)
{
  const Costmap map = open_map(40, 40);
  const Path input = zigzag_path(21, 1.0, 2.0, 0.2);

  double previous = smoothness_cost(
    smooth(input, map, make_cost_model(), SmootherParams{0.3, 0.3, 0.0, 9}));
  for (int iterations = 10; iterations <= 40; ++iterations) {
    const Path output =
      smooth(input, map, make_cost_model(), SmootherParams{0.3, 0.3, 0.0, iterations});
    const double cost = smoothness_cost(output);
    EXPECT_LT(std::abs(cost - previous), 1e-6) << "iterations " << iterations;
    previous = cost;
  }
}

TEST(PathSmoother, EnergyIsMonotonicallyNonIncreasing)
{
  const Costmap map = open_map(40, 40);
  const Path input = zigzag_path(21, 1.0, 2.0, 0.2);
  const std::vector<SmootherParams> settings{
    SmootherParams{0.3, 0.3, 0.0, 0}, SmootherParams{0.5, 0.3, 0.0, 0},
    SmootherParams{0.0, 0.45, 0.0, 0}};

  for (const SmootherParams & setting : settings) {
    double previous = smoother_energy(input, input, setting);
    for (int iterations = 1; iterations <= 40; ++iterations) {
      SmootherParams params = setting;
      params.max_iterations = iterations;
      const double energy = smoother_energy(smooth(input, map, make_cost_model(), params), input,
                                            params);
      EXPECT_LE(energy, previous + 1e-15) << "weight_smooth " << setting.weight_smooth
                                          << " iterations " << iterations;
      previous = energy;
    }
  }
}

TEST(PathSmoother, NeverEntersAnObstacleOrItsCircumscribedBand)
{
  const Costmap map = wall_with_band_map();
  const auto raw = eltanin::planner::plan(
    map, make_cost_model(), Pose2D{map.geometry().map_to_world(5, 25), 0.0},
    Pose2D{map.geometry().map_to_world(34, 25), 0.0});
  ASSERT_TRUE(raw.has_value());
  ASSERT_GT(raw->size(), 30u);

  const Path output = smooth(*raw, map, make_cost_model(), SmootherParams{0.0, 0.45, 0.0, 1000});
  EXPECT_EQ(output.size(), raw->size());
  expect_all_free(output, map, make_cost_model());
}

TEST(PathSmoother, KeepsEveryPoseInsideTheMap)
{
  const Costmap map = open_map(8, 8);
  const Path input = make_path(
    {Vector2d{0.05, 0.05}, Vector2d{0.15, 0.55}, Vector2d{0.25, 0.05}, Vector2d{0.35, 0.55},
     Vector2d{0.45, 0.05}, Vector2d{0.55, 0.55}, Vector2d{0.65, 0.05}, Vector2d{0.75, 0.05}});
  const Path output = smooth(input, map, make_cost_model(), SmootherParams{0.5, 0.3, 0.0, 5000});

  for (std::size_t i = 0; i < output.size(); ++i) {
    EXPECT_TRUE(map.geometry().world_to_map(output[i].position).has_value()) << "pose " << i;
  }
}

TEST(PathSmoother, StaysFiniteOnALongPath)
{
  const Costmap map = open_map(400, 400, 0.05);
  const Path input = zigzag_path(500, 1.0, 5.0, 0.1);
  const Path output = smooth(input, map, make_cost_model(), SmootherParams{0.5, 0.3, 0.0, 200});

  ASSERT_EQ(output.size(), 500u);
  for (std::size_t i = 0; i < output.size(); ++i) {
    EXPECT_TRUE(std::isfinite(output[i].position.x())) << "pose " << i;
    EXPECT_TRUE(std::isfinite(output[i].position.y())) << "pose " << i;
    EXPECT_TRUE(std::isfinite(output[i].yaw)) << "pose " << i;
  }
  EXPECT_EQ(output[0].position.x(), input[0].position.x());
  EXPECT_EQ(output[0].position.y(), input[0].position.y());
  EXPECT_EQ(output[499].position.x(), input[499].position.x());
  EXPECT_EQ(output[499].position.y(), input[499].position.y());
  EXPECT_LT(smoothness_cost(output), smoothness_cost(input));
}

TEST(PathSmoother, ShortPathsAreReturnedUnchanged)
{
  const Costmap map = open_map(40, 40);

  const Path empty;
  EXPECT_EQ(smooth(empty, map, make_cost_model()).size(), 0u);

  const Path single = make_path({Vector2d{2.0, 2.0}}, 0.9);
  const Path single_out = smooth(single, map, make_cost_model());
  ASSERT_EQ(single_out.size(), 1u);
  EXPECT_EQ(single_out[0].position.x(), 2.0);
  EXPECT_EQ(single_out[0].position.y(), 2.0);
  EXPECT_EQ(single_out[0].yaw, 0.9);

  const Path pair = make_path({Vector2d{2.0, 2.0}, Vector2d{2.0, 2.3}}, 0.9);
  const Path pair_out = smooth(pair, map, make_cost_model());
  ASSERT_EQ(pair_out.size(), 2u);
  EXPECT_EQ(pair_out[0].position.x(), 2.0);
  EXPECT_EQ(pair_out[0].position.y(), 2.0);
  EXPECT_EQ(pair_out[1].position.x(), 2.0);
  EXPECT_EQ(pair_out[1].position.y(), 2.3);
  EXPECT_NEAR(pair_out[0].yaw, std::atan2(0.3, 0.0), 1e-12);
  EXPECT_EQ(pair_out[1].yaw, 0.9);
}

TEST(PathSmoother, FreezesPosesThatAreNotTraversable)
{
  Costmap map = open_map(40, 40);
  for (int my = 15; my <= 24; ++my) {
    for (int mx = 15; mx <= 24; ++mx) {
      map(mx, my) = LETHAL_OBSTACLE;
    }
  }

  const Path input = zigzag_path(21, 1.0, 2.0, 0.1);
  const Path output = smooth(input, map, make_cost_model(), SmootherParams{0.5, 0.3, 0.0, 500});

  ASSERT_EQ(output.size(), input.size());
  std::size_t frozen = 0;
  std::size_t moved = 0;
  for (std::size_t i = 1; i + 1 < input.size(); ++i) {
    const auto index = map.geometry().world_to_map(input[i].position);
    ASSERT_TRUE(index.has_value());
    if (map(index->x, index->y) == LETHAL_OBSTACLE) {
      EXPECT_EQ(output[i].position.x(), input[i].position.x()) << "pose " << i;
      EXPECT_EQ(output[i].position.y(), input[i].position.y()) << "pose " << i;
      ++frozen;
    } else if (output[i].position.y() != input[i].position.y()) {
      ++moved;
    }
  }
  EXPECT_GT(frozen, 0u);
  EXPECT_GT(moved, 0u);
}

TEST(PathSmoother, ZeroIterationsKeepsEveryPosition)
{
  const Costmap map = open_map(40, 40);
  const Path input = zigzag_path(15, 1.0, 2.0, 0.2);
  const Path output = smooth(input, map, make_cost_model(), SmootherParams{0.5, 0.3, 1e-4, 0});

  ASSERT_EQ(output.size(), input.size());
  for (std::size_t i = 0; i < input.size(); ++i) {
    EXPECT_EQ(output[i].position.x(), input[i].position.x()) << "pose " << i;
    EXPECT_EQ(output[i].position.y(), input[i].position.y()) << "pose " << i;
  }
}

TEST(PathSmoother, IsDeterministic)
{
  const Costmap map = open_map(40, 40);
  const Path input = zigzag_path(31, 1.0, 2.0, 0.2);
  const Path first = smooth(input, map, make_cost_model());
  const Path second = smooth(input, map, make_cost_model());
  expect_same_path(first, second);
}

TEST(PathSmoother, KeepsThePoseCount)
{
  const Costmap map = open_map(40, 40);
  for (std::size_t count : {3u, 4u, 17u, 64u}) {
    const Path input = zigzag_path(count, 1.0, 2.0, 0.2);
    EXPECT_EQ(smooth(input, map, make_cost_model()).size(), count);
  }
}

TEST(PathSmoother, RecomputesYawFromTheSmoothedPositions)
{
  const Costmap map = open_map(40, 40);
  const Path input = zigzag_path(15, 1.0, 2.0, 0.2);
  const Path output = smooth(input, map, make_cost_model(), SmootherParams{0.5, 0.3, 0.0, 50});

  for (std::size_t i = 0; i + 1 < output.size(); ++i) {
    const Vector2d delta = output[i + 1].position - output[i].position;
    EXPECT_NEAR(output[i].yaw, std::atan2(delta.y(), delta.x()), 1e-12) << "pose " << i;
  }
}
