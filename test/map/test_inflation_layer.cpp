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

#include <eltanin/map/cost_values.hpp>
#include <eltanin/map/layers/inflation_layer.hpp>
#include <eltanin/map_io/pgm.hpp>

#include <map/costmap_fixture.hpp>
#include <map_io/pgm_fixture.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <vector>

namespace
{

using Eigen::Vector2d;
using eltanin::map::Costmap;
using eltanin::map::FREE_SPACE;
using eltanin::map::InflationCostModel;
using eltanin::map::InflationLayer;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::MapGeometry;
using eltanin::map::NO_INFORMATION;
using eltanin_test::cells_top_down;
using eltanin_test::make_costmap;
using eltanin_test::make_inflation_model;

/// Reference model of the fixture used by the exact-value tests; see .plait/01-plan.md §5.1.
InflationCostModel single_obstacle_model() { return make_inflation_model(0.15, 0.25, 0.35, 3.0); }

Costmap inflated(Costmap input, const InflationCostModel & model, bool inflate_unknown = false)
{
  InflationLayer layer(model, inflate_unknown);
  layer.update_costs(input);
  return input;
}

/// Independent expectation built by calling cost_at_distance() directly for every obstacle.
std::vector<std::uint8_t> expected_by_direct_computation(
  const Costmap & input, const InflationCostModel & model, bool inflate_unknown)
{
  Costmap expected = input;
  const double resolution = input.geometry().resolution();
  for (int my = 0; my < input.size_y(); ++my) {
    for (int mx = 0; mx < input.size_x(); ++mx) {
      if (input(mx, my) != LETHAL_OBSTACLE) {
        continue;
      }
      for (int y = 0; y < input.size_y(); ++y) {
        for (int x = 0; x < input.size_x(); ++x) {
          const double distance =
            std::hypot(static_cast<double>(x - mx), static_cast<double>(y - my)) * resolution;
          if (distance > model.radii().inflation_radius()) {
            continue;
          }
          const std::uint8_t cost = model.cost_at_distance(distance);
          std::uint8_t & cell = expected(x, y);
          if (cell == NO_INFORMATION) {
            if (inflate_unknown) {
              cell = cost;
            }
          } else {
            cell = std::max(cell, cost);
          }
        }
      }
    }
  }
  return cells_top_down(expected);
}

Costmap rotated_180(const Costmap & input)
{
  Costmap output(input.geometry(), FREE_SPACE);
  for (int my = 0; my < input.size_y(); ++my) {
    for (int mx = 0; mx < input.size_x(); ++mx) {
      output(input.size_x() - 1 - mx, input.size_y() - 1 - my) = input(mx, my);
    }
  }
  return output;
}

}  // namespace

TEST(InflationLayer, ProducesTheExactCostsForASingleObstacle)
{
  const Costmap result = inflated(
    make_costmap(
      {".......", ".......", ".......", "...#...", ".......", ".......", "......."}, 0.1),
    single_obstacle_model());

  const std::vector<std::uint8_t> expected = {
      0,   0, 153, 160, 153,   0,   0,
      0, 169, 202, 216, 202, 169,   0,
    153, 202, 253, 253, 253, 202, 153,
    160, 216, 253, 254, 253, 216, 160,
    153, 202, 253, 253, 253, 202, 153,
      0, 169, 202, 216, 202, 169,   0,
      0,   0, 153, 160, 153,   0,   0};
  EXPECT_EQ(cells_top_down(result), expected);
}

TEST(InflationLayer, IsInvariantUnderReflectionAndTransposition)
{
  const Costmap result = inflated(
    make_costmap(
      {".......", ".......", ".......", "...#...", ".......", ".......", "......."}, 0.1),
    single_obstacle_model());

  for (int my = 0; my < 7; ++my) {
    for (int mx = 0; mx < 7; ++mx) {
      EXPECT_EQ(result(mx, my), result(6 - mx, my)) << "x flip at " << mx << "," << my;
      EXPECT_EQ(result(mx, my), result(mx, 6 - my)) << "y flip at " << mx << "," << my;
      EXPECT_EQ(result(mx, my), result(my, mx)) << "transpose at " << mx << "," << my;
    }
  }
}

TEST(InflationLayer, CoversBothEndsOfTheWindow)
{
  const Costmap result = inflated(
    make_costmap(
      {".......", ".......", ".......", "...#...", ".......", ".......", "......."}, 0.1),
    single_obstacle_model());

  EXPECT_GT(result(0, 3), FREE_SPACE);
  EXPECT_GT(result(6, 3), FREE_SPACE);
  EXPECT_GT(result(3, 0), FREE_SPACE);
  EXPECT_GT(result(3, 6), FREE_SPACE);
}

TEST(InflationLayer, CostIsNonIncreasingWithDistance)
{
  const Costmap result = inflated(
    make_costmap(
      {".......", ".......", ".......", "...#...", ".......", ".......", "......."}, 0.1),
    single_obstacle_model());

  for (int ay = 0; ay < 7; ++ay) {
    for (int ax = 0; ax < 7; ++ax) {
      const double da = std::hypot(ax - 3.0, ay - 3.0);
      for (int by = 0; by < 7; ++by) {
        for (int bx = 0; bx < 7; ++bx) {
          const double db = std::hypot(bx - 3.0, by - 3.0);
          if (da < db) {
            EXPECT_GE(result(ax, ay), result(bx, by))
              << "at " << ax << "," << ay << " vs " << bx << "," << by;
          }
        }
      }
    }
  }
}

TEST(InflationLayer, TwoObstaclesComposeWithMax)
{
  const Costmap input = make_costmap(
    {"..........", "..........", "..#.......", "..........", ".......#..", ".........."}, 0.1);
  const InflationCostModel model = single_obstacle_model();

  EXPECT_EQ(
    cells_top_down(inflated(input, model)),
    expected_by_direct_computation(input, model, false));
}

TEST(InflationLayer, MatchesDirectCostAtDistanceOnAClutteredGrid)
{
  const Costmap input = make_costmap(
    {"#........#", "..?.......", "..#....?..", "....#.....", ".?.......#", "#........."}, 0.1);
  const InflationCostModel model = single_obstacle_model();

  EXPECT_EQ(
    cells_top_down(inflated(input, model)),
    expected_by_direct_computation(input, model, false));
  EXPECT_EQ(
    cells_top_down(inflated(input, model, true)),
    expected_by_direct_computation(input, model, true));
}

TEST(InflationLayer, RebuildsTheLookupTableWhenTheResolutionChanges)
{
  const InflationCostModel model = single_obstacle_model();
  InflationLayer layer(model);

  Costmap coarse = make_costmap({".....", ".....", "..#..", ".....", "....."}, 0.1);
  const Costmap coarse_input = coarse;
  layer.update_costs(coarse);
  EXPECT_EQ(cells_top_down(coarse), expected_by_direct_computation(coarse_input, model, false));

  Costmap fine = make_costmap({".....", ".....", "..#..", ".....", "....."}, 0.05);
  const Costmap fine_input = fine;
  layer.update_costs(fine);
  EXPECT_EQ(cells_top_down(fine), expected_by_direct_computation(fine_input, model, false));
}

TEST(InflationLayer, PreservesUnknownInBothTheInscribedAndDecayBands)
{
  const Costmap result = inflated(
    make_costmap({".......", ".....#.", "...?...", ".#?....", "......."}, 0.1),
    make_inflation_model(0.15, 0.20, 0.25, 0.0));

  EXPECT_EQ(result(2, 1), NO_INFORMATION);
  EXPECT_EQ(result(3, 2), NO_INFORMATION);
}

TEST(InflationLayer, ReplacesUnknownWhenInflateUnknownIsSet)
{
  const Costmap result = inflated(
    make_costmap({".......", ".....#.", "...?...", ".#?....", "......."}, 0.1),
    make_inflation_model(0.15, 0.20, 0.25, 0.0), true);

  EXPECT_EQ(result(2, 1), eltanin::map::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(result(3, 2), eltanin::map::MAX_NON_OBSTACLE);
}

TEST(InflationLayer, ReplacesUnknownEvenWhenTheComputedCostIsZero)
{
  const Costmap input =
    make_costmap({".......", ".......", ".......", "...#.?.", ".......", ".......", "......."}, 0.1);
  const InflationCostModel model = make_inflation_model(0.1, 0.1, 0.5, 100.0);
  ASSERT_EQ(model.cost_at_distance(0.2), FREE_SPACE);

  EXPECT_EQ(inflated(input, model)(5, 3), NO_INFORMATION);
  EXPECT_EQ(inflated(input, model, true)(5, 3), FREE_SPACE);
}

TEST(InflationLayer, DoesNotOverwriteLethalCells)
{
  const Costmap result = inflated(
    make_costmap({".....", ".....", "..##.", ".....", "....."}, 0.1), single_obstacle_model());

  EXPECT_EQ(result(2, 2), LETHAL_OBSTACLE);
  EXPECT_EQ(result(3, 2), LETHAL_OBSTACLE);
}

TEST(InflationLayer, ClampsTheWindowAtEveryBorder)
{
  const Costmap result = inflated(
    make_costmap({"#...#", ".....", "#...#", ".....", "#...#"}, 0.1), single_obstacle_model());

  EXPECT_EQ(result(0, 0), LETHAL_OBSTACLE);
  EXPECT_EQ(result(4, 0), LETHAL_OBSTACLE);
  EXPECT_EQ(result(0, 4), LETHAL_OBSTACLE);
  EXPECT_EQ(result(4, 4), LETHAL_OBSTACLE);
  EXPECT_EQ(result(1, 0), eltanin::map::INSCRIBED_INFLATED_OBSTACLE);
}

TEST(InflationLayer, IsIndependentOfTheScanOrder)
{
  const Costmap input = make_costmap(
    {".......", ".......", "...#...", ".......", ".......", "#......", "......."}, 0.1);
  const InflationCostModel model = single_obstacle_model();

  const Costmap forward = inflated(input, model);
  const Costmap reverse = inflated(rotated_180(input), model);

  EXPECT_EQ(cells_top_down(rotated_180(reverse)), cells_top_down(forward));
}

TEST(InflationLayer, NeverWritesBeyondTheInflationRadius)
{
  Costmap input = make_costmap(
    {".........", ".........", ".........", ".........", "....#....", ".........", ".........",
     ".........", "........."},
    0.1);
  constexpr std::uint8_t SENTINEL = 77;
  input(7, 7) = SENTINEL;
  input(0, 0) = SENTINEL;

  const Costmap result = inflated(input, make_inflation_model(0.05, 0.1, 0.25, 3.0));

  EXPECT_EQ(result(7, 7), SENTINEL);
  EXPECT_EQ(result(0, 0), SENTINEL);
}

TEST(InflationLayer, HandlesAZeroInflationRadius)
{
  const Costmap result = inflated(
    make_costmap({"...", ".#.", "..."}, 0.1), make_inflation_model(0.0, 0.0, 0.0, 3.0));

  const std::vector<std::uint8_t> expected = {0, 0, 0, 0, 254, 0, 0, 0, 0};
  EXPECT_EQ(cells_top_down(result), expected);
}

TEST(InflationLayer, HandlesAnInflationRadiusBelowOneCell)
{
  const Costmap result = inflated(
    make_costmap({"...", ".#.", "..."}, 0.05), make_inflation_model(0.01, 0.02, 0.04, 3.0));

  const std::vector<std::uint8_t> expected = {0, 0, 0, 0, 254, 0, 0, 0, 0};
  EXPECT_EQ(cells_top_down(result), expected);
}

TEST(InflationLayer, HandlesASingleCellMap)
{
  const Costmap result = inflated(make_costmap({"#"}, 0.1), single_obstacle_model());

  EXPECT_EQ(result(0, 0), LETHAL_OBSTACLE);
}

TEST(InflationLayer, LeavesTheGeometryUntouched)
{
  const MapGeometry geometry(7, 7, 0.1, Vector2d{-1.5, 2.5});
  Costmap master(geometry, FREE_SPACE);
  master(3, 3) = LETHAL_OBSTACLE;
  InflationLayer layer(single_obstacle_model());

  layer.update_costs(master);

  EXPECT_EQ(master.geometry(), geometry);
}

TEST(InflationLayer, WritesTheExpectedPgmBytes)
{
  const Costmap result = inflated(
    make_costmap({".......", ".....#.", "...?...", ".#?....", "......."}, 0.1),
    make_inflation_model(0.15, 0.20, 0.25, 0.0));

  const std::filesystem::path path =
    std::filesystem::path(ELTANIN_TEST_TMP_DIR) / "inflation_golden.pgm";
  eltanin::map_io::write_pgm(path, result);

  int width = 0;
  int height = 0;
  const std::vector<std::uint8_t> pixels = eltanin_test::read_raw_pgm(path, width, height);

  const std::vector<std::uint8_t> expected = {
      0,   0,   0, 252, 253, 253, 253,
    252, 252, 252, 252, 253, 254, 253,
    253, 253, 253, 255, 253, 253, 253,
    253, 254, 255, 252, 252, 252, 252,
    253, 253, 253, 252,   0,   0,   0};
  EXPECT_EQ(width, 7);
  EXPECT_EQ(height, 5);
  EXPECT_EQ(pixels, expected);
}
