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
#include <eltanin/map/layers/static_layer.hpp>

#include <map/costmap_fixture.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <utility>
#include <vector>

namespace
{

using Eigen::Vector2d;
using eltanin::map::Costmap;
using eltanin::map::FREE_SPACE;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::MapGeometry;
using eltanin::map::NO_INFORMATION;
using eltanin::map::StaticLayer;
using eltanin_test::cells_top_down;
using eltanin_test::make_costmap;

constexpr std::uint8_t L = LETHAL_OBSTACLE;
constexpr std::uint8_t U = NO_INFORMATION;
constexpr std::uint8_t F = FREE_SPACE;

Costmap source_map()
{
  return make_costmap({"#..", "..#"}, 0.1, Vector2d{0.0, 0.0});
}

}  // namespace

TEST(StaticLayer, CopiesEveryCellWhenGeometriesMatch)
{
  StaticLayer layer(source_map());
  Costmap master(source_map().geometry(), U);

  layer.update_costs(master);

  const std::vector<std::uint8_t> expected = {L, F, F, F, F, L};
  EXPECT_EQ(cells_top_down(master), expected);
}

TEST(StaticLayer, KeepsTheMasterGeometryWhenTheSourceDiffers)
{
  StaticLayer layer(source_map());
  const MapGeometry master_geometry(5, 4, 0.1, Vector2d{-0.1, -0.1});
  Costmap master(master_geometry, U);

  layer.update_costs(master);

  EXPECT_EQ(master.geometry(), master_geometry);
  EXPECT_EQ(master.cell_count(), 20u);
}

TEST(StaticLayer, SamplesTheOverlapAndLeavesTheRestUntouched)
{
  StaticLayer layer(source_map());
  Costmap master(MapGeometry(5, 4, 0.1, Vector2d{-0.1, -0.1}), U);

  layer.update_costs(master);

  const std::vector<std::uint8_t> expected = {
    U, U, U, U, U,
    U, L, F, F, U,
    U, F, F, L, U,
    U, U, U, U, U};
  EXPECT_EQ(cells_top_down(master), expected);
}

TEST(StaticLayer, UpsamplesWhenTheResolutionDiffers)
{
  StaticLayer layer(source_map());
  Costmap master(MapGeometry(6, 4, 0.05, Vector2d{0.0, 0.0}), U);

  layer.update_costs(master);

  const std::vector<std::uint8_t> expected = {
    L, L, F, F, F, F,
    L, L, F, F, F, F,
    F, F, F, F, L, L,
    F, F, F, F, L, L};
  EXPECT_EQ(cells_top_down(master), expected);
}

TEST(StaticLayer, KeepsTheGeometryWhenTheSourceIsIdentical)
{
  StaticLayer layer(source_map());
  const MapGeometry geometry = source_map().geometry();
  Costmap master(geometry, U);

  layer.update_costs(master);

  EXPECT_EQ(master.geometry(), geometry);
}

TEST(StaticLayer, WritesUnconditionallyWithoutMaxComposition)
{
  StaticLayer layer(source_map());
  Costmap master(source_map().geometry(), U);
  master(1, 0) = L;

  layer.update_costs(master);

  EXPECT_EQ(master(1, 0), F);
}

TEST(StaticLayer, NearestNeighbourSamplingDropsObstaclesWhenDownsampling)
{
  Costmap source(MapGeometry(4, 4, 0.05, Vector2d{0.0, 0.0}), F);
  source(1, 1) = L;
  source(2, 2) = L;
  source(3, 1) = L;
  StaticLayer layer(std::move(source));
  Costmap master(MapGeometry(2, 2, 0.1, Vector2d{0.0, 0.0}), U);

  layer.update_costs(master);

  const std::vector<std::uint8_t> expected = {F, F, L, L};
  EXPECT_EQ(cells_top_down(master), expected);
}
