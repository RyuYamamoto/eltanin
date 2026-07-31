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

#include <eltanin/map/layers/raytrace_layer.hpp>

#include <map/costmap_fixture.hpp>

#include <gtest/gtest.h>

#include <initializer_list>
#include <limits>
#include <string_view>
#include <vector>

namespace
{

using Eigen::Vector2d;
using eltanin::map::Costmap;
using eltanin::map::RaytraceLayer;
using eltanin_test::cells_top_down;
using eltanin_test::make_costmap;

constexpr double kResolution = 1.0;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

/// Cell centres sit on the half-integers, so a ray between them is easy to read off the rows.
Costmap unknown_map()
{
  return make_costmap(
    {"?????",
     "?????",
     "?????"},
    kResolution);
}

void expect_cells(const Costmap & master, std::initializer_list<std::string_view> rows)
{
  EXPECT_EQ(cells_top_down(master), cells_top_down(make_costmap(rows, kResolution)));
}

}  // namespace

TEST(RaytraceLayer, DoesNothingBeforeAnyObservationIsSupplied)
{
  RaytraceLayer layer;
  Costmap master = unknown_map();

  layer.update_costs(master);

  expect_cells(
    master,
    {"?????",
     "?????",
     "?????"});
}

TEST(RaytraceLayer, ClearsTheCellsAlongTheBeamButNotItsEndpoint)
{
  const std::vector<Vector2d> endpoints = {Vector2d{4.5, 0.5}};
  RaytraceLayer layer;
  layer.set_observation(Vector2d{0.5, 0.5}, {}, endpoints);
  Costmap master = unknown_map();

  layer.update_costs(master);

  expect_cells(
    master,
    {"?????",
     "?????",
     "....?"});
}

TEST(RaytraceLayer, ClearsUnknownCellsOnADiagonalBeam)
{
  const std::vector<Vector2d> endpoints = {Vector2d{2.5, 2.5}};
  RaytraceLayer layer;
  layer.set_observation(Vector2d{0.5, 0.5}, {}, endpoints);
  Costmap master = unknown_map();

  layer.update_costs(master);

  expect_cells(
    master,
    {"?????",
     "?.???",
     ".????"});
}

TEST(RaytraceLayer, MarksTheEndpointOverUnknown)
{
  const std::vector<Vector2d> marks = {Vector2d{4.5, 0.5}};
  const std::vector<Vector2d> endpoints = {Vector2d{4.5, 0.5}};
  RaytraceLayer layer;
  layer.set_observation(Vector2d{0.5, 0.5}, marks, endpoints);
  Costmap master = unknown_map();

  layer.update_costs(master);

  expect_cells(
    master,
    {"?????",
     "?????",
     "....#"});
}

TEST(RaytraceLayer, MarkingWinsOverAClearingRayOnTheSameCell)
{
  const std::vector<Vector2d> marks = {Vector2d{2.5, 0.5}};
  const std::vector<Vector2d> endpoints = {Vector2d{4.5, 0.5}};
  RaytraceLayer layer;
  layer.set_observation(Vector2d{0.5, 0.5}, marks, endpoints);
  Costmap master = unknown_map();

  layer.update_costs(master);

  expect_cells(
    master,
    {"?????",
     "?????",
     "..#.?"});
}

TEST(RaytraceLayer, KeepsALethalCellTheEarlierLayersWroteByDefault)
{
  const std::vector<Vector2d> endpoints = {Vector2d{4.5, 0.5}};
  RaytraceLayer layer;
  layer.set_observation(Vector2d{0.5, 0.5}, {}, endpoints);
  Costmap master = make_costmap(
    {"?????",
     "?????",
     "??#??"},
    kResolution);

  layer.update_costs(master);

  expect_cells(
    master,
    {"?????",
     "?????",
     "..#.?"});
}

TEST(RaytraceLayer, ClearsALethalCellWhenAskedTo)
{
  const std::vector<Vector2d> endpoints = {Vector2d{4.5, 0.5}};
  RaytraceLayer layer(true);
  layer.set_observation(Vector2d{0.5, 0.5}, {}, endpoints);
  Costmap master = make_costmap(
    {"?????",
     "?????",
     "??#??"},
    kResolution);

  layer.update_costs(master);

  expect_cells(
    master,
    {"?????",
     "?????",
     "....?"});
}

TEST(RaytraceLayer, MarkingStillWinsWhenClearingOwnsTheObstacles)
{
  const std::vector<Vector2d> marks = {Vector2d{2.5, 0.5}};
  const std::vector<Vector2d> endpoints = {Vector2d{4.5, 0.5}};
  RaytraceLayer layer(true);
  layer.set_observation(Vector2d{0.5, 0.5}, marks, endpoints);
  Costmap master = make_costmap(
    {"?????",
     "?????",
     "??#??"},
    kResolution);

  layer.update_costs(master);

  expect_cells(
    master,
    {"?????",
     "?????",
     "..#.?"});
}

TEST(RaytraceLayer, ClearingDoesNotStopAtAProtectedCell)
{
  const std::vector<Vector2d> endpoints = {Vector2d{4.5, 0.5}};
  RaytraceLayer layer;
  layer.set_observation(Vector2d{0.5, 0.5}, {}, endpoints);
  Costmap master = make_costmap(
    {"?????",
     "?????",
     "?#???"},
    kResolution);

  layer.update_costs(master);

  expect_cells(
    master,
    {"?????",
     "?????",
     ".#..?"});
}

TEST(RaytraceLayer, ClipsARayThatLeavesTheMap)
{
  const std::vector<Vector2d> endpoints = {Vector2d{20.0, 0.5}};
  RaytraceLayer layer;
  layer.set_observation(Vector2d{0.5, 0.5}, {}, endpoints);
  Costmap master = unknown_map();

  layer.update_costs(master);

  expect_cells(
    master,
    {"?????",
     "?????",
     "....?"});
}

TEST(RaytraceLayer, ClipsARayThatStartsOutsideTheMap)
{
  const std::vector<Vector2d> endpoints = {Vector2d{2.5, 0.5}};
  RaytraceLayer layer;
  layer.set_observation(Vector2d{-10.0, 0.5}, {}, endpoints);
  Costmap master = unknown_map();

  layer.update_costs(master);

  expect_cells(
    master,
    {"?????",
     "?????",
     "..???"});
}

TEST(RaytraceLayer, RaysThatMissTheMapEntirelyChangeNothing)
{
  const std::vector<Vector2d> endpoints = {
    Vector2d{-4.0, -4.0}, Vector2d{20.0, 20.0}, Vector2d{kNan, 1.5}};
  RaytraceLayer layer;
  layer.set_observation(Vector2d{-2.0, 20.0}, {}, endpoints);
  Costmap master = unknown_map();

  layer.update_costs(master);

  expect_cells(
    master,
    {"?????",
     "?????",
     "?????"});
}

TEST(RaytraceLayer, MarkingPointsOutsideTheMapAreDiscarded)
{
  const std::vector<Vector2d> marks = {Vector2d{-0.5, 0.5}, Vector2d{20.0, 0.5}};
  RaytraceLayer layer;
  layer.set_observation(Vector2d{0.5, 0.5}, marks, {});
  Costmap master = unknown_map();

  layer.update_costs(master);

  expect_cells(
    master,
    {"?????",
     "?????",
     "?????"});
}

TEST(RaytraceLayer, ReplacesThePreviousObservationOnEveryCall)
{
  const std::vector<Vector2d> first = {Vector2d{4.5, 2.5}};
  const std::vector<Vector2d> second = {Vector2d{4.5, 0.5}};
  RaytraceLayer layer;
  Costmap master = unknown_map();

  layer.set_observation(Vector2d{0.5, 2.5}, first, first);
  layer.set_observation(Vector2d{0.5, 0.5}, second, second);
  layer.update_costs(master);

  expect_cells(
    master,
    {"?????",
     "?????",
     "....#"});
}

TEST(RaytraceLayer, ClearObservationDropsTheStaleScan)
{
  const std::vector<Vector2d> points = {Vector2d{4.5, 0.5}};
  RaytraceLayer layer;
  layer.set_observation(Vector2d{0.5, 0.5}, points, points);
  Costmap master = unknown_map();

  layer.clear_observation();
  layer.update_costs(master);

  expect_cells(
    master,
    {"?????",
     "?????",
     "?????"});
}

TEST(RaytraceLayer, KeepsItsOwnCopyOfTheObservation)
{
  RaytraceLayer layer;
  {
    const std::vector<Vector2d> marks = {Vector2d{4.5, 0.5}};
    const std::vector<Vector2d> endpoints = {Vector2d{4.5, 0.5}};
    layer.set_observation(Vector2d{0.5, 0.5}, marks, endpoints);
  }
  Costmap master = unknown_map();

  layer.update_costs(master);

  expect_cells(
    master,
    {"?????",
     "?????",
     "....#"});
}

TEST(RaytraceLayer, ZeroLengthRayClearsNothing)
{
  const std::vector<Vector2d> endpoints = {Vector2d{0.5, 0.5}};
  RaytraceLayer layer;
  layer.set_observation(Vector2d{0.5, 0.5}, {}, endpoints);
  Costmap master = unknown_map();

  layer.update_costs(master);

  expect_cells(
    master,
    {"?????",
     "?????",
     "?????"});
}

TEST(RaytraceLayer, LeavesTheGeometryUntouched)
{
  const std::vector<Vector2d> points = {Vector2d{4.5, 0.5}};
  RaytraceLayer layer;
  layer.set_observation(Vector2d{0.5, 0.5}, points, points);
  Costmap master = unknown_map();
  const eltanin::map::MapGeometry before = master.geometry();

  layer.update_costs(master);

  EXPECT_EQ(master.geometry(), before);
}

TEST(RaytraceLayer, AnEmptyMasterIsLeftAlone)
{
  const std::vector<Vector2d> points = {Vector2d{4.5, 0.5}};
  RaytraceLayer layer;
  layer.set_observation(Vector2d{0.5, 0.5}, points, points);
  Costmap master;

  layer.update_costs(master);

  EXPECT_EQ(master.cell_count(), 0u);
}
