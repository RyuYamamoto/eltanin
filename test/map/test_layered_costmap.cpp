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
#include <eltanin/map/layered_costmap.hpp>
#include <eltanin/map/layers/inflation_layer.hpp>
#include <eltanin/map/layers/obstacle_layer.hpp>
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
using eltanin::map::InflationCostModel;
using eltanin::map::InflationLayer;
using eltanin::map::LayeredCostmap;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::MapGeometry;
using eltanin::map::NO_INFORMATION;
using eltanin::map::ObstacleLayer;
using eltanin::map::StaticLayer;
using eltanin_test::make_costmap;
using eltanin_test::make_inflation_model;

MapGeometry local_geometry() { return MapGeometry(6, 6, 0.1, Vector2d{0.0, 0.0}); }

}  // namespace

TEST(LayeredCostmap, StartsFilledWithTheDefaultCost)
{
  const LayeredCostmap costmap(local_geometry(), NO_INFORMATION);

  for (std::size_t i = 0; i < costmap.costmap().cell_count(); ++i) {
    EXPECT_EQ(costmap.costmap()[i], NO_INFORMATION);
  }
}

TEST(LayeredCostmap, UpdateWithoutLayersResetsToTheDefaultCost)
{
  LayeredCostmap costmap(local_geometry(), FREE_SPACE);

  costmap.update();

  for (std::size_t i = 0; i < costmap.costmap().cell_count(); ++i) {
    EXPECT_EQ(costmap.costmap()[i], FREE_SPACE);
  }
}

TEST(LayeredCostmap, DropsThePreviousResultOnEveryUpdate)
{
  LayeredCostmap costmap(local_geometry(), FREE_SPACE);
  ObstacleLayer & obstacles = costmap.add_layer<ObstacleLayer>();

  const std::vector<Vector2d> points = {Vector2d{0.25, 0.25}};
  obstacles.set_points(points);
  costmap.update();
  ASSERT_EQ(costmap.costmap()(2, 2), LETHAL_OBSTACLE);

  obstacles.set_points({});
  costmap.update();
  EXPECT_EQ(costmap.costmap()(2, 2), FREE_SPACE);
}

TEST(LayeredCostmap, AppliesLayersInRegistrationOrder)
{
  const InflationCostModel model = make_inflation_model(0.15, 0.20, 0.25, 0.0);
  const std::vector<Vector2d> points = {Vector2d{0.25, 0.25}};

  LayeredCostmap inflation_last(local_geometry(), FREE_SPACE);
  inflation_last.add_layer<ObstacleLayer>().set_points(points);
  inflation_last.add_layer<InflationLayer>(model);
  inflation_last.update();

  LayeredCostmap inflation_first(local_geometry(), FREE_SPACE);
  inflation_first.add_layer<InflationLayer>(model);
  inflation_first.add_layer<ObstacleLayer>().set_points(points);
  inflation_first.update();

  EXPECT_EQ(inflation_last.costmap()(3, 2), eltanin::map::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(inflation_first.costmap()(3, 2), FREE_SPACE);
}

TEST(LayeredCostmap, StaticLayerDoesNotReplaceTheContainerGeometry)
{
  LayeredCostmap costmap(local_geometry(), NO_INFORMATION);
  costmap.add_layer<StaticLayer>(make_costmap({"###", "###", "###"}, 0.2, Vector2d{0.0, 0.0}));

  costmap.update();

  EXPECT_EQ(costmap.geometry(), local_geometry());
  EXPECT_EQ(costmap.costmap()(0, 0), LETHAL_OBSTACLE);
}

TEST(LayeredCostmap, SetOriginDoesNotReallocateTheCells)
{
  LayeredCostmap costmap(local_geometry(), FREE_SPACE);
  const std::uint8_t * const before = costmap.costmap().data().data();

  costmap.set_origin(Vector2d{-3.0, 7.5});

  EXPECT_EQ(costmap.costmap().data().data(), before);
  EXPECT_EQ(costmap.geometry().origin().x(), -3.0);
  EXPECT_EQ(costmap.geometry().origin().y(), 7.5);
  EXPECT_EQ(costmap.geometry().size_x(), 6);
  EXPECT_EQ(costmap.geometry().size_y(), 6);
  EXPECT_EQ(costmap.geometry().resolution(), 0.1);
}

TEST(LayeredCostmap, CenterOnPutsTheRobotAtTheWindowCentre)
{
  LayeredCostmap costmap(local_geometry(), FREE_SPACE);

  costmap.center_on(Vector2d{10.0, -4.0});

  EXPECT_DOUBLE_EQ(costmap.geometry().origin().x(), 10.0 - 0.3);
  EXPECT_DOUBLE_EQ(costmap.geometry().origin().y(), -4.0 - 0.3);
}

TEST(LayeredCostmap, GivesTheSameWorldFrameResultAfterTheWindowMoves)
{
  const std::vector<Vector2d> points = {Vector2d{10.05, -4.05}, Vector2d{10.15, -3.95}};

  LayeredCostmap costmap(local_geometry(), FREE_SPACE);
  ObstacleLayer & obstacles = costmap.add_layer<ObstacleLayer>();
  obstacles.set_points(points);
  costmap.center_on(Vector2d{10.0, -4.0});
  costmap.update();

  LayeredCostmap shifted(local_geometry(), FREE_SPACE);
  ObstacleLayer & shifted_obstacles = shifted.add_layer<ObstacleLayer>();
  shifted_obstacles.set_points(points);
  shifted.center_on(Vector2d{10.1, -4.1});
  shifted.update();

  for (const Vector2d & point : points) {
    const auto index = costmap.geometry().world_to_map(point);
    const auto shifted_index = shifted.geometry().world_to_map(point);
    ASSERT_TRUE(index.has_value());
    ASSERT_TRUE(shifted_index.has_value());
    EXPECT_EQ(costmap.costmap()(index->x, index->y), LETHAL_OBSTACLE);
    EXPECT_EQ(shifted.costmap()(shifted_index->x, shifted_index->y), LETHAL_OBSTACLE);
  }
}

TEST(LayeredCostmap, LayerReferencesSurviveFurtherRegistrations)
{
  LayeredCostmap costmap(local_geometry(), FREE_SPACE);
  ObstacleLayer & obstacles = costmap.add_layer<ObstacleLayer>();
  costmap.add_layer<InflationLayer>(make_inflation_model(0.15, 0.20, 0.25, 0.0));

  const std::vector<Vector2d> points = {Vector2d{0.25, 0.25}};
  obstacles.set_points(points);
  costmap.update();

  EXPECT_EQ(costmap.costmap()(2, 2), LETHAL_OBSTACLE);
}

TEST(LayeredCostmap, KeepsTheGeometryAcrossAFullUpdate)
{
  LayeredCostmap costmap(local_geometry(), NO_INFORMATION);
  costmap.add_layer<StaticLayer>(make_costmap({"...", ".#.", "..."}, 0.2, Vector2d{0.0, 0.0}));
  costmap.add_layer<ObstacleLayer>().set_points(std::vector<Vector2d>{Vector2d{0.55, 0.55}});
  costmap.add_layer<InflationLayer>(make_inflation_model(0.15, 0.20, 0.25, 0.0));

  costmap.update();

  EXPECT_EQ(costmap.geometry(), local_geometry());
}

TEST(LayeredCostmap, LayerReferencesSurviveAMove)
{
  LayeredCostmap original(local_geometry(), FREE_SPACE);
  ObstacleLayer & obstacles = original.add_layer<ObstacleLayer>();

  LayeredCostmap moved = std::move(original);
  const std::vector<Vector2d> points = {Vector2d{0.25, 0.25}};
  obstacles.set_points(points);
  moved.update();

  EXPECT_EQ(moved.costmap()(2, 2), LETHAL_OBSTACLE);
}
