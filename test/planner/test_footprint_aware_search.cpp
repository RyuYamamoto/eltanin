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

#include <eltanin/collision/collision_checker.hpp>
#include <eltanin/core/footprint.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/core/polygon.hpp>
#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/cost_values.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/map/layers/inflation_layer.hpp>
#include <eltanin/planner/hybrid_astar_planner.hpp>

#include <gtest/gtest.h>

#include <numbers>
#include <optional>

namespace
{

using Eigen::Vector2d;
using eltanin::DistanceTraversabilityModel;
using eltanin::Path;
using eltanin::Polygon2D;
using eltanin::Pose2D;
using eltanin::map::Costmap;
using eltanin::map::CostTraversabilityModel;
using eltanin::map::FREE_SPACE;
using eltanin::map::InflationCostModel;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::MapGeometry;
using eltanin::planner::HybridAStarParams;
using eltanin::planner::plan_hybrid_astar;

constexpr double RESOLUTION = 0.025;

/// A body 0.387 m long and 0.240 m wide whose origin sits ahead of centre, as a Kachaka's does.
Polygon2D body()
{
  return Polygon2D{{0.237, 0.120}, {-0.150, 0.120}, {-0.150, -0.120}, {0.237, -0.120}};
}

struct Scene
{
  Costmap raw;
  Costmap inflated;
  CostTraversabilityModel model;
};

/// A wall pierced by a corridor `cells` wide, both before and after inflation.
Scene corridor(int cells)
{
  const auto distance_model = DistanceTraversabilityModel::from_footprint(body(), 0.55);
  const auto inflation = InflationCostModel::create(*distance_model, 10.0);
  Costmap raw(MapGeometry(200, 80, RESOLUTION, Vector2d::Zero()), FREE_SPACE);
  const int low = 40 - cells / 2;
  for (int mx = 60; mx <= 140; ++mx) {
    for (int my = 0; my < 80; ++my) {
      if (my >= low && my < low + cells) {
        continue;
      }
      raw(mx, my) = LETHAL_OBSTACLE;
    }
  }
  Costmap inflated = raw;
  eltanin::map::InflationLayer(*inflation, false).update_costs(inflated);
  return Scene{std::move(raw), std::move(inflated),
               CostTraversabilityModel(inflation->circumscribed_cost(), false)};
}

std::size_t footprint_collisions(const Path & path, const Scene & scene)
{
  std::size_t hits = 0;
  for (const Pose2D & pose : path) {
    if (
      eltanin::collision::check_footprint_exact(scene.raw, scene.model, body(), pose) ==
      eltanin::collision::CollisionCheck::Collision) {
      ++hits;
    }
  }
  return hits;
}

}  // namespace

TEST(FootprintAwareSearch, ACorridorNarrowerThanTheCircumscribedCircleIsStillDriven)
{
  // 0.30 m of corridor for a 0.24 m body: passable, but under the 0.53 m the Free cells need.
  const Scene scene = corridor(12);
  const Pose2D start{scene.inflated.geometry().map_to_world(20, 40), 0.0};
  const Pose2D goal{scene.inflated.geometry().map_to_world(180, 40), 0.0};

  HybridAStarParams blind;
  HybridAStarParams aware;
  aware.common.footprint = body();

  const auto without = plan_hybrid_astar(scene.inflated, scene.model, start, goal, blind);
  const auto with = plan_hybrid_astar(scene.inflated, scene.model, start, goal, aware);

  EXPECT_FALSE(without.has_value());
  ASSERT_TRUE(with.has_value()) << to_string(with.error());
  EXPECT_EQ(footprint_collisions(*with, scene), 0u);
}

TEST(FootprintAwareSearch, NeverReturnsAPathWhoseFootprintTouchesAnObstacle)
{
  HybridAStarParams aware;
  aware.common.footprint = body();

  for (const int cells : {8, 10, 12, 16, 20, 24}) {
    const Scene scene = corridor(cells);
    const Pose2D start{scene.inflated.geometry().map_to_world(20, 40), 0.0};
    const Pose2D goal{scene.inflated.geometry().map_to_world(180, 40), 0.0};

    const auto path = plan_hybrid_astar(scene.inflated, scene.model, start, goal, aware);
    if (!path.has_value()) {
      continue;
    }
    EXPECT_EQ(footprint_collisions(*path, scene), 0u) << "corridor " << cells * RESOLUTION << " m";
  }
}

TEST(FootprintAwareSearch, ACorridorNarrowerThanTheBodyStaysUnreachable)
{
  // 0.20 m of corridor for a 0.24 m body: no heading fits, so the outline must not rescue it.
  const Scene scene = corridor(8);
  const Pose2D start{scene.inflated.geometry().map_to_world(20, 40), 0.0};
  const Pose2D goal{scene.inflated.geometry().map_to_world(180, 40), 0.0};

  HybridAStarParams aware;
  aware.common.footprint = body();

  EXPECT_FALSE(plan_hybrid_astar(scene.inflated, scene.model, start, goal, aware).has_value());
}

TEST(FootprintAwareSearch, AnOpenMapPlansTheSamePathWithOrWithoutTheOutline)
{
  // Nothing is in the band, so the outline changes no decision and costs the search nothing.
  const Scene scene = corridor(24);
  const Pose2D start{scene.inflated.geometry().map_to_world(20, 40), 0.0};
  const Pose2D goal{scene.inflated.geometry().map_to_world(180, 40), 0.0};

  HybridAStarParams blind;
  HybridAStarParams aware;
  aware.common.footprint = body();

  const auto without = plan_hybrid_astar(scene.inflated, scene.model, start, goal, blind);
  const auto with = plan_hybrid_astar(scene.inflated, scene.model, start, goal, aware);

  ASSERT_TRUE(without.has_value());
  ASSERT_TRUE(with.has_value());
  ASSERT_EQ(without->size(), with->size());
  for (std::size_t i = 0; i < without->size(); ++i) {
    EXPECT_EQ((*without)[i].position.x(), (*with)[i].position.x()) << "pose " << i;
    EXPECT_EQ((*without)[i].position.y(), (*with)[i].position.y()) << "pose " << i;
  }
}

namespace
{

/// An L-shaped corridor `cells` wide: no straight analytic shot exists, so the search must work.
Scene bent_corridor(int cells)
{
  const auto distance_model = DistanceTraversabilityModel::from_footprint(body(), 0.55);
  const auto inflation = InflationCostModel::create(*distance_model, 10.0);
  Costmap raw(MapGeometry(160, 160, RESOLUTION, Vector2d::Zero()), LETHAL_OBSTACLE);
  for (int mx = 10; mx < 80 + cells; ++mx) {
    for (int my = 80; my < 80 + cells; ++my) {
      raw(mx, my) = FREE_SPACE;
    }
  }
  for (int my = 10; my < 80 + cells; ++my) {
    for (int mx = 80; mx < 80 + cells; ++mx) {
      raw(mx, my) = FREE_SPACE;
    }
  }
  Costmap inflated = raw;
  eltanin::map::InflationLayer(*inflation, false).update_costs(inflated);
  return Scene{std::move(raw), std::move(inflated),
               CostTraversabilityModel(inflation->circumscribed_cost(), false)};
}

}  // namespace

TEST(FootprintAwareSearch, TurnsACornerInACorridorNoAnalyticShotCanCross)
{
  // 0.35 m of corridor bent through a right angle: only the search can find its way round.
  const Scene scene = bent_corridor(14);
  // Both ends sit far enough in that the body, whose origin is off centre, stays inside.
  const Pose2D start{scene.inflated.geometry().map_to_world(22, 86), 0.0};
  const Pose2D goal{scene.inflated.geometry().map_to_world(86, 22), -std::numbers::pi / 2.0};

  HybridAStarParams aware;
  aware.common.footprint = body();
  aware.motion_model.minimum_turning_radius = 0.15;

  const auto path = plan_hybrid_astar(scene.inflated, scene.model, start, goal, aware);

  ASSERT_TRUE(path.has_value()) << to_string(path.error());
  EXPECT_EQ(footprint_collisions(*path, scene), 0u);
  // Going round the corner means the path is far longer than the straight-line distance.
  EXPECT_GT(eltanin::path_length(*path), 2.0);
}
