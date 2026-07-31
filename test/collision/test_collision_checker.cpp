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

#include <collision/collision_fixture.hpp>

#include <gtest/gtest.h>

#include <numbers>
#include <vector>

namespace
{

using eltanin::Polygon2D;
using eltanin::Pose2D;
using eltanin::Traversability;
using eltanin::collision::check_footprint;
using eltanin::collision::check_footprint_exact;
using eltanin::collision::CollisionCheck;
using eltanin::collision::detail::classify_first_stage;
using eltanin::collision::detail::classify_first_stage_exact;
using eltanin::collision::detail::FirstStage;
using eltanin_test::boundary_footprint;
using eltanin_test::boundary_scenario;
using eltanin_test::default_footprint;
using eltanin_test::free_scenario;
using eltanin_test::reversed_footprint;
using eltanin_test::CollisionScenario;
using eltanin_test::single_obstacle_scenario;
using eltanin_test::uninflated_scenario;
using eltanin_test::wall_scenario;
using Eigen::Vector2d;

constexpr double kPi = std::numbers::pi;

/// The eight headings A-6 requires the short-circuit to be independent of.
constexpr double EIGHT_HEADINGS[] = {
  0.0, kPi / 4.0, kPi / 2.0, 3.0 * kPi / 4.0, kPi, -3.0 * kPi / 4.0, -kPi / 2.0, -kPi / 4.0};

Pose2D pose_at_cell(int mx, int my, double resolution, double yaw)
{
  return Pose2D{
    Vector2d{
      resolution * (static_cast<double>(mx) + 0.5), resolution * (static_cast<double>(my) + 0.5)},
    yaw};
}

Traversability centre_classification(const CollisionScenario & scenario, int mx, int my)
{
  return scenario.model.classify(scenario.map(mx, my));
}

/// Sub-cell displacements in units of the resolution; 0.0 reproduces the on-centre poses.
constexpr double SUBCELL_OFFSETS[] = {-0.4, -0.2, 0.0, 0.2, 0.4};

/// Sweeps cells, sub-cell offsets and headings, returning how many collisions the exact check adds.
int count_collisions_added_by_the_exact_check(
  const CollisionScenario & scenario, const Polygon2D & footprint)
{
  const eltanin::map::MapGeometry & geometry = scenario.map.geometry();
  const double resolution = geometry.resolution();
  int added = 0;
  for (int my = 0; my < geometry.size_y(); ++my) {
    for (int mx = 0; mx < geometry.size_x(); ++mx) {
      const Vector2d centre = geometry.map_to_world(mx, my);
      for (const double offset_y : SUBCELL_OFFSETS) {
        for (const double offset_x : SUBCELL_OFFSETS) {
          for (const double yaw : EIGHT_HEADINGS) {
            const Pose2D pose{
              centre + Vector2d{resolution * offset_x, resolution * offset_y}, yaw};
            const CollisionCheck two_stage =
              check_footprint(scenario.map, scenario.model, footprint, pose);
            const CollisionCheck exact =
              check_footprint_exact(scenario.map, scenario.model, footprint, pose);
            if (two_stage == CollisionCheck::Collision) {
              EXPECT_EQ(exact, CollisionCheck::Collision) << "pose " << pose.position.transpose();
            }
            EXPECT_EQ(two_stage == CollisionCheck::OutsideMap, exact == CollisionCheck::OutsideMap)
              << "pose " << pose.position.transpose();
            if (two_stage == CollisionCheck::Free && exact == CollisionCheck::Collision) {
              ++added;
            }
          }
        }
      }
    }
  }
  return added;
}

}  // namespace

TEST(FirstStage, EncodesTheTwoStagePolicy)
{
  EXPECT_EQ(classify_first_stage(Traversability::Free), FirstStage::NoCollision);
  EXPECT_EQ(classify_first_stage(Traversability::Inscribed), FirstStage::Collision);
  EXPECT_EQ(classify_first_stage(Traversability::Circumscribed), FirstStage::NeedsExactCheck);
}

TEST(FirstStage, TheExactPolicyNeverDeniesACollision)
{
  EXPECT_EQ(classify_first_stage_exact(Traversability::Free), FirstStage::NeedsExactCheck);
  EXPECT_EQ(classify_first_stage_exact(Traversability::Inscribed), FirstStage::Collision);
  EXPECT_EQ(
    classify_first_stage_exact(Traversability::Circumscribed), FirstStage::NeedsExactCheck);
}

TEST(CollisionChecker, ReportsOutsideMapForAPoseOffTheMap)
{
  const CollisionScenario scenario = free_scenario();
  const Pose2D outside{Vector2d{5.0, 5.0}, 0.0};

  EXPECT_EQ(
    check_footprint(scenario.map, scenario.model, default_footprint(), outside),
    CollisionCheck::OutsideMap);
}

TEST(CollisionChecker, FreeClassificationShortCircuitsForEveryHeading)
{
  const CollisionScenario scenario = single_obstacle_scenario(10, 0);
  ASSERT_EQ(centre_classification(scenario, 11, 11), Traversability::Free);

  for (const double yaw : EIGHT_HEADINGS) {
    EXPECT_EQ(
      check_footprint(
        scenario.map, scenario.model, default_footprint(), pose_at_cell(11, 11, 0.05, yaw)),
      CollisionCheck::Free)
      << "yaw = " << yaw;
  }
}

TEST(CollisionChecker, InscribedClassificationCollidesForEveryHeading)
{
  const CollisionScenario scenario = single_obstacle_scenario(5, 0);
  ASSERT_EQ(centre_classification(scenario, 11, 11), Traversability::Inscribed);

  for (const double yaw : EIGHT_HEADINGS) {
    EXPECT_EQ(
      check_footprint(
        scenario.map, scenario.model, default_footprint(), pose_at_cell(11, 11, 0.05, yaw)),
      CollisionCheck::Collision)
      << "yaw = " << yaw;
  }
}

TEST(CollisionChecker, CircumscribedBandDependsOnTheHeading)
{
  const CollisionScenario scenario = single_obstacle_scenario(5, 5);
  ASSERT_EQ(centre_classification(scenario, 11, 11), Traversability::Circumscribed);

  // The obstacle centre sits at (0.25, 0.25) in the base frame, inside the 0.6 m square.
  EXPECT_EQ(
    check_footprint(
      scenario.map, scenario.model, default_footprint(), pose_at_cell(11, 11, 0.05, 0.0)),
    CollisionCheck::Collision);

  // Rotating by pi/4 puts it at (0.3536, 0) in the base frame, which is outside.
  EXPECT_EQ(
    check_footprint(
      scenario.map, scenario.model, default_footprint(), pose_at_cell(11, 11, 0.05, kPi / 4.0)),
    CollisionCheck::Free);
}

TEST(CollisionChecker, CircumscribedBandIsNotAFalsePositiveNextToAWall)
{
  const CollisionScenario scenario = eltanin_test::wall_scenario(false);
  ASSERT_EQ(centre_classification(scenario, 7, 11), Traversability::Circumscribed);

  EXPECT_EQ(
    check_footprint(
      scenario.map, scenario.model, default_footprint(), pose_at_cell(7, 11, 0.05, 0.0)),
    CollisionCheck::Free);
}

TEST(CollisionChecker, CellCentreOnAFootprintEdgeCollides)
{
  const CollisionScenario scenario = boundary_scenario(2, 0);
  ASSERT_EQ(centre_classification(scenario, 0, 0), Traversability::Circumscribed);

  // The obstacle centre (0.625, 0.125) lies exactly on the edge x = 0.625 of the footprint.
  EXPECT_EQ(
    check_footprint(
      scenario.map, scenario.model, boundary_footprint(), pose_at_cell(0, 0, 0.25, 0.0)),
    CollisionCheck::Collision);
}

TEST(CollisionChecker, CellCentreOnAFootprintVertexCollides)
{
  const CollisionScenario scenario = boundary_scenario(2, 2);
  ASSERT_EQ(centre_classification(scenario, 0, 0), Traversability::Circumscribed);

  // The obstacle centre (0.625, 0.625) lies exactly on the footprint vertex.
  EXPECT_EQ(
    check_footprint(
      scenario.map, scenario.model, boundary_footprint(), pose_at_cell(0, 0, 0.25, 0.0)),
    CollisionCheck::Collision);
}

TEST(CollisionChecker, CellCentreOutsideTheFootprintIsFree)
{
  const CollisionScenario scenario = boundary_scenario(3, 0);
  ASSERT_EQ(centre_classification(scenario, 0, 0), Traversability::Free);

  EXPECT_EQ(
    check_footprint(
      scenario.map, scenario.model, boundary_footprint(), pose_at_cell(0, 0, 0.25, 0.0)),
    CollisionCheck::Free);
}

TEST(CollisionChecker, ClampsAFootprintReachingOutsideTheMap)
{
  const CollisionScenario scenario = boundary_scenario(2, 0);
  // The footprint at cell (0, 0) spans [-0.375, 0.625], so the cell rectangle needs clamping.
  EXPECT_EQ(
    check_footprint(
      scenario.map, scenario.model, boundary_footprint(), pose_at_cell(0, 0, 0.25, 0.0)),
    CollisionCheck::Collision);
}

TEST(CollisionChecker, IsIndependentOfTheFootprintVertexOrder)
{
  const CollisionScenario circumscribed = single_obstacle_scenario(5, 5);
  const Polygon2D clockwise = reversed_footprint(default_footprint());

  for (const double yaw : EIGHT_HEADINGS) {
    const Pose2D pose = pose_at_cell(11, 11, 0.05, yaw);
    EXPECT_EQ(
      check_footprint(circumscribed.map, circumscribed.model, default_footprint(), pose),
      check_footprint(circumscribed.map, circumscribed.model, clockwise, pose))
      << "yaw = " << yaw;
  }

  const CollisionScenario boundary = boundary_scenario(2, 2);
  const Pose2D pose = pose_at_cell(0, 0, 0.25, 0.0);
  EXPECT_EQ(
    check_footprint(boundary.map, boundary.model, boundary_footprint(), pose),
    check_footprint(boundary.map, boundary.model, reversed_footprint(boundary_footprint()), pose));
}

TEST(CollisionCheckerExact, CatchesTheFootprintOverlapTheFreeGateMissesOnARawMap)
{
  const CollisionScenario scenario = uninflated_scenario(5, 0);
  const Pose2D pose = pose_at_cell(11, 11, 0.05, 0.0);
  ASSERT_EQ(centre_classification(scenario, 11, 11), Traversability::Free);

  // The lethal centre (0.825, 0.575) sits inside the 0.6 m square spanning x = [0.275, 0.875].
  EXPECT_EQ(
    check_footprint(scenario.map, scenario.model, default_footprint(), pose), CollisionCheck::Free);
  EXPECT_EQ(
    check_footprint_exact(scenario.map, scenario.model, default_footprint(), pose),
    CollisionCheck::Collision);
}

TEST(CollisionCheckerExact, ReportsOutsideMapExactlyLikeTheTwoStageCheck)
{
  const CollisionScenario scenario = uninflated_scenario(-11, 0);
  // The origin is off the map while the footprint still covers the lethal cell (0, 11).
  const Pose2D outside{Vector2d{-0.01, 0.575}, 0.0};

  EXPECT_EQ(
    check_footprint_exact(scenario.map, scenario.model, default_footprint(), outside),
    CollisionCheck::OutsideMap);
  EXPECT_EQ(
    check_footprint(scenario.map, scenario.model, default_footprint(), outside),
    check_footprint_exact(scenario.map, scenario.model, default_footprint(), outside));

  const Pose2D far_away{Vector2d{5.0, 5.0}, 0.0};
  EXPECT_EQ(
    check_footprint_exact(scenario.map, scenario.model, default_footprint(), far_away),
    CollisionCheck::OutsideMap);
}

TEST(CollisionCheckerExact, KeepsTheInscribedShortCircuit)
{
  const CollisionScenario scenario = uninflated_scenario(0, 0);
  ASSERT_EQ(centre_classification(scenario, 11, 11), Traversability::Inscribed);

  for (const double yaw : EIGHT_HEADINGS) {
    EXPECT_EQ(
      check_footprint_exact(
        scenario.map, scenario.model, default_footprint(), pose_at_cell(11, 11, 0.05, yaw)),
      CollisionCheck::Collision)
      << "yaw = " << yaw;
  }
}

TEST(CollisionCheckerExact, StillDependsOnTheHeadingInTheCircumscribedBand)
{
  const CollisionScenario scenario = single_obstacle_scenario(5, 5);
  ASSERT_EQ(centre_classification(scenario, 11, 11), Traversability::Circumscribed);

  EXPECT_EQ(
    check_footprint_exact(
      scenario.map, scenario.model, default_footprint(), pose_at_cell(11, 11, 0.05, 0.0)),
    CollisionCheck::Collision);
  EXPECT_EQ(
    check_footprint_exact(
      scenario.map, scenario.model, default_footprint(), pose_at_cell(11, 11, 0.05, kPi / 4.0)),
    CollisionCheck::Free);
}

TEST(CollisionCheckerExact, IsAConservativeSupersetOfTheTwoStageCheck)
{
  // Inflated at 0.05 m the residue band of docs/collision-design.md 2.4 is thinner than this grid.
  EXPECT_EQ(count_collisions_added_by_the_exact_check(wall_scenario(true), default_footprint()), 0);
  EXPECT_EQ(
    count_collisions_added_by_the_exact_check(single_obstacle_scenario(5, 5), default_footprint()),
    0);

  // The 0.25 m cells widen that band, and without inflation the Free gate misses outright.
  EXPECT_GT(
    count_collisions_added_by_the_exact_check(boundary_scenario(2, 2), boundary_footprint()), 0);
  EXPECT_GT(
    count_collisions_added_by_the_exact_check(uninflated_scenario(5, 0), default_footprint()), 0);
}

TEST(CellsCovering, ReturnsTheClampedRectangleOfTheFootprint)
{
  const CollisionScenario scenario = boundary_scenario(2, 0);
  const eltanin::Polygon2D world =
    eltanin::transform(boundary_footprint(), pose_at_cell(0, 0, 0.25, 0.0));

  const auto rect = eltanin::collision::cells_covering(scenario.map.geometry(), world);

  ASSERT_TRUE(rect.has_value());
  EXPECT_EQ(rect->min_x, 0);
  EXPECT_EQ(rect->min_y, 0);
  EXPECT_EQ(rect->max_x, 2);
  EXPECT_EQ(rect->max_y, 2);
}

TEST(CellsCovering, RejectsAFootprintThatMissesTheMapAndAnEmptyPolygon)
{
  const CollisionScenario scenario = free_scenario();
  const eltanin::Polygon2D far_away =
    eltanin::transform(default_footprint(), Pose2D{Vector2d{9.0, 9.0}, 0.0});

  EXPECT_FALSE(eltanin::collision::cells_covering(scenario.map.geometry(), far_away).has_value());
  EXPECT_FALSE(
    eltanin::collision::cells_covering(scenario.map.geometry(), eltanin::Polygon2D{}).has_value());
}

TEST(ContainsAny, FindsAPointInsideThePolygonIncludingTheBoundary)
{
  const eltanin::Polygon2D square = boundary_footprint();
  const std::vector<Vector2d> outside{Vector2d{2.0, 0.0}, Vector2d{0.0, -3.0}};
  const std::vector<Vector2d> one_inside{Vector2d{2.0, 0.0}, Vector2d{0.1, 0.1}};
  const std::vector<Vector2d> on_the_edge{Vector2d{0.5, 0.0}};
  const std::vector<Vector2d> on_a_vertex{Vector2d{0.5, 0.5}};

  EXPECT_FALSE(eltanin::collision::contains_any(square, outside));
  EXPECT_TRUE(eltanin::collision::contains_any(square, one_inside));
  EXPECT_TRUE(eltanin::collision::contains_any(square, on_the_edge));
  EXPECT_TRUE(eltanin::collision::contains_any(square, on_a_vertex));
  EXPECT_FALSE(eltanin::collision::contains_any(square, std::vector<Vector2d>{}));
}

TEST(FootprintHitsPoints, MovesWithThePose)
{
  const std::vector<Vector2d> points{Vector2d{1.0, 0.0}};

  EXPECT_FALSE(eltanin::collision::footprint_hits_points(
    boundary_footprint(), Pose2D{Vector2d::Zero(), 0.0}, points));
  EXPECT_TRUE(eltanin::collision::footprint_hits_points(
    boundary_footprint(), Pose2D{Vector2d{0.6, 0.0}, 0.0}, points));
}

TEST(IsCellOccupied, OnlySelectsLethalCellsInsideTheMap)
{
  const CollisionScenario scenario = single_obstacle_scenario(5, 0);

  EXPECT_TRUE(eltanin::collision::is_cell_occupied(scenario.map, scenario.model, 16, 11));
  // The cell next to it carries an inflated cost, which is not an obstacle.
  EXPECT_FALSE(eltanin::collision::is_cell_occupied(scenario.map, scenario.model, 15, 11));
  EXPECT_FALSE(eltanin::collision::is_cell_occupied(scenario.map, scenario.model, 11, 11));
  EXPECT_FALSE(eltanin::collision::is_cell_occupied(scenario.map, scenario.model, -1, 11));
  EXPECT_FALSE(eltanin::collision::is_cell_occupied(scenario.map, scenario.model, 24, 11));
}

TEST(ContainsOccupiedCell, DependsOnTheOrientationOfTheGivenPolygon)
{
  const CollisionScenario scenario = single_obstacle_scenario(5, 5);
  const eltanin::Polygon2D upright =
    eltanin::transform(default_footprint(), pose_at_cell(11, 11, 0.05, 0.0));
  const eltanin::Polygon2D rotated =
    eltanin::transform(default_footprint(), pose_at_cell(11, 11, 0.05, kPi / 4.0));

  EXPECT_TRUE(eltanin::collision::contains_occupied_cell(scenario.map, scenario.model, upright));
  EXPECT_FALSE(eltanin::collision::contains_occupied_cell(scenario.map, scenario.model, rotated));
}

TEST(ContainsOccupiedCell, ReportsFreeWhenThePolygonMissesTheMap)
{
  const CollisionScenario scenario = single_obstacle_scenario(5, 5);
  const eltanin::Polygon2D far_away =
    eltanin::transform(default_footprint(), Pose2D{Vector2d{-9.0, -9.0}, 0.0});

  EXPECT_FALSE(eltanin::collision::contains_occupied_cell(scenario.map, scenario.model, far_away));
}
