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

namespace
{

using eltanin::Polygon2D;
using eltanin::Pose2D;
using eltanin::Traversability;
using eltanin::collision::check_footprint;
using eltanin::collision::CollisionCheck;
using eltanin::collision::detail::classify_first_stage;
using eltanin::collision::detail::FirstStage;
using eltanin_test::boundary_footprint;
using eltanin_test::boundary_scenario;
using eltanin_test::default_footprint;
using eltanin_test::free_scenario;
using eltanin_test::reversed_footprint;
using eltanin_test::CollisionScenario;
using eltanin_test::single_obstacle_scenario;
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

}  // namespace

TEST(FirstStage, EncodesTheTwoStagePolicy)
{
  EXPECT_EQ(classify_first_stage(Traversability::Free), FirstStage::NoCollision);
  EXPECT_EQ(classify_first_stage(Traversability::Inscribed), FirstStage::Collision);
  EXPECT_EQ(classify_first_stage(Traversability::Circumscribed), FirstStage::NeedsExactCheck);
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
