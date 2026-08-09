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

#include <eltanin/core/footprint.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace
{

using eltanin::circumscribed_radius;
using eltanin::DistanceTraversabilityModel;
using eltanin::inscribed_radius;
using eltanin::Polygon2D;
using eltanin::Traversability;
using Eigen::Vector2d;

constexpr double kTol = 1e-12;

Polygon2D centered_square(double half)
{
  return Polygon2D{
    Vector2d{-half, -half}, Vector2d{half, -half}, Vector2d{half, half}, Vector2d{-half, half}};
}

/// Concave footprint whose arm edge has its perpendicular foot outside the segment.
Polygon2D notched_footprint()
{
  return Polygon2D{
    Vector2d{-2.0, -2.0}, Vector2d{2.0, -2.0}, Vector2d{2.0, 0.5},
    Vector2d{6.0, 0.5}, Vector2d{6.0, 3.0}, Vector2d{-2.0, 3.0}};
}

}  // namespace

static_assert(eltanin::TraversabilityModel<DistanceTraversabilityModel, double>);

TEST(Footprint, RadiiOfCenteredSquare)
{
  const Polygon2D square = centered_square(1.0);
  const auto inscribed = inscribed_radius(square);
  const auto circumscribed = circumscribed_radius(square);
  ASSERT_TRUE(inscribed.has_value());
  ASSERT_TRUE(circumscribed.has_value());
  EXPECT_NEAR(*inscribed, 1.0, kTol);
  EXPECT_NEAR(*circumscribed, std::sqrt(2.0), kTol);
  EXPECT_LE(*inscribed, *circumscribed);
}

TEST(Footprint, RadiiOfRectangle)
{
  const Polygon2D rectangle{
    Vector2d{-1.0, -0.5}, Vector2d{1.0, -0.5}, Vector2d{1.0, 0.5}, Vector2d{-1.0, 0.5}};
  const auto inscribed = inscribed_radius(rectangle);
  const auto circumscribed = circumscribed_radius(rectangle);
  ASSERT_TRUE(inscribed.has_value());
  ASSERT_TRUE(circumscribed.has_value());
  EXPECT_NEAR(*inscribed, 0.5, kTol);
  EXPECT_NEAR(*circumscribed, std::sqrt(1.25), kTol);
}

TEST(Footprint, RadiiWhenOriginIsNotTheCentroid)
{
  const Polygon2D offset{
    Vector2d{-0.2, -0.3}, Vector2d{1.8, -0.3}, Vector2d{1.8, 0.3}, Vector2d{-0.2, 0.3}};
  const auto inscribed = inscribed_radius(offset);
  const auto circumscribed = circumscribed_radius(offset);
  ASSERT_TRUE(inscribed.has_value());
  ASSERT_TRUE(circumscribed.has_value());
  EXPECT_NEAR(*inscribed, 0.2, kTol);
  EXPECT_NEAR(*circumscribed, std::hypot(1.8, 0.3), kTol);
  EXPECT_LE(*inscribed, *circumscribed);
}

TEST(Footprint, InscribedRadiusClampsToSegmentNotSupportingLine)
{
  const Polygon2D footprint = notched_footprint();
  const auto inscribed = inscribed_radius(footprint);
  ASSERT_TRUE(inscribed.has_value());
  // Without clamping the arm edge (supporting line y = 0.5) would yield 0.5.
  EXPECT_NEAR(*inscribed, 2.0, kTol);
}

TEST(Footprint, RadiiIndependentOfWinding)
{
  const Polygon2D footprint = notched_footprint();
  std::vector<Vector2d> reversed_vertices = footprint.vertices();
  std::reverse(reversed_vertices.begin(), reversed_vertices.end());
  const Polygon2D flipped(std::move(reversed_vertices));

  const auto forward = inscribed_radius(footprint);
  const auto backward = inscribed_radius(flipped);
  ASSERT_TRUE(forward.has_value());
  ASSERT_TRUE(backward.has_value());
  EXPECT_NEAR(*forward, *backward, kTol);
}

TEST(Footprint, OriginOutsidePolygonHasNoInscribedRadius)
{
  const Polygon2D away{
    Vector2d{1.0, 1.0}, Vector2d{3.0, 1.0}, Vector2d{3.0, 3.0}, Vector2d{1.0, 3.0}};
  EXPECT_FALSE(inscribed_radius(away).has_value());
  EXPECT_TRUE(circumscribed_radius(away).has_value());
}

TEST(Footprint, DegeneratePolygonHasNoRadii)
{
  EXPECT_FALSE(inscribed_radius(Polygon2D{}).has_value());
  EXPECT_FALSE(circumscribed_radius(Polygon2D{}).has_value());

  const Polygon2D two_points{Vector2d{-1.0, 0.0}, Vector2d{1.0, 0.0}};
  EXPECT_FALSE(inscribed_radius(two_points).has_value());
  EXPECT_FALSE(circumscribed_radius(two_points).has_value());

  const Polygon2D zero_area{Vector2d{-1.0, 0.0}, Vector2d{0.0, 0.0}, Vector2d{1.0, 0.0}};
  EXPECT_FALSE(inscribed_radius(zero_area).has_value());
  EXPECT_TRUE(circumscribed_radius(zero_area).has_value());
}

TEST(Footprint, DistanceModelFromFootprint)
{
  const auto distance_model = DistanceTraversabilityModel::from_footprint(centered_square(1.0), 3.0);
  ASSERT_TRUE(distance_model.has_value());
  EXPECT_NEAR(distance_model->inscribed_radius(), 1.0, kTol);
  EXPECT_NEAR(distance_model->circumscribed_radius(), std::sqrt(2.0), kTol);
  EXPECT_NEAR(distance_model->inflation_radius(), 3.0, kTol);
}

TEST(Footprint, DistanceModelFromFootprintRejectsTooSmallInflation)
{
  EXPECT_FALSE(DistanceTraversabilityModel::from_footprint(centered_square(1.0), 1.2).has_value());
  EXPECT_FALSE(DistanceTraversabilityModel::from_footprint(Polygon2D{}, 3.0).has_value());
}

TEST(Footprint, DistanceModelRejectsInconsistentOrdering)
{
  EXPECT_TRUE(DistanceTraversabilityModel::from_radii(0.3, 0.5, 1.0).has_value());
  EXPECT_TRUE(DistanceTraversabilityModel::from_radii(0.5, 0.5, 0.5).has_value());
  EXPECT_FALSE(DistanceTraversabilityModel::from_radii(0.6, 0.5, 1.0).has_value());
  EXPECT_FALSE(DistanceTraversabilityModel::from_radii(0.3, 1.2, 1.0).has_value());
  EXPECT_FALSE(DistanceTraversabilityModel::from_radii(-0.1, 0.5, 1.0).has_value());
  EXPECT_FALSE(
    DistanceTraversabilityModel::from_radii(0.3, 0.5, std::numeric_limits<double>::quiet_NaN()).has_value());
  EXPECT_FALSE(
    DistanceTraversabilityModel::from_radii(0.3, 0.5, std::numeric_limits<double>::infinity()).has_value());
}

TEST(Footprint, DistanceModelBoundaries)
{
  const auto distance_model = DistanceTraversabilityModel::from_radii(0.3, 0.5, 1.0);
  ASSERT_TRUE(distance_model.has_value());

  EXPECT_EQ(distance_model->classify(0.0), Traversability::Inscribed);
  EXPECT_EQ(distance_model->classify(0.29), Traversability::Inscribed);
  EXPECT_EQ(distance_model->classify(0.3), Traversability::Circumscribed);
  EXPECT_EQ(distance_model->classify(0.49), Traversability::Circumscribed);
  EXPECT_EQ(distance_model->classify(0.5), Traversability::Free);
  EXPECT_EQ(distance_model->classify(10.0), Traversability::Free);
}
