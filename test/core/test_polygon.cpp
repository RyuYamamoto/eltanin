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

#include <eltanin/core/polygon.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <numbers>
#include <utility>
#include <vector>

namespace
{

using eltanin::contains;
using eltanin::Polygon2D;
using eltanin::Pose2D;
using eltanin::signed_area;
using eltanin::Transform2D;
using Eigen::Vector2d;

constexpr double kPi = std::numbers::pi;
constexpr double kTol = 1e-12;

Polygon2D unit_square_ccw()
{
  return Polygon2D{Vector2d{0.0, 0.0}, Vector2d{1.0, 0.0}, Vector2d{1.0, 1.0}, Vector2d{0.0, 1.0}};
}

Polygon2D reversed(const Polygon2D & polygon)
{
  std::vector<Vector2d> vertices = polygon.vertices();
  std::reverse(vertices.begin(), vertices.end());
  return Polygon2D(std::move(vertices));
}

/// L-shaped concave polygon, counter-clockwise.
Polygon2D l_shape()
{
  return Polygon2D{
    Vector2d{0.0, 0.0}, Vector2d{2.0, 0.0}, Vector2d{2.0, 1.0},
    Vector2d{1.0, 1.0}, Vector2d{1.0, 2.0}, Vector2d{0.0, 2.0}};
}

}  // namespace

TEST(Polygon, DegeneratePolygonNeverContains)
{
  EXPECT_FALSE(contains(Polygon2D{}, Vector2d{0.0, 0.0}));
  EXPECT_FALSE(contains(Polygon2D{Vector2d{0.0, 0.0}}, Vector2d{0.0, 0.0}));
  EXPECT_FALSE(contains(Polygon2D{Vector2d{0.0, 0.0}, Vector2d{1.0, 0.0}}, Vector2d{0.5, 0.0}));
}

TEST(Polygon, ConvexContainment)
{
  const Polygon2D square = unit_square_ccw();
  EXPECT_TRUE(contains(square, Vector2d{0.5, 0.5}));
  EXPECT_FALSE(contains(square, Vector2d{1.5, 0.5}));
  EXPECT_FALSE(contains(square, Vector2d{-0.1, 0.5}));
  EXPECT_FALSE(contains(square, Vector2d{0.5, 1.0001}));
}

TEST(Polygon, BoundaryAndVerticesAreInside)
{
  const Polygon2D square = unit_square_ccw();
  EXPECT_TRUE(contains(square, Vector2d{0.0, 0.5}));
  EXPECT_TRUE(contains(square, Vector2d{0.5, 0.0}));
  EXPECT_TRUE(contains(square, Vector2d{1.0, 0.5}));
  EXPECT_TRUE(contains(square, Vector2d{0.5, 1.0}));
  EXPECT_TRUE(contains(square, Vector2d{0.0, 0.0}));
  EXPECT_TRUE(contains(square, Vector2d{1.0, 1.0}));
}

TEST(Polygon, ConcaveContainment)
{
  const Polygon2D shape = l_shape();
  EXPECT_TRUE(contains(shape, Vector2d{0.5, 0.5}));
  EXPECT_TRUE(contains(shape, Vector2d{1.5, 0.5}));
  EXPECT_TRUE(contains(shape, Vector2d{0.5, 1.5}));
  EXPECT_FALSE(contains(shape, Vector2d{1.5, 1.5}));
  EXPECT_TRUE(contains(shape, Vector2d{1.0, 1.0}));
}

TEST(Polygon, ContainmentIsIndependentOfWinding)
{
  const Polygon2D shapes[] = {unit_square_ccw(), l_shape()};
  for (const Polygon2D & shape : shapes) {
    const Polygon2D flipped = reversed(shape);
    for (int i = -5; i <= 25; ++i) {
      for (int j = -5; j <= 25; ++j) {
        const Vector2d p{0.1 * static_cast<double>(i), 0.1 * static_cast<double>(j)};
        EXPECT_EQ(contains(shape, p), contains(flipped, p)) << "p=" << p.transpose();
      }
    }
  }
}

TEST(Polygon, HorizontalRayThroughVerticesIsNotDoubleCounted)
{
  const Polygon2D diamond{
    Vector2d{-1.0, 0.0}, Vector2d{0.0, -1.0}, Vector2d{1.0, 0.0}, Vector2d{0.0, 1.0}};
  EXPECT_TRUE(contains(diamond, Vector2d{0.0, 0.0}));
  EXPECT_FALSE(contains(diamond, Vector2d{2.0, 0.0}));
  EXPECT_FALSE(contains(diamond, Vector2d{-2.0, 0.0}));
  EXPECT_TRUE(contains(diamond, Vector2d{-1.0, 0.0}));
  EXPECT_TRUE(contains(diamond, Vector2d{1.0, 0.0}));
}

TEST(Polygon, SignedAreaSignFollowsWinding)
{
  const Polygon2D square = unit_square_ccw();
  EXPECT_NEAR(signed_area(square), 1.0, kTol);
  EXPECT_NEAR(signed_area(reversed(square)), -1.0, kTol);
  EXPECT_NEAR(signed_area(l_shape()), 3.0, kTol);
}

TEST(Polygon, SignedAreaOfDegenerateIsZero)
{
  EXPECT_NEAR(signed_area(Polygon2D{}), 0.0, kTol);
  EXPECT_NEAR(signed_area(Polygon2D{Vector2d{0.0, 0.0}, Vector2d{1.0, 1.0}}), 0.0, kTol);
  const Polygon2D collinear{Vector2d{0.0, 0.0}, Vector2d{1.0, 0.0}, Vector2d{2.0, 0.0}};
  EXPECT_NEAR(signed_area(collinear), 0.0, kTol);
}

TEST(Polygon, TransformByIdentityIsUnchanged)
{
  const Polygon2D square = unit_square_ccw();
  const Polygon2D result = eltanin::transform(square, Transform2D{});
  ASSERT_EQ(result.size(), square.size());
  for (std::size_t i = 0; i < result.size(); ++i) {
    EXPECT_NEAR((result[i] - square[i]).norm(), 0.0, kTol);
  }
}

TEST(Polygon, TransformMovesAndRotates)
{
  const Polygon2D footprint{
    Vector2d{-0.5, -0.3}, Vector2d{0.5, -0.3}, Vector2d{0.5, 0.3}, Vector2d{-0.5, 0.3}};
  const Pose2D pose{Vector2d{2.0, 1.0}, 0.5 * kPi};
  const Polygon2D world = eltanin::transform(footprint, pose);

  ASSERT_EQ(world.size(), 4u);
  EXPECT_NEAR(world[0].x(), 2.3, 1e-12);
  EXPECT_NEAR(world[0].y(), 0.5, 1e-12);
  EXPECT_NEAR(signed_area(world), signed_area(footprint), 1e-12);
}

TEST(Polygon, TransformByPoseAndTransformAgree)
{
  const Polygon2D square = unit_square_ccw();
  const Pose2D pose{Vector2d{1.0, -2.0}, 0.7};
  const Polygon2D from_pose = eltanin::transform(square, pose);
  const Polygon2D from_tf = eltanin::transform(square, Transform2D::from_pose(pose));
  ASSERT_EQ(from_pose.size(), from_tf.size());
  for (std::size_t i = 0; i < from_pose.size(); ++i) {
    EXPECT_NEAR((from_pose[i] - from_tf[i]).norm(), 0.0, kTol);
  }
}

TEST(Polygon, WindingReportsVertexOrder)
{
  EXPECT_EQ(eltanin::winding(unit_square_ccw()), eltanin::Winding::CounterClockwise);
  EXPECT_EQ(eltanin::winding(reversed(unit_square_ccw())), eltanin::Winding::Clockwise);
  EXPECT_EQ(eltanin::winding(l_shape()), eltanin::Winding::CounterClockwise);
}

TEST(Polygon, WindingReportsDegenerateShapes)
{
  EXPECT_EQ(eltanin::winding(Polygon2D{}), eltanin::Winding::Degenerate);
  EXPECT_EQ(
    eltanin::winding(Polygon2D{Vector2d{0.0, 0.0}, Vector2d{1.0, 1.0}}),
    eltanin::Winding::Degenerate);
  const Polygon2D collinear{Vector2d{0.0, 0.0}, Vector2d{1.0, 0.0}, Vector2d{2.0, 0.0}};
  EXPECT_EQ(eltanin::winding(collinear), eltanin::Winding::Degenerate);
}

TEST(Polygon, WindingHonoursTheAreaTolerance)
{
  const Polygon2D sliver{Vector2d{0.0, 0.0}, Vector2d{1.0, 0.0}, Vector2d{1.0, 1e-9}};
  EXPECT_EQ(eltanin::winding(sliver), eltanin::Winding::CounterClockwise);
  EXPECT_EQ(eltanin::winding(sliver, 1e-6), eltanin::Winding::Degenerate);
}

TEST(Polygon, ToCounterClockwiseNormalizesTheOrder)
{
  const Polygon2D clockwise = reversed(unit_square_ccw());
  const Polygon2D normalized = eltanin::to_counter_clockwise(clockwise);

  ASSERT_EQ(normalized.size(), clockwise.size());
  EXPECT_GT(signed_area(normalized), 0.0);
  EXPECT_NEAR(signed_area(normalized), -signed_area(clockwise), kTol);
}

TEST(Polygon, ToCounterClockwiseIsIdentityForCounterClockwiseInput)
{
  const Polygon2D square = unit_square_ccw();
  const Polygon2D normalized = eltanin::to_counter_clockwise(square);
  ASSERT_EQ(normalized.size(), square.size());
  for (std::size_t i = 0; i < normalized.size(); ++i) {
    EXPECT_NEAR((normalized[i] - square[i]).norm(), 0.0, kTol);
  }
}

TEST(Polygon, ToCounterClockwiseIsIdempotent)
{
  const Polygon2D once = eltanin::to_counter_clockwise(reversed(unit_square_ccw()));
  const Polygon2D twice = eltanin::to_counter_clockwise(once);
  ASSERT_EQ(twice.size(), once.size());
  for (std::size_t i = 0; i < twice.size(); ++i) {
    EXPECT_NEAR((twice[i] - once[i]).norm(), 0.0, kTol);
  }
}

TEST(Polygon, ToCounterClockwiseLeavesDegenerateInputAlone)
{
  const Polygon2D collinear{Vector2d{0.0, 0.0}, Vector2d{1.0, 0.0}, Vector2d{2.0, 0.0}};
  const Polygon2D normalized = eltanin::to_counter_clockwise(collinear);
  ASSERT_EQ(normalized.size(), collinear.size());
  for (std::size_t i = 0; i < normalized.size(); ++i) {
    EXPECT_NEAR((normalized[i] - collinear[i]).norm(), 0.0, kTol);
  }
}

TEST(Polygon, IsConvexIsIndependentOfWinding)
{
  EXPECT_TRUE(eltanin::is_convex(unit_square_ccw()));
  EXPECT_TRUE(eltanin::is_convex(reversed(unit_square_ccw())));
}

TEST(Polygon, IsConvexRejectsAConcavePolygon)
{
  EXPECT_FALSE(eltanin::is_convex(l_shape()));
  EXPECT_FALSE(eltanin::is_convex(reversed(l_shape())));
}

TEST(Polygon, IsConvexAcceptsCollinearVertices)
{
  const Polygon2D square_with_midpoint{
    Vector2d{0.0, 0.0}, Vector2d{0.5, 0.0}, Vector2d{1.0, 0.0}, Vector2d{1.0, 1.0},
    Vector2d{0.0, 1.0}};
  EXPECT_TRUE(eltanin::is_convex(square_with_midpoint));
}

TEST(Polygon, IsConvexRejectsFewerThanThreeVertices)
{
  EXPECT_FALSE(eltanin::is_convex(Polygon2D{}));
  EXPECT_FALSE(eltanin::is_convex(Polygon2D{Vector2d{0.0, 0.0}, Vector2d{1.0, 0.0}}));
}

TEST(Polygon, IsConvexIsInvariantUnderRigidTransform)
{
  const Pose2D pose{Vector2d{3.0, -7.0}, 1.1};
  EXPECT_TRUE(eltanin::is_convex(eltanin::transform(unit_square_ccw(), pose)));
  EXPECT_FALSE(eltanin::is_convex(eltanin::transform(l_shape(), pose)));
}

TEST(Polygon, IsConvexIsScaleInvariant)
{
  const Polygon2D tiny{
    Vector2d{0.0, 0.0}, Vector2d{1e-4, 0.0}, Vector2d{1e-4, 1e-4}, Vector2d{0.0, 1e-4}};
  EXPECT_TRUE(eltanin::is_convex(tiny));
}

TEST(Polygon, BoundingBoxCoversEveryVertex)
{
  const auto [min, max] = eltanin::bounding_box(l_shape());
  EXPECT_DOUBLE_EQ(min.x(), 0.0);
  EXPECT_DOUBLE_EQ(min.y(), 0.0);
  EXPECT_DOUBLE_EQ(max.x(), 2.0);
  EXPECT_DOUBLE_EQ(max.y(), 2.0);
}

TEST(Polygon, BoundingBoxOfASingleVertexIsDegenerate)
{
  const auto [min, max] = eltanin::bounding_box(Polygon2D{Vector2d{1.0, -2.0}});
  EXPECT_DOUBLE_EQ(min.x(), 1.0);
  EXPECT_DOUBLE_EQ(min.y(), -2.0);
  EXPECT_DOUBLE_EQ(max.x(), 1.0);
  EXPECT_DOUBLE_EQ(max.y(), -2.0);
}

TEST(Polygon, ContainsIsExactOnEdgesAndVertices)
{
  const Polygon2D square{
    Vector2d{-1.0, -1.0}, Vector2d{1.0, -1.0}, Vector2d{1.0, 1.0}, Vector2d{-1.0, 1.0}};

  EXPECT_TRUE(contains(square, Vector2d{1.0, 1.0}));
  EXPECT_TRUE(contains(square, Vector2d{-1.0, -1.0}));
  EXPECT_TRUE(contains(square, Vector2d{1.0, 0.5}));
  EXPECT_TRUE(contains(square, Vector2d{0.0, -1.0}));
  EXPECT_TRUE(contains(square, Vector2d{0.0, 0.0}));
  EXPECT_FALSE(contains(square, Vector2d{1.5, 0.0}));
  EXPECT_FALSE(contains(square, Vector2d{1.0, 1.5}));
}

TEST(Polygon, ContainsAgreesForBothWindings)
{
  const Polygon2D square{
    Vector2d{-1.0, -1.0}, Vector2d{1.0, -1.0}, Vector2d{1.0, 1.0}, Vector2d{-1.0, 1.0}};
  const Polygon2D flipped = reversed(square);
  const Vector2d probes[] = {
    Vector2d{0.0, 0.0}, Vector2d{1.0, 1.0}, Vector2d{1.0, 0.5}, Vector2d{1.5, 0.0},
    Vector2d{-1.0, 0.0}};
  for (const Vector2d & probe : probes) {
    EXPECT_EQ(contains(square, probe), contains(flipped, probe)) << probe.transpose();
  }
}
