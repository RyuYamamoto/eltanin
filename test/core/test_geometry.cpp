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

#include <eltanin/core/geometry.hpp>

#include <gtest/gtest.h>

namespace
{

using eltanin::closest_point_on_segment;
using eltanin::distance_to_segment;
using eltanin::segment_intersection;
using Eigen::Vector2d;

constexpr double kTol = 1e-12;

}  // namespace

TEST(Geometry, FootInsideSegment)
{
  const Vector2d a{0.0, 0.0};
  const Vector2d b{2.0, 0.0};
  const Vector2d p{1.0, 1.5};

  const Vector2d closest = closest_point_on_segment(p, a, b);
  EXPECT_NEAR(closest.x(), 1.0, kTol);
  EXPECT_NEAR(closest.y(), 0.0, kTol);
  EXPECT_NEAR(distance_to_segment(p, a, b), 1.5, kTol);
}

TEST(Geometry, FootBeyondStartClampsToA)
{
  const Vector2d a{0.0, 0.0};
  const Vector2d b{2.0, 0.0};
  const Vector2d p{-3.0, 4.0};

  const Vector2d closest = closest_point_on_segment(p, a, b);
  EXPECT_NEAR(closest.x(), 0.0, kTol);
  EXPECT_NEAR(closest.y(), 0.0, kTol);
  EXPECT_NEAR(distance_to_segment(p, a, b), 5.0, kTol);
}

TEST(Geometry, FootBeyondEndClampsToB)
{
  const Vector2d a{0.0, 0.0};
  const Vector2d b{2.0, 0.0};
  const Vector2d p{5.0, 4.0};

  const Vector2d closest = closest_point_on_segment(p, a, b);
  EXPECT_NEAR(closest.x(), 2.0, kTol);
  EXPECT_NEAR(closest.y(), 0.0, kTol);
  EXPECT_NEAR(distance_to_segment(p, a, b), 5.0, kTol);
}

TEST(Geometry, DegenerateSegmentMeasuresToEndpoint)
{
  const Vector2d a{1.0, 1.0};
  const Vector2d p{4.0, 5.0};

  const Vector2d closest = closest_point_on_segment(p, a, a);
  EXPECT_NEAR(closest.x(), 1.0, kTol);
  EXPECT_NEAR(closest.y(), 1.0, kTol);
  EXPECT_NEAR(distance_to_segment(p, a, a), 5.0, kTol);
}

TEST(Geometry, PointOnSegmentHasZeroDistance)
{
  const Vector2d a{-1.0, -1.0};
  const Vector2d b{3.0, 3.0};
  EXPECT_NEAR(distance_to_segment(Vector2d{1.0, 1.0}, a, b), 0.0, kTol);
  EXPECT_NEAR(distance_to_segment(a, a, b), 0.0, kTol);
  EXPECT_NEAR(distance_to_segment(b, a, b), 0.0, kTol);
}

TEST(Geometry, DistanceAgreesWithClosestPoint)
{
  const Vector2d a{-2.0, 0.5};
  const Vector2d b{1.5, 3.0};
  for (int i = -20; i <= 20; ++i) {
    for (int j = -20; j <= 20; ++j) {
      const Vector2d p{0.4 * static_cast<double>(i), 0.3 * static_cast<double>(j)};
      const double from_closest = (p - closest_point_on_segment(p, a, b)).norm();
      EXPECT_NEAR(distance_to_segment(p, a, b), from_closest, kTol);
    }
  }
}

TEST(Geometry, ClosestPointIsSymmetricInSegmentOrientation)
{
  const Vector2d a{0.0, 0.0};
  const Vector2d b{2.0, 1.0};
  const Vector2d p{0.3, 1.7};
  EXPECT_NEAR(distance_to_segment(p, a, b), distance_to_segment(p, b, a), kTol);
}

TEST(Geometry, SegmentIntersectionCrossing)
{
  const auto point = segment_intersection(
    Vector2d{-1.0, -1.0}, Vector2d{1.0, 1.0}, Vector2d{-1.0, 1.0}, Vector2d{1.0, -1.0});
  ASSERT_TRUE(point.has_value());
  EXPECT_NEAR(point->x(), 0.0, kTol);
  EXPECT_NEAR(point->y(), 0.0, kTol);
}

TEST(Geometry, SegmentIntersectionOffCenterCrossing)
{
  const auto point = segment_intersection(
    Vector2d{0.0, 0.0}, Vector2d{4.0, 0.0}, Vector2d{1.0, -2.0}, Vector2d{1.0, 2.0});
  ASSERT_TRUE(point.has_value());
  EXPECT_NEAR(point->x(), 1.0, kTol);
  EXPECT_NEAR(point->y(), 0.0, kTol);
}

TEST(Geometry, SegmentIntersectionIsSymmetricInArgumentOrder)
{
  const Vector2d a1{0.0, 0.0};
  const Vector2d a2{4.0, 2.0};
  const Vector2d b1{0.0, 2.0};
  const Vector2d b2{4.0, 0.0};

  const auto forward = segment_intersection(a1, a2, b1, b2);
  const auto swapped = segment_intersection(b1, b2, a1, a2);
  const auto reversed = segment_intersection(a2, a1, b2, b1);
  ASSERT_TRUE(forward.has_value());
  ASSERT_TRUE(swapped.has_value());
  ASSERT_TRUE(reversed.has_value());
  EXPECT_NEAR((*forward - *swapped).norm(), 0.0, kTol);
  EXPECT_NEAR((*forward - *reversed).norm(), 0.0, kTol);
}

TEST(Geometry, SegmentIntersectionTouchingEndpointIsIncluded)
{
  // Coordinates are exact in binary, so the closed-interval comparison is not at rounding risk.
  const auto tee = segment_intersection(
    Vector2d{0.0, 0.0}, Vector2d{2.0, 0.0}, Vector2d{1.0, 0.0}, Vector2d{1.0, 1.0});
  ASSERT_TRUE(tee.has_value());
  EXPECT_NEAR(tee->x(), 1.0, kTol);
  EXPECT_NEAR(tee->y(), 0.0, kTol);

  const auto corner = segment_intersection(
    Vector2d{0.0, 0.0}, Vector2d{2.0, 0.0}, Vector2d{2.0, 0.0}, Vector2d{2.0, 2.0});
  ASSERT_TRUE(corner.has_value());
  EXPECT_NEAR(corner->x(), 2.0, kTol);
  EXPECT_NEAR(corner->y(), 0.0, kTol);
}

TEST(Geometry, SegmentIntersectionBeyondTheSegmentIsNullopt)
{
  EXPECT_FALSE(segment_intersection(
                 Vector2d{0.0, 0.0}, Vector2d{1.0, 0.0}, Vector2d{2.0, -1.0}, Vector2d{2.0, 1.0})
                 .has_value());
  EXPECT_FALSE(segment_intersection(
                 Vector2d{0.0, 0.0}, Vector2d{4.0, 0.0}, Vector2d{1.0, 1.0}, Vector2d{1.0, 2.0})
                 .has_value());
}

TEST(Geometry, SegmentIntersectionParallelIsNullopt)
{
  EXPECT_FALSE(segment_intersection(
                 Vector2d{0.0, 0.0}, Vector2d{2.0, 0.0}, Vector2d{0.0, 1.0}, Vector2d{2.0, 1.0})
                 .has_value());
  EXPECT_FALSE(segment_intersection(
                 Vector2d{0.0, 0.0}, Vector2d{2.0, 2.0}, Vector2d{1.0, 0.0}, Vector2d{3.0, 2.0})
                 .has_value());
}

TEST(Geometry, SegmentIntersectionCollinearIsNullopt)
{
  EXPECT_FALSE(segment_intersection(
                 Vector2d{0.0, 0.0}, Vector2d{2.0, 0.0}, Vector2d{1.0, 0.0}, Vector2d{3.0, 0.0})
                 .has_value());
  EXPECT_FALSE(segment_intersection(
                 Vector2d{0.0, 0.0}, Vector2d{2.0, 0.0}, Vector2d{2.0, 0.0}, Vector2d{4.0, 0.0})
                 .has_value());
  EXPECT_FALSE(segment_intersection(
                 Vector2d{0.0, 0.0}, Vector2d{2.0, 0.0}, Vector2d{3.0, 0.0}, Vector2d{5.0, 0.0})
                 .has_value());
}

TEST(Geometry, SegmentIntersectionDegenerateIsNullopt)
{
  const Vector2d point{1.0, 0.0};
  EXPECT_FALSE(
    segment_intersection(point, point, Vector2d{0.0, 0.0}, Vector2d{2.0, 0.0}).has_value());
  EXPECT_FALSE(
    segment_intersection(Vector2d{0.0, 0.0}, Vector2d{2.0, 0.0}, point, point).has_value());
  EXPECT_FALSE(segment_intersection(point, point, point, point).has_value());
}

TEST(Geometry, SegmentIntersectionIsScaleInvariant)
{
  constexpr double scale = 1e6;
  const auto scaled = segment_intersection(
    Vector2d{-scale, -scale}, Vector2d{scale, scale}, Vector2d{-scale, scale},
    Vector2d{scale, -scale});
  ASSERT_TRUE(scaled.has_value());
  EXPECT_NEAR(scaled->norm(), 0.0, 1e-6);

  EXPECT_FALSE(segment_intersection(
                 Vector2d{0.0, 0.0}, Vector2d{2.0 * scale, 0.0}, Vector2d{0.0, scale},
                 Vector2d{2.0 * scale, scale})
                 .has_value());
  EXPECT_FALSE(segment_intersection(
                 Vector2d{0.0, 0.0}, Vector2d{2.0e-6, 0.0}, Vector2d{0.0, 1.0e-6},
                 Vector2d{2.0e-6, 1.0e-6})
                 .has_value());
}
