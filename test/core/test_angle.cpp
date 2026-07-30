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

#include <eltanin/core/angle.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

namespace
{

using eltanin::angle_in_range;
using eltanin::AngleRange;
using eltanin::interpolate_angle;
using eltanin::normalize_angle;
using eltanin::normalize_angle_positive;
using eltanin::shortest_angular_distance;

constexpr double kPi = std::numbers::pi;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kTol = 1e-12;

bool in_normalized_range(double angle)
{
  return angle > -kPi && angle <= kPi;
}

}  // namespace

TEST(Angle, NormalizeZeroAndSmall)
{
  EXPECT_DOUBLE_EQ(normalize_angle(0.0), 0.0);
  EXPECT_DOUBLE_EQ(normalize_angle(1.0), 1.0);
  EXPECT_DOUBLE_EQ(normalize_angle(-1.0), -1.0);
}

TEST(Angle, NormalizeBoundaryMapsBothPiToPositivePi)
{
  EXPECT_DOUBLE_EQ(normalize_angle(kPi), kPi);
  EXPECT_DOUBLE_EQ(normalize_angle(-kPi), kPi);
}

TEST(Angle, NormalizeJustInsideAndOutsideBoundary)
{
  const double eps = 1e-9;
  EXPECT_NEAR(normalize_angle(kPi - eps), kPi - eps, kTol);
  EXPECT_NEAR(normalize_angle(kPi + eps), -kPi + eps, kTol);
  EXPECT_NEAR(normalize_angle(-kPi + eps), -kPi + eps, kTol);
  EXPECT_NEAR(normalize_angle(-kPi - eps), kPi - eps, kTol);
}

TEST(Angle, NormalizeFullTurns)
{
  EXPECT_NEAR(normalize_angle(kTwoPi), 0.0, kTol);
  EXPECT_NEAR(normalize_angle(-kTwoPi), 0.0, kTol);
  EXPECT_NEAR(normalize_angle(kTwoPi + 0.5), 0.5, kTol);
  EXPECT_NEAR(normalize_angle(-kTwoPi - 0.5), -0.5, kTol);
}

TEST(Angle, NormalizeLargeMagnitudeInOneCall)
{
  // At odd multiples of pi the result sign depends on rounding of the input, so compare |value|.
  EXPECT_NEAR(std::abs(normalize_angle(3.0 * kPi)), kPi, 1e-12);
  EXPECT_NEAR(std::abs(normalize_angle(-3.0 * kPi)), kPi, 1e-12);
  EXPECT_NEAR(std::abs(normalize_angle(101.0 * kPi)), kPi, 1e-9);
  EXPECT_NEAR(normalize_angle(100.0 * kPi), 0.0, 1e-9);
  EXPECT_NEAR(normalize_angle(-100.0 * kPi), 0.0, 1e-9);
  EXPECT_NEAR(normalize_angle(3.0 * kPi + 0.5), 0.5 - kPi, 1e-12);
  EXPECT_TRUE(in_normalized_range(normalize_angle(3.0 * kPi)));
  EXPECT_TRUE(in_normalized_range(normalize_angle(-3.0 * kPi)));
  EXPECT_TRUE(in_normalized_range(normalize_angle(101.0 * kPi)));
}

TEST(Angle, NormalizeSweepStaysInRange)
{
  for (int k = -200; k <= 200; ++k) {
    const double angle = 0.37 * static_cast<double>(k) * kPi;
    EXPECT_TRUE(in_normalized_range(normalize_angle(angle))) << "k=" << k;
  }
}

TEST(Angle, NormalizeNonFiniteReturnsNan)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(std::isnan(normalize_angle(nan)));
  EXPECT_TRUE(std::isnan(normalize_angle(inf)));
  EXPECT_TRUE(std::isnan(normalize_angle(-inf)));
  EXPECT_TRUE(std::isnan(normalize_angle_positive(nan)));
  EXPECT_TRUE(std::isnan(normalize_angle_positive(inf)));
}

TEST(Angle, NormalizePositiveRange)
{
  EXPECT_DOUBLE_EQ(normalize_angle_positive(0.0), 0.0);
  EXPECT_DOUBLE_EQ(normalize_angle_positive(1.0), 1.0);
  EXPECT_NEAR(normalize_angle_positive(-1.0), kTwoPi - 1.0, kTol);
  EXPECT_NEAR(normalize_angle_positive(kTwoPi), 0.0, kTol);
  EXPECT_NEAR(normalize_angle_positive(-kTwoPi), 0.0, kTol);
  EXPECT_DOUBLE_EQ(normalize_angle_positive(kPi), kPi);
}

TEST(Angle, NormalizePositiveTinyNegativeDoesNotReachTwoPi)
{
  const double result = normalize_angle_positive(-1e-17);
  EXPECT_GE(result, 0.0);
  EXPECT_LT(result, kTwoPi);
}

TEST(Angle, NormalizePositiveSweepStaysInRange)
{
  for (int k = -200; k <= 200; ++k) {
    const double angle = 0.37 * static_cast<double>(k) * kPi;
    const double result = normalize_angle_positive(angle);
    EXPECT_GE(result, 0.0) << "k=" << k;
    EXPECT_LT(result, kTwoPi) << "k=" << k;
  }
}

TEST(Angle, ShortestAngularDistanceAcrossPi)
{
  EXPECT_NEAR(shortest_angular_distance(0.1, -0.1), -0.2, kTol);
  EXPECT_NEAR(shortest_angular_distance(kPi - 0.1, -kPi + 0.1), 0.2, kTol);
  EXPECT_NEAR(shortest_angular_distance(-kPi + 0.1, kPi - 0.1), -0.2, kTol);
  EXPECT_NEAR(shortest_angular_distance(0.0, 0.5), 0.5, kTol);
}

TEST(Angle, ShortestAngularDistanceRange)
{
  for (int i = -50; i <= 50; ++i) {
    for (int j = -50; j <= 50; ++j) {
      const double from = 0.31 * static_cast<double>(i);
      const double to = 0.29 * static_cast<double>(j);
      EXPECT_TRUE(in_normalized_range(shortest_angular_distance(from, to)));
    }
  }
}

TEST(Angle, InterpolateHitsBothEndpoints)
{
  EXPECT_DOUBLE_EQ(interpolate_angle(0.3, 1.2, 0.0), 0.3);
  EXPECT_NEAR(interpolate_angle(0.3, 1.2, 1.0), 1.2, kTol);
  EXPECT_NEAR(interpolate_angle(kPi - 0.1, -kPi + 0.1, 1.0), -kPi + 0.1, kTol);
}

TEST(Angle, InterpolateIsLinearInside)
{
  EXPECT_NEAR(interpolate_angle(0.0, 1.0, 0.25), 0.25, kTol);
  EXPECT_NEAR(interpolate_angle(0.0, 1.0, 0.5), 0.5, kTol);
  EXPECT_NEAR(interpolate_angle(-0.4, 0.4, 0.5), 0.0, kTol);
}

TEST(Angle, InterpolateClampsRatio)
{
  EXPECT_DOUBLE_EQ(interpolate_angle(0.3, 1.2, -1.0), interpolate_angle(0.3, 1.2, 0.0));
  EXPECT_DOUBLE_EQ(interpolate_angle(0.3, 1.2, 5.0), interpolate_angle(0.3, 1.2, 1.0));
  EXPECT_DOUBLE_EQ(interpolate_angle(0.3, 1.2, -1e-9), 0.3);
}

TEST(Angle, InterpolateTakesTheShortestRotation)
{
  // 3.0 -> -3.0 is +0.283 rad across pi, not -6.0 rad through zero.
  const double result = interpolate_angle(3.0, -3.0, 0.5);
  EXPECT_NEAR(std::abs(shortest_angular_distance(result, kPi)), 0.0, 1e-9);
  EXPECT_GT(std::abs(result), 3.0);
  // The midpoint lands on the (-pi, pi] boundary, so compare it as an angle, not as a value.
  const double mirrored = interpolate_angle(-3.0, 3.0, 0.5);
  EXPECT_NEAR(std::abs(shortest_angular_distance(mirrored, kPi)), 0.0, 1e-9);
  EXPECT_NEAR(interpolate_angle(3.0, -3.0, 0.25), 3.0707963267948966, kTol);
  EXPECT_NEAR(interpolate_angle(-3.0, 3.0, 0.25), -3.0707963267948966, kTol);
}

TEST(Angle, InterpolateResultStaysNormalized)
{
  for (int i = -30; i <= 30; ++i) {
    for (int j = -30; j <= 30; ++j) {
      const double from = 0.41 * static_cast<double>(i);
      const double to = 0.37 * static_cast<double>(j);
      for (const double t : {0.0, 0.1, 0.5, 0.9, 1.0}) {
        EXPECT_TRUE(in_normalized_range(interpolate_angle(from, to, t)))
          << "from=" << from << " to=" << to << " t=" << t;
      }
    }
  }
}

TEST(Angle, InterpolateAntipodalIsDeterministic)
{
  // shortest_angular_distance returns +pi for an exactly opposite pair, so the arc goes CCW.
  EXPECT_NEAR(interpolate_angle(0.0, kPi, 0.5), 0.5 * kPi, kTol);
  EXPECT_NEAR(interpolate_angle(0.0, -kPi, 0.5), 0.5 * kPi, kTol);
  EXPECT_DOUBLE_EQ(interpolate_angle(0.0, kPi, 0.5), interpolate_angle(0.0, -kPi, 0.5));
}

TEST(Angle, InterpolateIgnoresFullTurnsInTheInput)
{
  for (const double t : {0.0, 0.25, 0.5, 1.0}) {
    const double expected = interpolate_angle(0.4, -1.1, t);
    EXPECT_NEAR(interpolate_angle(0.4 + kTwoPi, -1.1, t), expected, kTol) << "t=" << t;
    EXPECT_NEAR(interpolate_angle(0.4, -1.1 - kTwoPi, t), expected, kTol) << "t=" << t;
  }
}

TEST(Angle, InRangeWithoutWrapIncludesBothEnds)
{
  EXPECT_TRUE(angle_in_range(0.0, 0.0, 0.5 * kPi));
  EXPECT_TRUE(angle_in_range(0.5 * kPi, 0.0, 0.5 * kPi));
  EXPECT_TRUE(angle_in_range(0.25 * kPi, 0.0, 0.5 * kPi));
  EXPECT_FALSE(angle_in_range(-0.25 * kPi, 0.0, 0.5 * kPi));
  EXPECT_FALSE(angle_in_range(0.75 * kPi, 0.0, 0.5 * kPi));
  EXPECT_FALSE(angle_in_range(kPi, 0.0, 0.5 * kPi));
}

TEST(Angle, InRangeAcrossPi)
{
  const double from = 0.75 * kPi;
  const double to = -0.75 * kPi;
  EXPECT_TRUE(angle_in_range(kPi, from, to));
  EXPECT_TRUE(angle_in_range(-kPi, from, to));
  EXPECT_TRUE(angle_in_range(from, from, to));
  EXPECT_TRUE(angle_in_range(to, from, to));
  EXPECT_TRUE(angle_in_range(0.9 * kPi, from, to));
  EXPECT_TRUE(angle_in_range(-0.9 * kPi, from, to));
  EXPECT_FALSE(angle_in_range(0.0, from, to));
  EXPECT_FALSE(angle_in_range(0.5 * kPi, from, to));
  EXPECT_FALSE(angle_in_range(-0.5 * kPi, from, to));
}

TEST(Angle, InRangeWithEqualEndsMatchesOnlyThatAngle)
{
  EXPECT_TRUE(angle_in_range(0.3, 0.3, 0.3));
  EXPECT_TRUE(angle_in_range(0.3 + kTwoPi, 0.3, 0.3));
  EXPECT_FALSE(angle_in_range(0.3 + 1e-6, 0.3, 0.3));
  EXPECT_FALSE(angle_in_range(0.3 - 1e-6, 0.3, 0.3));
}

TEST(Angle, InRangeIgnoresFullTurnsInTheInput)
{
  const double from = -0.25 * kPi;
  const double to = 0.25 * kPi;
  for (const double angle : {-0.5 * kPi, -0.1, 0.0, 0.1, 0.5 * kPi, 2.0}) {
    const bool expected = angle_in_range(angle, from, to);
    EXPECT_EQ(angle_in_range(angle + kTwoPi, from, to), expected) << "angle=" << angle;
    EXPECT_EQ(angle_in_range(angle - 2.0 * kTwoPi, from, to), expected) << "angle=" << angle;
    EXPECT_EQ(angle_in_range(angle, from + kTwoPi, to - kTwoPi), expected) << "angle=" << angle;
  }
}

TEST(Angle, InRangeNonFiniteIsFalse)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double bad : {nan, inf, -inf}) {
    EXPECT_FALSE(angle_in_range(bad, 0.0, kPi));
    EXPECT_FALSE(angle_in_range(0.5, bad, kPi));
    EXPECT_FALSE(angle_in_range(0.5, 0.0, bad));
  }
}

TEST(Angle, InRangeReversedArcIsTheComplement)
{
  const double from = 0.6;
  const double to = 2.4;
  for (int k = 0; k <= 400; ++k) {
    const double angle = -kTwoPi + 0.0157 * static_cast<double>(k);
    const double offset_from = std::abs(shortest_angular_distance(from, angle));
    const double offset_to = std::abs(shortest_angular_distance(to, angle));
    if (offset_from < 1e-6 || offset_to < 1e-6) {
      continue;
    }
    EXPECT_NE(angle_in_range(angle, from, to), angle_in_range(angle, to, from)) << "k=" << k;
  }
}

TEST(Angle, InRangeFromMinusPiToPiIsNotAFullTurn)
{
  EXPECT_TRUE(angle_in_range(-kPi, -kPi, kPi));
  EXPECT_TRUE(angle_in_range(kPi, -kPi, kPi));
  EXPECT_FALSE(angle_in_range(0.0, -kPi, kPi));
  EXPECT_FALSE(angle_in_range(0.5 * kPi, -kPi, kPi));
  EXPECT_FALSE(angle_in_range(-0.5 * kPi, -kPi, kPi));
}

TEST(Angle, AngleRangeDefaultsToZero)
{
  const AngleRange range;
  EXPECT_DOUBLE_EQ(range.from, 0.0);
  EXPECT_DOUBLE_EQ(range.to, 0.0);
  EXPECT_TRUE(angle_in_range(0.0, range.from, range.to));
  EXPECT_FALSE(angle_in_range(1.0, range.from, range.to));
}
