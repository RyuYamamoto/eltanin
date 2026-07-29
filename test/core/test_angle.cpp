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

namespace
{

using eltanin::kPi;
using eltanin::kTwoPi;
using eltanin::normalize_angle;
using eltanin::normalize_angle_positive;
using eltanin::shortest_angular_distance;

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
