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
#include <eltanin/sensor/scan_projection.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace
{

using eltanin::AngleRange;
using eltanin::shortest_angular_distance;
using eltanin::Transform2D;
using eltanin::sensor::project_scan;
using eltanin::sensor::ScanData;
using eltanin::sensor::ScanFilter;

constexpr double kPi = std::numbers::pi;
constexpr double kTol = 1e-12;
constexpr float kNanF = std::numeric_limits<float>::quiet_NaN();
constexpr float kInfF = std::numeric_limits<float>::infinity();

/// Beam angle as the implementation must compute it, so expectations never accumulate error.
double beam_angle(const ScanData & scan, std::size_t i)
{
  return scan.angle_min + static_cast<double>(i) * scan.angle_increment;
}

ScanData uniform_scan(double angle_min, double angle_increment, std::size_t count, double range)
{
  ScanData scan;
  scan.angle_min = angle_min;
  scan.angle_increment = angle_increment;
  scan.ranges.assign(count, static_cast<float>(range));
  return scan;
}

void expect_point_near(const Eigen::Vector2d & actual, double x, double y)
{
  EXPECT_NEAR(actual.x(), x, kTol);
  EXPECT_NEAR(actual.y(), y, kTol);
}

/// Compares bearings through the shortest rotation so that a beam at +-pi does not flip sign.
void expect_bearing_near(const Eigen::Vector2d & point, double expected_angle)
{
  const double bearing = std::atan2(point.y(), point.x());
  EXPECT_NEAR(shortest_angular_distance(expected_angle, bearing), 0.0, kTol);
}

}  // namespace

TEST(ScanProjection, RejectsInvalidScanAndFilterArguments)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<Eigen::Vector2d> points;

  ScanData scan = uniform_scan(0.0, 0.1, 4, 1.0);
  scan.angle_min = nan;
  EXPECT_THROW(project_scan(scan, ScanFilter{}, points), std::invalid_argument);
  scan = uniform_scan(0.0, nan, 4, 1.0);
  EXPECT_THROW(project_scan(scan, ScanFilter{}, points), std::invalid_argument);

  scan = uniform_scan(0.0, 0.1, 4, 1.0);
  ScanFilter filter;
  filter.min_range = -0.1;
  EXPECT_THROW(project_scan(scan, filter, points), std::invalid_argument);
  filter = ScanFilter{};
  filter.min_range = 2.0;
  filter.max_range = 1.0;
  EXPECT_THROW(project_scan(scan, filter, points), std::invalid_argument);
  filter = ScanFilter{};
  filter.angle_range = AngleRange{0.0, nan};
  EXPECT_THROW(project_scan(scan, filter, points), std::invalid_argument);
}

TEST(ScanProjection, QuadrantBeamsLandOnTheAxes)
{
  const ScanData scan = uniform_scan(0.0, 0.5 * kPi, 4, 1.0);
  std::vector<Eigen::Vector2d> points;
  project_scan(scan, ScanFilter{}, points);

  ASSERT_EQ(points.size(), 4u);
  expect_point_near(points[0], 1.0, 0.0);
  expect_point_near(points[1], 0.0, 1.0);
  expect_point_near(points[2], -1.0, 0.0);
  expect_point_near(points[3], 0.0, -1.0);
}

TEST(ScanProjection, NonZeroAngleMinRotatesEveryPoint)
{
  const ScanData scan = uniform_scan(-0.5 * kPi, 0.5 * kPi, 4, 2.0);
  std::vector<Eigen::Vector2d> points;
  project_scan(scan, ScanFilter{}, points);

  ASSERT_EQ(points.size(), 4u);
  expect_point_near(points[0], 0.0, -2.0);
  expect_point_near(points[1], 2.0, 0.0);
  expect_point_near(points[2], 0.0, 2.0);
  expect_point_near(points[3], -2.0, 0.0);
}

TEST(ScanProjection, NegativeIncrementSweepsClockwise)
{
  const ScanData scan = uniform_scan(0.0, -0.5 * kPi, 4, 1.0);
  std::vector<Eigen::Vector2d> points;
  project_scan(scan, ScanFilter{}, points);

  ASSERT_EQ(points.size(), 4u);
  expect_point_near(points[0], 1.0, 0.0);
  expect_point_near(points[1], 0.0, -1.0);
  expect_point_near(points[2], -1.0, 0.0);
  expect_point_near(points[3], 0.0, 1.0);
}

TEST(ScanProjection, NormAndBearingMatchRangeAndBeamAngle)
{
  const ScanData scan = uniform_scan(-1.2, 0.17, 12, 3.5);
  std::vector<Eigen::Vector2d> points;
  project_scan(scan, ScanFilter{}, points);

  ASSERT_EQ(points.size(), scan.ranges.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_NEAR(points[i].norm(), 3.5, kTol) << "i=" << i;
    expect_bearing_near(points[i], beam_angle(scan, i));
  }
}

TEST(ScanProjection, LastBeamAngleIsNotAccumulated)
{
  const ScanData scan = uniform_scan(-1.57, 3.14 / 719.0, 720, 1.0);
  std::vector<Eigen::Vector2d> points;
  project_scan(scan, ScanFilter{}, points);

  ASSERT_EQ(points.size(), 720u);
  const double expected = scan.angle_min + 719.0 * scan.angle_increment;
  expect_bearing_near(points.back(), expected);
  EXPECT_NEAR(expected, 1.57, 1e-15);
}

TEST(ScanProjection, EmptyAndSingleBeamScans)
{
  std::vector<Eigen::Vector2d> points;

  project_scan(uniform_scan(0.0, 0.1, 0, 1.0), ScanFilter{}, points);
  EXPECT_TRUE(points.empty());

  project_scan(uniform_scan(0.0, 0.1, 1, 1.0), ScanFilter{}, points);
  ASSERT_EQ(points.size(), 1u);
  expect_point_near(points[0], 1.0, 0.0);
}

TEST(ScanProjection, SurvivingBeamsKeepAscendingIndexOrder)
{
  ScanData scan = uniform_scan(-1.0, 0.2, 10, 1.0);
  scan.ranges[2] = kNanF;
  scan.ranges[5] = 100.0F;
  scan.ranges[7] = -1.0F;
  ScanFilter filter;
  filter.max_range = 5.0;

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, filter, points);

  const std::vector<std::size_t> expected_beams{0, 1, 3, 4, 6, 8, 9};
  ASSERT_EQ(points.size(), expected_beams.size());
  double previous = -kPi;
  for (std::size_t i = 0; i < points.size(); ++i) {
    expect_bearing_near(points[i], beam_angle(scan, expected_beams[i]));
    const double bearing = std::atan2(points[i].y(), points[i].x());
    EXPECT_GT(bearing, previous) << "i=" << i;
    previous = bearing;
  }
}

TEST(ScanProjection, OutputBufferIsClearedNotAppendedTo)
{
  std::vector<Eigen::Vector2d> points{Eigen::Vector2d{9.0, 9.0}, Eigen::Vector2d{8.0, 8.0}};
  project_scan(uniform_scan(0.0, 0.5 * kPi, 2, 1.0), ScanFilter{}, points);

  ASSERT_EQ(points.size(), 2u);
  expect_point_near(points[0], 1.0, 0.0);
  expect_point_near(points[1], 0.0, 1.0);
}

TEST(ScanProjection, NonFiniteRangesAreDiscarded)
{
  ScanData scan = uniform_scan(0.0, 0.1, 5, 1.0);
  scan.ranges[1] = kNanF;
  scan.ranges[2] = kInfF;
  scan.ranges[3] = -kInfF;

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, ScanFilter{}, points);

  ASSERT_EQ(points.size(), 2u);
  expect_bearing_near(points[0], beam_angle(scan, 0));
  expect_bearing_near(points[1], beam_angle(scan, 4));
}

TEST(ScanProjection, RangeBoundsAreClosed)
{
  ScanData scan = uniform_scan(0.0, 0.1, 4, 1.0);
  scan.ranges = {0.5F, 2.5F, 1.0F, 1.5F};
  ScanFilter filter;
  filter.min_range = 0.5;
  filter.max_range = 2.5;

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, filter, points);

  ASSERT_EQ(points.size(), 4u);
  EXPECT_NEAR(points[0].norm(), 0.5, kTol);
  EXPECT_NEAR(points[1].norm(), 2.5, kTol);
}

TEST(ScanProjection, ValuesJustOutsideTheBoundsAreDiscarded)
{
  ScanData scan = uniform_scan(0.0, 0.1, 4, 1.0);
  scan.ranges = {0.499F, 2.501F, 0.5F, 2.5F};
  ScanFilter filter;
  filter.min_range = 0.5;
  filter.max_range = 2.5;

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, filter, points);

  ASSERT_EQ(points.size(), 2u);
  EXPECT_NEAR(points[0].norm(), 0.5, kTol);
  EXPECT_NEAR(points[1].norm(), 2.5, kTol);
}

TEST(ScanProjection, BoundsIntersectTheSensorLimitsWithTheFilter)
{
  ScanData scan = uniform_scan(0.0, 0.1, 5, 1.0);
  scan.range_min = 0.1;
  scan.range_max = 30.0;
  scan.ranges = {0.05F, 0.5F, 5.0F, 6.0F, 20.0F};
  ScanFilter filter;
  filter.min_range = 0.0;
  filter.max_range = 5.0;

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, filter, points);

  ASSERT_EQ(points.size(), 2u);
  EXPECT_NEAR(points[0].norm(), 0.5, kTol);
  EXPECT_NEAR(points[1].norm(), 5.0, kTol);
}

TEST(ScanProjection, SensorLimitsApplyWhenTheFilterIsWider)
{
  ScanData scan = uniform_scan(0.0, 0.1, 4, 1.0);
  scan.range_min = 1.0;
  scan.range_max = 2.0;
  scan.ranges = {0.5F, 1.0F, 2.0F, 2.5F};

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, ScanFilter{}, points);

  ASSERT_EQ(points.size(), 2u);
  EXPECT_NEAR(points[0].norm(), 1.0, kTol);
  EXPECT_NEAR(points[1].norm(), 2.0, kTol);
}

TEST(ScanProjection, NonFiniteSensorLimitsMeanNoLimit)
{
  ScanData scan = uniform_scan(0.0, 0.1, 4, 1.0);
  scan.range_min = std::numeric_limits<double>::quiet_NaN();
  scan.range_max = std::numeric_limits<double>::infinity();
  scan.ranges = {0.5F, 1.0F, 2.0F, 2.5F};
  ScanFilter filter;
  filter.min_range = 1.0;
  filter.max_range = 2.0;

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, filter, points);

  ASSERT_EQ(points.size(), 2u);
  EXPECT_NEAR(points[0].norm(), 1.0, kTol);
  EXPECT_NEAR(points[1].norm(), 2.0, kTol);
}

TEST(ScanProjection, EmptyEffectiveIntervalYieldsNoPoints)
{
  ScanData scan = uniform_scan(0.0, 0.1, 4, 1.5);
  scan.range_min = 3.0;
  scan.range_max = 10.0;
  ScanFilter filter;
  filter.min_range = 0.0;
  filter.max_range = 1.0;

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, filter, points);

  EXPECT_TRUE(points.empty());
}

TEST(ScanProjection, NegativeRangesAreDiscarded)
{
  ScanData scan = uniform_scan(0.0, 0.1, 3, 1.0);
  scan.ranges = {-1.0F, -0.001F, 1.0F};

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, ScanFilter{}, points);

  ASSERT_EQ(points.size(), 1u);
  EXPECT_NEAR(points[0].norm(), 1.0, kTol);
}

TEST(ScanProjection, OutputSizeEqualsTheSurvivingBeamCountWithNoPlaceholders)
{
  ScanData scan = uniform_scan(0.0, 0.05, 40, 1.0);
  for (std::size_t i = 0; i < scan.ranges.size(); i += 3) {
    scan.ranges[i] = kNanF;
  }
  std::size_t surviving = 0;
  for (const float range : scan.ranges) {
    if (std::isfinite(range)) {
      ++surviving;
    }
  }

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, ScanFilter{}, points);

  EXPECT_EQ(points.size(), surviving);
  EXPECT_LT(points.size(), scan.ranges.size());
}

TEST(ScanProjection, InputScanIsNotModified)
{
  ScanData scan = uniform_scan(-0.4, 0.05, 20, 1.0);
  scan.range_min = 0.1;
  scan.range_max = 30.0;
  scan.ranges[3] = kNanF;
  scan.ranges[7] = kInfF;
  scan.ranges[9] = 40.0F;
  scan.ranges[11] = 0.05F;
  const ScanData before = scan;
  ScanFilter filter;
  filter.min_range = 0.2;
  filter.max_range = 5.0;
  filter.angle_range = AngleRange{-0.3, 0.3};

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, filter, points);

  EXPECT_DOUBLE_EQ(scan.angle_min, before.angle_min);
  EXPECT_DOUBLE_EQ(scan.angle_increment, before.angle_increment);
  EXPECT_DOUBLE_EQ(scan.range_min, before.range_min);
  EXPECT_DOUBLE_EQ(scan.range_max, before.range_max);
  ASSERT_EQ(scan.ranges.size(), before.ranges.size());
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    if (std::isnan(before.ranges[i])) {
      EXPECT_TRUE(std::isnan(scan.ranges[i])) << "i=" << i;
    } else {
      EXPECT_EQ(scan.ranges[i], before.ranges[i]) << "i=" << i;
    }
  }
}

TEST(ScanProjection, AngleRangeKeepsTheBeamsOnTheArcIncludingBothEnds)
{
  const ScanData scan = uniform_scan(-0.5 * kPi, 0.125 * kPi, 9, 1.0);
  ScanFilter filter;
  filter.angle_range = AngleRange{beam_angle(scan, 2), beam_angle(scan, 6)};

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, filter, points);

  ASSERT_EQ(points.size(), 5u);
  for (std::size_t i = 0; i < points.size(); ++i) {
    expect_bearing_near(points[i], beam_angle(scan, i + 2));
  }
}

TEST(ScanProjection, AngleRangeAcrossPiFiltersCorrectly)
{
  const ScanData scan = uniform_scan(-kPi, 0.25 * kPi, 8, 1.0);
  ScanFilter filter;
  filter.angle_range = AngleRange{0.7 * kPi, -0.7 * kPi};

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, filter, points);

  const std::vector<std::size_t> expected_beams{0, 1, 7};
  ASSERT_EQ(points.size(), expected_beams.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    expect_bearing_near(points[i], beam_angle(scan, expected_beams[i]));
  }
}

TEST(ScanProjection, UnsetAngleRangeKeepsEveryBeamButMinusPiToPiDoesNot)
{
  const ScanData scan = uniform_scan(-0.5 * kPi, 0.125 * kPi, 9, 1.0);

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, ScanFilter{}, points);
  EXPECT_EQ(points.size(), 9u);

  ScanFilter full_turn_attempt;
  full_turn_attempt.angle_range = AngleRange{-kPi, kPi};
  project_scan(scan, full_turn_attempt, points);
  EXPECT_TRUE(points.empty());
}

TEST(ScanProjection, RangeAndAngleFiltersApplyTogether)
{
  ScanData scan = uniform_scan(-0.5 * kPi, 0.125 * kPi, 9, 1.0);
  scan.ranges[3] = 10.0F;
  scan.ranges[5] = kNanF;
  ScanFilter filter;
  filter.max_range = 5.0;
  filter.angle_range = AngleRange{beam_angle(scan, 2), beam_angle(scan, 6)};

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, filter, points);

  const std::vector<std::size_t> expected_beams{2, 4, 6};
  ASSERT_EQ(points.size(), expected_beams.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    expect_bearing_near(points[i], beam_angle(scan, expected_beams[i]));
  }
}

TEST(ScanProjection, WorldOverloadEqualsMappingTheSensorFramePoints)
{
  ScanData scan = uniform_scan(-1.0, 0.13, 24, 2.0);
  scan.ranges[5] = kNanF;
  scan.ranges[9] = 40.0F;
  ScanFilter filter;
  filter.max_range = 5.0;
  const Transform2D sensor_to_world(Eigen::Vector2d{1.5, -0.75}, 0.6);

  std::vector<Eigen::Vector2d> sensor_points;
  project_scan(scan, filter, sensor_points);
  std::vector<Eigen::Vector2d> world_points;
  project_scan(scan, filter, sensor_to_world, world_points);

  ASSERT_EQ(world_points.size(), sensor_points.size());
  for (std::size_t i = 0; i < world_points.size(); ++i) {
    const Eigen::Vector2d expected = sensor_to_world * sensor_points[i];
    EXPECT_DOUBLE_EQ(world_points[i].x(), expected.x()) << "i=" << i;
    EXPECT_DOUBLE_EQ(world_points[i].y(), expected.y()) << "i=" << i;
  }
}

TEST(ScanProjection, WorldRoundTripThroughTheInverseTransform)
{
  const ScanData scan = uniform_scan(-0.9, 0.11, 18, 3.0);
  const Transform2D sensor_to_world(Eigen::Vector2d{-2.25, 4.5}, -2.1);

  std::vector<Eigen::Vector2d> sensor_points;
  project_scan(scan, ScanFilter{}, sensor_points);
  std::vector<Eigen::Vector2d> world_points;
  project_scan(scan, ScanFilter{}, sensor_to_world, world_points);

  ASSERT_EQ(world_points.size(), sensor_points.size());
  const Transform2D world_to_sensor = sensor_to_world.inverse();
  for (std::size_t i = 0; i < world_points.size(); ++i) {
    const Eigen::Vector2d back = world_to_sensor * world_points[i];
    EXPECT_NEAR(back.x(), sensor_points[i].x(), kTol) << "i=" << i;
    EXPECT_NEAR(back.y(), sensor_points[i].y(), kTol) << "i=" << i;
  }
}

TEST(ScanProjection, IdentityTransformLeavesThePointsInTheSensorFrame)
{
  const ScanData scan = uniform_scan(-0.5, 0.2, 6, 1.25);

  std::vector<Eigen::Vector2d> sensor_points;
  project_scan(scan, ScanFilter{}, sensor_points);
  std::vector<Eigen::Vector2d> world_points;
  project_scan(scan, ScanFilter{}, Transform2D{}, world_points);

  ASSERT_EQ(world_points.size(), sensor_points.size());
  for (std::size_t i = 0; i < world_points.size(); ++i) {
    EXPECT_DOUBLE_EQ(world_points[i].x(), sensor_points[i].x()) << "i=" << i;
    EXPECT_DOUBLE_EQ(world_points[i].y(), sensor_points[i].y()) << "i=" << i;
  }
}

TEST(ScanProjection, TranslationRotationAndCombinedTransforms)
{
  const ScanData scan = uniform_scan(0.0, 0.5 * kPi, 4, 1.0);

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, ScanFilter{}, Transform2D(Eigen::Vector2d{3.0, -1.0}, 0.0), points);
  ASSERT_EQ(points.size(), 4u);
  expect_point_near(points[0], 4.0, -1.0);
  expect_point_near(points[1], 3.0, 0.0);

  project_scan(scan, ScanFilter{}, Transform2D(Eigen::Vector2d::Zero(), 0.5 * kPi), points);
  ASSERT_EQ(points.size(), 4u);
  expect_point_near(points[0], 0.0, 1.0);
  expect_point_near(points[1], -1.0, 0.0);

  project_scan(scan, ScanFilter{}, Transform2D(Eigen::Vector2d{3.0, -1.0}, 0.5 * kPi), points);
  ASSERT_EQ(points.size(), 4u);
  expect_point_near(points[0], 3.0, 0.0);
  expect_point_near(points[1], 2.0, -1.0);
}

TEST(ScanProjection, SurvivingBeamCountDoesNotDependOnTheTransform)
{
  ScanData scan = uniform_scan(-kPi, 0.05, 120, 2.0);
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  scan.ranges[60] = kNanF;
  scan.ranges[64] = 20.0F;
  ScanFilter filter;
  filter.max_range = 5.0;
  filter.angle_range = AngleRange{-0.4, 0.9};

  std::vector<Eigen::Vector2d> sensor_points;
  project_scan(scan, filter, sensor_points);
  ASSERT_FALSE(sensor_points.empty());

  const Transform2D transforms[]{
    Transform2D{}, Transform2D(Eigen::Vector2d{10.0, -10.0}, 0.0),
    Transform2D(Eigen::Vector2d::Zero(), 2.5), Transform2D(Eigen::Vector2d{-3.0, 7.0}, -1.9)};
  std::vector<Eigen::Vector2d> world_points;
  for (const Transform2D & sensor_to_world : transforms) {
    project_scan(scan, filter, sensor_to_world, world_points);
    EXPECT_EQ(world_points.size(), sensor_points.size());
  }
}

TEST(ScanProjection, KnownSensorPoseGivesTheHandComputedWorldPoint)
{
  const ScanData scan = uniform_scan(0.0, 0.5 * kPi, 1, 1.0);
  const Transform2D sensor_to_world(Eigen::Vector2d{1.0, 2.0}, 0.5 * kPi);

  std::vector<Eigen::Vector2d> points;
  project_scan(scan, ScanFilter{}, sensor_to_world, points);

  ASSERT_EQ(points.size(), 1u);
  expect_point_near(points[0], 1.0, 3.0);
}
