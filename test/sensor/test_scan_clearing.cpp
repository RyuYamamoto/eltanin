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
#include <eltanin/sensor/scan_clearing.hpp>
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
using eltanin::sensor::project_scan_for_clearing;
using eltanin::sensor::ScanData;
using eltanin::sensor::ScanFilter;

constexpr double kPi = std::numbers::pi;
constexpr double kTol = 1e-12;
constexpr double kInf = std::numeric_limits<double>::infinity();
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

TEST(ScanClearing, RejectsInvalidProjectionArguments)
{
  const ScanData scan = uniform_scan(0.0, 0.1, 4, 1.0);
  std::vector<Eigen::Vector2d> endpoints;
  EXPECT_THROW(
    project_scan_for_clearing(scan, ScanFilter{}, -0.1, endpoints), std::invalid_argument);
  EXPECT_THROW(
    project_scan_for_clearing(
      scan, ScanFilter{}, std::numeric_limits<double>::quiet_NaN(), endpoints),
    std::invalid_argument);

  ScanFilter filter;
  filter.min_range = -0.1;
  EXPECT_THROW(project_scan_for_clearing(scan, filter, 3.0, endpoints), std::invalid_argument);
}

TEST(ScanClearing, EndpointsSitAtTheReportedRange)
{
  ScanData scan = uniform_scan(0.0, 0.5 * kPi, 4, 2.0);
  scan.range_max = 30.0;

  std::vector<Eigen::Vector2d> endpoints;
  project_scan_for_clearing(scan, ScanFilter{}, 10.0, endpoints);

  ASSERT_EQ(endpoints.size(), 4u);
  expect_point_near(endpoints[0], 2.0, 0.0);
  expect_point_near(endpoints[1], 0.0, 2.0);
  expect_point_near(endpoints[2], -2.0, 0.0);
  expect_point_near(endpoints[3], 0.0, -2.0);
}

TEST(ScanClearing, InfiniteBeamIsReplacedByTheSensorMaximum)
{
  ScanData scan = uniform_scan(0.0, 0.1, 3, 1.0);
  scan.range_max = 10.0;
  scan.ranges[1] = kInfF;

  std::vector<Eigen::Vector2d> endpoints;
  project_scan_for_clearing(scan, ScanFilter{}, kInf, endpoints);

  ASSERT_EQ(endpoints.size(), 3u);
  EXPECT_NEAR(endpoints[0].norm(), 1.0, kTol);
  EXPECT_NEAR(endpoints[1].norm(), 10.0, kTol);
  expect_bearing_near(endpoints[1], beam_angle(scan, 1));
}

TEST(ScanClearing, ReplacedInfiniteBeamIsStillTruncatedByTheClearingRange)
{
  ScanData scan = uniform_scan(0.0, 0.1, 1, 1.0);
  scan.range_max = 10.0;
  scan.ranges[0] = kInfF;

  std::vector<Eigen::Vector2d> endpoints;
  project_scan_for_clearing(scan, ScanFilter{}, 3.0, endpoints);

  ASSERT_EQ(endpoints.size(), 1u);
  EXPECT_NEAR(endpoints[0].norm(), 3.0, kTol);
}

TEST(ScanClearing, InfiniteBeamIsDroppedWhenTheSensorDeclaresNoMaximum)
{
  ScanData scan = uniform_scan(0.0, 0.1, 3, 1.0);
  scan.ranges[1] = kInfF;

  std::vector<Eigen::Vector2d> endpoints;
  project_scan_for_clearing(scan, ScanFilter{}, kInf, endpoints);

  ASSERT_EQ(endpoints.size(), 2u);
  expect_bearing_near(endpoints[0], beam_angle(scan, 0));
  expect_bearing_near(endpoints[1], beam_angle(scan, 2));
}

TEST(ScanClearing, NanAndNegativeInfinityAndNegativeRangesAreDropped)
{
  ScanData scan = uniform_scan(0.0, 0.1, 5, 1.0);
  scan.range_max = 30.0;
  scan.ranges[1] = kNanF;
  scan.ranges[2] = -kInfF;
  scan.ranges[3] = -1.0F;

  std::vector<Eigen::Vector2d> endpoints;
  project_scan_for_clearing(scan, ScanFilter{}, 5.0, endpoints);

  ASSERT_EQ(endpoints.size(), 2u);
  expect_bearing_near(endpoints[0], beam_angle(scan, 0));
  expect_bearing_near(endpoints[1], beam_angle(scan, 4));
}

TEST(ScanClearing, BeamsBeyondTheClearingRangeAreTruncatedWhileMarkingDropsThem)
{
  ScanData scan = uniform_scan(0.0, 0.1, 2, 1.0);
  scan.range_min = 0.1;
  scan.range_max = 30.0;
  scan.ranges = {0.5F, 5.0F};
  ScanFilter marking_filter;
  marking_filter.max_range = 3.0;

  std::vector<Eigen::Vector2d> marks;
  project_scan(scan, marking_filter, marks);
  std::vector<Eigen::Vector2d> endpoints;
  project_scan_for_clearing(scan, marking_filter, 3.0, endpoints);

  ASSERT_EQ(marks.size(), 1u);
  EXPECT_NEAR(marks[0].norm(), 0.5, kTol);
  ASSERT_EQ(endpoints.size(), 2u);
  EXPECT_NEAR(endpoints[0].norm(), 0.5, kTol);
  EXPECT_NEAR(endpoints[1].norm(), 3.0, kTol);
}

TEST(ScanClearing, ClearingRangeIsIndependentOfTheMarkingRange)
{
  ScanData scan = uniform_scan(0.0, 0.1, 1, 6.0);
  scan.range_max = 30.0;
  ScanFilter marking_filter;
  marking_filter.max_range = 8.0;

  std::vector<Eigen::Vector2d> endpoints;
  project_scan_for_clearing(scan, marking_filter, 3.0, endpoints);

  ASSERT_EQ(endpoints.size(), 1u);
  EXPECT_NEAR(endpoints[0].norm(), 3.0, kTol);
}

TEST(ScanClearing, SensorMaximumStillCapsTheClearingRange)
{
  ScanData scan = uniform_scan(0.0, 0.1, 1, 1.0);
  scan.range_max = 2.0;
  scan.ranges[0] = kInfF;

  std::vector<Eigen::Vector2d> endpoints;
  project_scan_for_clearing(scan, ScanFilter{}, 100.0, endpoints);

  ASSERT_EQ(endpoints.size(), 1u);
  EXPECT_NEAR(endpoints[0].norm(), 2.0, kTol);
}

TEST(ScanClearing, ReadingsBelowTheEffectiveNearLimitAreDropped)
{
  ScanData scan = uniform_scan(0.0, 0.1, 4, 1.0);
  scan.range_min = 0.25;
  scan.range_max = 30.0;
  scan.ranges = {0.125F, 0.25F, 0.2F, 0.5F};
  ScanFilter marking_filter;
  marking_filter.min_range = 0.125;

  std::vector<Eigen::Vector2d> endpoints;
  project_scan_for_clearing(scan, marking_filter, 3.0, endpoints);

  ASSERT_EQ(endpoints.size(), 2u);
  EXPECT_NEAR(endpoints[0].norm(), 0.25, kTol);
  EXPECT_NEAR(endpoints[1].norm(), 0.5, kTol);
}

TEST(ScanClearing, FilterNearLimitAppliesWhenItIsTheTighterOne)
{
  ScanData scan = uniform_scan(0.0, 0.1, 3, 1.0);
  scan.range_min = 0.125;
  scan.range_max = 30.0;
  scan.ranges = {0.25F, 0.75F, 0.2F};
  ScanFilter marking_filter;
  marking_filter.min_range = 0.5;

  std::vector<Eigen::Vector2d> endpoints;
  project_scan_for_clearing(scan, marking_filter, 3.0, endpoints);

  ASSERT_EQ(endpoints.size(), 1u);
  EXPECT_NEAR(endpoints[0].norm(), 0.75, kTol);
}

TEST(ScanClearing, ClearingRangeBelowTheNearLimitYieldsNoEndpoints)
{
  ScanData scan = uniform_scan(0.0, 0.1, 4, 1.0);
  scan.range_min = 0.5;
  scan.range_max = 30.0;

  std::vector<Eigen::Vector2d> endpoints;
  project_scan_for_clearing(scan, ScanFilter{}, 0.25, endpoints);

  EXPECT_TRUE(endpoints.empty());
}

TEST(ScanClearing, AngleSectorMatchesTheMarkingPassBeamForBeam)
{
  ScanData scan = uniform_scan(-0.5 * kPi, 0.125 * kPi, 9, 1.0);
  scan.range_min = 0.1;
  scan.range_max = 30.0;
  ScanFilter marking_filter;
  marking_filter.max_range = 5.0;
  marking_filter.angle_range = AngleRange{beam_angle(scan, 2), beam_angle(scan, 6)};

  std::vector<Eigen::Vector2d> marks;
  project_scan(scan, marking_filter, marks);
  std::vector<Eigen::Vector2d> endpoints;
  project_scan_for_clearing(scan, marking_filter, 3.0, endpoints);

  ASSERT_EQ(endpoints.size(), 5u);
  ASSERT_EQ(marks.size(), endpoints.size());
  for (std::size_t i = 0; i < endpoints.size(); ++i) {
    expect_bearing_near(endpoints[i], beam_angle(scan, i + 2));
    expect_bearing_near(marks[i], beam_angle(scan, i + 2));
  }
}

TEST(ScanClearing, AngleSectorHoldsForBeamsThatOnlyClearingKeeps)
{
  ScanData scan = uniform_scan(-0.5 * kPi, 0.125 * kPi, 9, 20.0);
  scan.range_min = 0.1;
  scan.range_max = 30.0;
  ScanFilter marking_filter;
  marking_filter.max_range = 5.0;
  marking_filter.angle_range = AngleRange{beam_angle(scan, 2), beam_angle(scan, 6)};

  std::vector<Eigen::Vector2d> marks;
  project_scan(scan, marking_filter, marks);
  std::vector<Eigen::Vector2d> endpoints;
  project_scan_for_clearing(scan, marking_filter, 3.0, endpoints);

  EXPECT_TRUE(marks.empty());
  ASSERT_EQ(endpoints.size(), 5u);
  for (std::size_t i = 0; i < endpoints.size(); ++i) {
    EXPECT_NEAR(endpoints[i].norm(), 3.0, kTol);
    expect_bearing_near(endpoints[i], beam_angle(scan, i + 2));
  }
}

TEST(ScanClearing, AngleSectorAcrossPiFiltersCorrectly)
{
  ScanData scan = uniform_scan(-kPi, 0.25 * kPi, 8, 1.0);
  scan.range_max = 30.0;
  ScanFilter marking_filter;
  marking_filter.angle_range = AngleRange{0.7 * kPi, -0.7 * kPi};

  std::vector<Eigen::Vector2d> endpoints;
  project_scan_for_clearing(scan, marking_filter, 3.0, endpoints);

  const std::vector<std::size_t> expected_beams{0, 1, 7};
  ASSERT_EQ(endpoints.size(), expected_beams.size());
  for (std::size_t i = 0; i < endpoints.size(); ++i) {
    expect_bearing_near(endpoints[i], beam_angle(scan, expected_beams[i]));
  }
}

TEST(ScanClearing, LastBeamAngleIsNotAccumulated)
{
  ScanData scan = uniform_scan(-1.57, 3.14 / 719.0, 720, 1.0);
  scan.range_max = 30.0;

  std::vector<Eigen::Vector2d> endpoints;
  project_scan_for_clearing(scan, ScanFilter{}, 3.0, endpoints);

  ASSERT_EQ(endpoints.size(), 720u);
  expect_bearing_near(endpoints.back(), scan.angle_min + 719.0 * scan.angle_increment);
}

TEST(ScanClearing, OutputBufferIsClearedNotAppendedTo)
{
  ScanData scan = uniform_scan(0.0, 0.5 * kPi, 2, 1.0);
  scan.range_max = 30.0;

  std::vector<Eigen::Vector2d> endpoints{Eigen::Vector2d{9.0, 9.0}, Eigen::Vector2d{8.0, 8.0}};
  project_scan_for_clearing(scan, ScanFilter{}, 3.0, endpoints);

  ASSERT_EQ(endpoints.size(), 2u);
  expect_point_near(endpoints[0], 1.0, 0.0);
  expect_point_near(endpoints[1], 0.0, 1.0);
}

TEST(ScanClearing, EmptyScanYieldsNoEndpoints)
{
  ScanData scan = uniform_scan(0.0, 0.1, 0, 1.0);
  scan.range_max = 30.0;

  std::vector<Eigen::Vector2d> endpoints;
  project_scan_for_clearing(scan, ScanFilter{}, 3.0, endpoints);

  EXPECT_TRUE(endpoints.empty());
}

TEST(ScanClearing, TransformOverloadMapsEveryEndpointIntoTheWorld)
{
  ScanData scan = uniform_scan(0.0, 0.5 * kPi, 4, 1.0);
  scan.range_max = 30.0;
  const Transform2D sensor_to_world(Eigen::Vector2d{2.0, -1.0}, 0.5 * kPi);

  std::vector<Eigen::Vector2d> sensor_frame;
  project_scan_for_clearing(scan, ScanFilter{}, 3.0, sensor_frame);
  std::vector<Eigen::Vector2d> world_frame;
  project_scan_for_clearing(scan, ScanFilter{}, 3.0, sensor_to_world, world_frame);

  ASSERT_EQ(world_frame.size(), sensor_frame.size());
  for (std::size_t i = 0; i < world_frame.size(); ++i) {
    const Eigen::Vector2d expected = sensor_to_world * sensor_frame[i];
    expect_point_near(world_frame[i], expected.x(), expected.y());
  }
  expect_point_near(world_frame[0], 2.0, 0.0);
}

TEST(ScanClearing, InputScanIsNotModified)
{
  ScanData scan = uniform_scan(-0.4, 0.05, 20, 1.0);
  scan.range_min = 0.1;
  scan.range_max = 30.0;
  scan.ranges[3] = kNanF;
  scan.ranges[7] = kInfF;
  scan.ranges[9] = 40.0F;
  scan.ranges[11] = 0.05F;
  const ScanData before = scan;
  ScanFilter marking_filter;
  marking_filter.min_range = 0.2;
  marking_filter.max_range = 5.0;
  marking_filter.angle_range = AngleRange{-0.3, 0.3};

  std::vector<Eigen::Vector2d> endpoints;
  project_scan_for_clearing(scan, marking_filter, 3.0, endpoints);

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
