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

#include <eltanin/collision/velocity_governor.hpp>

#include <collision/collision_fixture.hpp>

#include <gtest/gtest.h>

#include <cassert>
#include <cmath>
#include <limits>

namespace
{

using eltanin::Pose2D;
using eltanin::Twist2D;
using eltanin::collision::VelocityGovernor;
using eltanin::collision::VelocityGovernorParams;
using eltanin::collision::VelocityLimiter;
using eltanin_test::CollisionScenario;
using eltanin_test::distance_scenario;
using eltanin_test::DistanceScenario;
using eltanin_test::narrow_footprint;
using eltanin_test::raw_wall_map;
using eltanin_test::wall_scenario;
using Eigen::Vector2d;

VelocityGovernorParams narrow_params()
{
  VelocityGovernorParams params;
  params.limiter.footprint = narrow_footprint();
  return params;
}

VelocityGovernor make_governor(const VelocityGovernorParams & params)
{
  const auto governor = VelocityGovernor::create(params);
  assert(governor.has_value());
  return *governor;
}

Twist2D twist(double v, double w) { return Twist2D{Vector2d{v, 0.0}, w}; }

Pose2D pose_at_cell(int mx, int my)
{
  return Pose2D{
    Vector2d{0.05 * static_cast<double>(mx) + 0.025, 0.05 * static_cast<double>(my) + 0.025}, 0.0};
}

/// Cell 9 is close enough to the wall for the ramp to bite and far enough not to be a collision.
constexpr int NEAR_WALL_CELL = 9;
constexpr int OPEN_CELL = 20;

}  // namespace

TEST(VelocityGovernorCreate, AcceptsTheDefaultsAndRejectsABadReleaseTime)
{
  EXPECT_TRUE(VelocityGovernor::create(VelocityGovernorParams{}).has_value());
  EXPECT_DOUBLE_EQ(VelocityGovernorParams{}.release_time, 0.5);

  for (const double bad : {0.0, -0.5, std::numeric_limits<double>::infinity()}) {
    VelocityGovernorParams params;
    params.release_time = bad;
    EXPECT_FALSE(VelocityGovernor::create(params).has_value()) << "release_time=" << bad;
  }
}

TEST(VelocityGovernorCreate, RejectsWhateverTheLimiterRejects)
{
  VelocityGovernorParams params;
  params.limiter.min_proximity_scale = 0.0;
  EXPECT_FALSE(VelocityGovernor::create(params).has_value());
}

TEST(VelocityGovernorCreate, StartsFullyReleased)
{
  EXPECT_DOUBLE_EQ(make_governor(VelocityGovernorParams{}).held_scale(), 1.0);
}

TEST(VelocityGovernor, DropsImmediatelyAndReleasesAtTheConfiguredRate)
{
  const DistanceScenario scenario = distance_scenario(raw_wall_map(false), narrow_footprint());
  VelocityGovernor governor = make_governor(narrow_params());
  const Twist2D cmd_in = twist(-0.3, 0.0);

  const VelocityLimiter::Result near_wall = governor.update(
    scenario.map, scenario.model, pose_at_cell(NEAR_WALL_CELL, 11), cmd_in, 0.05);
  const double raw = near_wall.proximity_scale;

  ASSERT_LT(raw, 1.0);
  EXPECT_DOUBLE_EQ(governor.held_scale(), raw);
  EXPECT_DOUBLE_EQ(near_wall.command.linear.x(), cmd_in.linear.x() * raw);

  const VelocityLimiter::Result released =
    governor.update(scenario.map, scenario.model, pose_at_cell(OPEN_CELL, 11), cmd_in, 0.1);

  EXPECT_DOUBLE_EQ(released.proximity_scale, raw + 0.1 / narrow_params().release_time);
  EXPECT_LT(released.proximity_scale, 1.0);
  EXPECT_DOUBLE_EQ(released.command.linear.x(), cmd_in.linear.x() * released.proximity_scale);
}

TEST(VelocityGovernor, TheReleaseStopsAtTheRawScale)
{
  const DistanceScenario scenario = distance_scenario(raw_wall_map(false), narrow_footprint());
  VelocityGovernor governor = make_governor(narrow_params());
  const Twist2D cmd_in = twist(0.3, 0.0);

  governor.update(scenario.map, scenario.model, pose_at_cell(NEAR_WALL_CELL, 11), cmd_in, 0.05);
  for (int cycle = 0; cycle < 20; ++cycle) {
    governor.update(scenario.map, scenario.model, pose_at_cell(OPEN_CELL, 11), cmd_in, 0.1);
  }

  EXPECT_DOUBLE_EQ(governor.held_scale(), 1.0);
}

TEST(VelocityGovernor, ANonPositiveStepHoldsTheCurrentScale)
{
  const DistanceScenario scenario = distance_scenario(raw_wall_map(false), narrow_footprint());
  VelocityGovernor governor = make_governor(narrow_params());
  const Twist2D cmd_in = twist(-0.3, 0.0);

  const double dropped = governor
                           .update(
                             scenario.map, scenario.model, pose_at_cell(NEAR_WALL_CELL, 11), cmd_in,
                             0.05)
                           .proximity_scale;
  for (const double dt : {0.0, -1.0}) {
    const VelocityLimiter::Result held =
      governor.update(scenario.map, scenario.model, pose_at_cell(OPEN_CELL, 11), cmd_in, dt);
    EXPECT_DOUBLE_EQ(held.proximity_scale, dropped) << "dt=" << dt;
  }
}

TEST(VelocityGovernor, ResetReleasesButTheNextCycleDropsAgain)
{
  const DistanceScenario scenario = distance_scenario(raw_wall_map(false), narrow_footprint());
  VelocityGovernor governor = make_governor(narrow_params());
  const Twist2D cmd_in = twist(-0.3, 0.0);

  const double dropped = governor
                           .update(
                             scenario.map, scenario.model, pose_at_cell(NEAR_WALL_CELL, 11), cmd_in,
                             0.05)
                           .proximity_scale;
  governor.reset();
  EXPECT_DOUBLE_EQ(governor.held_scale(), 1.0);

  const VelocityLimiter::Result again = governor.update(
    scenario.map, scenario.model, pose_at_cell(NEAR_WALL_CELL, 11), cmd_in, 0.0);
  EXPECT_DOUBLE_EQ(again.proximity_scale, dropped);
}

TEST(VelocityGovernor, ACostMapNeverEngagesTheHysteresis)
{
  const CollisionScenario scenario = wall_scenario(false);
  VelocityGovernor governor = make_governor(VelocityGovernorParams{});

  const VelocityLimiter::Result result =
    governor.update(scenario.map, scenario.model, pose_at_cell(15, 11), twist(-0.5, 0.0), 0.05);

  EXPECT_FALSE(result.clearance.has_value());
  EXPECT_DOUBLE_EQ(result.proximity_scale, 1.0);
  EXPECT_DOUBLE_EQ(governor.held_scale(), 1.0);
  EXPECT_DOUBLE_EQ(result.command.linear.x(), -std::sqrt(0.246875));
}

TEST(VelocityGovernor, TheLongitudinalLawStillStopsThroughTheGovernor)
{
  const DistanceScenario scenario = distance_scenario(raw_wall_map(false), narrow_footprint());
  VelocityGovernor governor = make_governor(narrow_params());

  const VelocityLimiter::Result result =
    governor.update(scenario.map, scenario.model, pose_at_cell(4, 11), twist(-0.3, 0.0), 0.05);

  EXPECT_TRUE(result.has_collision);
  EXPECT_DOUBLE_EQ(result.command.linear.x(), 0.0);
}
