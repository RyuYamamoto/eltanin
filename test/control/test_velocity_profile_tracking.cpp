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

#include <eltanin/control/pure_pursuit.hpp>

#include <control/tracking_fixture.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/core/types.hpp>

#include <gtest/gtest.h>

#include <cassert>
#include <cstddef>
#include <numbers>

namespace
{

using eltanin::Path;
using eltanin::path_length;
using eltanin::Pose2D;
using eltanin::control::PurePursuit;
using eltanin::control::PurePursuitParams;
using eltanin::control::VelocityProfileParams;
using eltanin_test::make_arc_path;
using eltanin_test::PurePursuitDriver;
using eltanin_test::simulate;
using eltanin_test::SIMULATION_DT;
using eltanin_test::TrackingResult;

/// Above the 0.5 m/s default, so the geometry alone can no longer hold a tight arc.
constexpr double FAST_LINEAR_VEL = 0.8;

/// The arcs here are one to six metres long, so the 1 m default gate would measure too little.
constexpr double GATE = 0.2;

/// The runaway cases orbit forever; 50 s of simulated time is far past the point being made.
constexpr std::size_t MAX_STEPS = 5000;

PurePursuitParams fast_params(bool with_profile)
{
  PurePursuitParams params;
  params.desired_linear_vel = FAST_LINEAR_VEL;
  if (with_profile) {
    VelocityProfileParams profile;
    profile.max_linear_vel = FAST_LINEAR_VEL;
    params.velocity_profile = profile;
  }
  return params;
}

TrackingResult run(const Path & path, bool with_profile)
{
  const auto tracker = PurePursuit::create(fast_params(with_profile));
  assert(tracker.has_value());
  const Pose2D start{path[0].position, path[0].yaw};
  return simulate(
    PurePursuitDriver(*tracker, path), path, start, SIMULATION_DT, GATE, MAX_STEPS);
}

}  // namespace

TEST(VelocityProfileTracking, TheProfileStopsTheRunawayOnArcsTheBodyCannotHoldAtSpeed)
{
  // At 0.8 m/s these radii need 2.0 and 1.6 rad/s, well past the 1.0 rad/s budget.
  for (const double radius : {0.4, 0.5}) {
    const Path path = make_arc_path(radius, std::numbers::pi, 0.05, 1.0);
    const TrackingResult without = run(path, false);
    const TrackingResult with = run(path, true);

    EXPECT_GT(without.max_lateral_error_after_gate, 1.0) << "radius " << radius;
    EXPECT_GT(without.travelled, 10.0 * path_length(path)) << "radius " << radius;

    EXPECT_LT(with.max_lateral_error_after_gate, 0.6) << "radius " << radius;
    EXPECT_LT(with.travelled, 2.0 * path_length(path)) << "radius " << radius;
  }
}

TEST(VelocityProfileTracking, TheProfileCostsNothingOnArcsTheBodyCanHold)
{
  // Curvature 0.5 at 0.8 m/s needs half the angular budget, so the bound never binds.
  const Path path = make_arc_path(2.0, std::numbers::pi, 0.05, 1.0);
  const TrackingResult without = run(path, false);
  const TrackingResult with = run(path, true);

  EXPECT_LT(without.max_lateral_error_after_gate, 0.01);
  EXPECT_LT(with.max_lateral_error_after_gate, 0.01);
  EXPECT_NEAR(with.max_lateral_error_after_gate, without.max_lateral_error_after_gate, 1e-3);
}

TEST(VelocityProfileTracking, TheProfileDoesNotRescueArcsTighterThanTheLookaheadFloor)
{
  // On a 0.3 m radius the 0.3 m lookahead floor aims across the arc, and no speed bound fixes that.
  const Path path = make_arc_path(0.3, std::numbers::pi, 0.05, 1.0);
  const TrackingResult without = run(path, false);
  const TrackingResult with = run(path, true);

  EXPECT_LT(without.max_lateral_error_after_gate, 0.2);
  EXPECT_LT(with.max_lateral_error_after_gate, 0.2);
  EXPECT_GT(with.max_lateral_error_after_gate, without.max_lateral_error_after_gate);
}

TEST(VelocityProfileTracking, TheCommandNeverExceedsTheBudgets)
{
  const PurePursuitParams params = fast_params(true);
  for (const double radius : {0.3, 0.4, 0.5, 2.0}) {
    const Path path = make_arc_path(radius, std::numbers::pi, 0.05, 1.0);
    const TrackingResult with = run(path, true);
    EXPECT_LE(with.max_linear_vel, FAST_LINEAR_VEL + 1e-12) << "radius " << radius;
    EXPECT_LE(with.max_abs_angular_vel, params.max_angular_vel + 1e-12) << "radius " << radius;
  }
}

TEST(VelocityProfileTracking, TheDefaultsCarryNoProfile)
{
  PurePursuitParams params;
  EXPECT_FALSE(params.velocity_profile.has_value());
  const auto tracker = PurePursuit::create(params);
  ASSERT_TRUE(tracker.has_value());
  EXPECT_FALSE(tracker->velocity_limit_at(0.0).has_value());
}
