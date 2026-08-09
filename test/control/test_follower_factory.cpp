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

#include <eltanin/control/follower_factory.hpp>

#include <control/tracking_fixture.hpp>
#include <eltanin/core/path.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace
{

using eltanin::Path;
using eltanin::Pose2D;
using eltanin::control::FollowerError;
using eltanin::control::FollowerFactoryParams;
using eltanin::control::FollowerResult;
using eltanin::control::FollowerState;
using eltanin::control::FollowerType;
using eltanin::control::FollowStatus;
using eltanin::control::make_path_follower;
using eltanin::control::PathFollower;
using eltanin::control::to_string;
using eltanin_test::make_straight_path;

constexpr double kDt = 0.05;

}  // namespace

TEST(FollowerFactory, PurePursuitIsTheDefaultAndItDrives)
{
  FollowerResult result = make_path_follower(FollowerFactoryParams{});
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.error(), FollowerError::None);

  const std::unique_ptr<PathFollower> follower = result.take();
  ASSERT_NE(follower, nullptr);
  const Path path = make_straight_path(2.0, 0.05);
  EXPECT_EQ(follower->follow(FollowerState{}, path, kDt).status, FollowStatus::Tracking);
}

TEST(FollowerFactory, InvalidParametersAreReportedAsSuch)
{
  FollowerFactoryParams params;
  params.pure_pursuit.desired_linear_vel = -1.0;
  const FollowerResult result = make_path_follower(params);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), FollowerError::InvalidParams);
}

TEST(FollowerFactory, TheMpcRequestSaysWhetherTheBuildCarriesIt)
{
  FollowerFactoryParams params;
  params.type = FollowerType::Mpc;
  const FollowerResult result = make_path_follower(params);

#ifdef ELTANIN_WITH_MPC
  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result.error(), FollowerError::None);
#else
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), FollowerError::MpcNotBuilt);
#endif
}

#ifdef ELTANIN_WITH_MPC
TEST(FollowerFactory, InvalidMpcParametersAreDistinctFromAMissingBuild)
{
  FollowerFactoryParams params;
  params.type = FollowerType::Mpc;
  params.mpc.prediction_horizon = 0;
  const FollowerResult result = make_path_follower(params);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), FollowerError::InvalidParams);
}

TEST(FollowerFactory, TheMpcDrivesThroughTheBasePointer)
{
  FollowerFactoryParams params;
  params.type = FollowerType::Mpc;
  FollowerResult result = make_path_follower(params);
  ASSERT_TRUE(result.has_value());

  const std::unique_ptr<PathFollower> follower = result.take();
  const Path path = make_straight_path(2.0, 0.05);
  EXPECT_EQ(follower->follow(FollowerState{}, path, kDt).status, FollowStatus::Tracking);
}
#endif

TEST(FollowerFactory, EveryErrorHasAName)
{
  for (const FollowerError error :
       {FollowerError::None, FollowerError::UnknownType, FollowerError::InvalidParams,
        FollowerError::MpcNotBuilt}) {
    EXPECT_NE(std::string(to_string(error)), "unknown");
  }
}
