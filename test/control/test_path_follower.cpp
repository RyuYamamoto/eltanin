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

#include <eltanin/control/path_follower.hpp>

#include <eltanin/control/velocity_profile.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

namespace
{

using eltanin::Path;
using eltanin::Pose2D;
using eltanin::Twist2D;
using eltanin::control::FollowerState;
using eltanin::control::FollowerType;
using eltanin::control::FollowResult;
using eltanin::control::FollowStatus;
using eltanin::control::PathFollower;
using eltanin::control::VelocityProfile;
using eltanin::control::VelocityProfileParams;
using eltanin::control::name_of;
using eltanin::control::to_follower_type;
using eltanin::control::to_string;
using Eigen::Vector2d;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kDt = 0.05;

/// Records what the base handed down, and echoes a command derived from the substituted twist.
class RecordingFollower : public PathFollower
{
public:
  RecordingFollower() = default;

  explicit RecordingFollower(const VelocityProfileParams & profile)
  : PathFollower(VelocityProfile::create(profile))
  {
  }

  using PathFollower::limit_at_arc;
  using PathFollower::limit_at_index;

  std::size_t follow_calls{0};
  std::size_t reset_calls{0};
  Twist2D seen_twist{};
  Pose2D seen_pose{};
  std::size_t seen_path_size{0};
  double seen_dt{0.0};

protected:
  FollowResult follow_on_path(
    const FollowerState & state, const Path & path, double dt) override
  {
    ++follow_calls;
    seen_twist = twist_of(state);
    seen_pose = state.pose;
    seen_path_size = path.size();
    seen_dt = dt;
    return FollowResult{
      Twist2D{Vector2d{seen_twist.linear.x() + 0.1, 0.0}, 0.0}, FollowStatus::Tracking};
  }

  void reset_derived() noexcept override { ++reset_calls; }
};

Path make_path(std::size_t poses)
{
  Path path;
  for (std::size_t i = 0; i < poses; ++i) {
    path.push_back(Pose2D{Vector2d{0.1 * static_cast<double>(i), 0.0}, 0.0});
  }
  return path;
}

/// Returns whatever it is told to, so the base guard can be examined on its own.
class ScriptedFollower : public PathFollower
{
public:
  Twist2D next_command{};
  bool accepts{true};
  std::size_t follow_calls{0};

  bool supports(const Path & path) const noexcept override
  {
    (void)path;
    return accepts;
  }

protected:
  FollowResult follow_on_path(
    const FollowerState & state, const Path & path, double dt) override
  {
    (void)state;
    (void)path;
    (void)dt;
    ++follow_calls;
    return FollowResult{next_command, FollowStatus::Tracking};
  }

  void reset_derived() noexcept override {}
};

}  // namespace

TEST(PathFollower, EmptyPathReportsNoPathAndResetsTheDerivedState)
{
  RecordingFollower follower;
  const FollowResult result = follower.follow(FollowerState{}, Path{}, kDt);

  EXPECT_EQ(result.status, FollowStatus::NoPath);
  EXPECT_DOUBLE_EQ(result.command.linear.x(), 0.0);
  EXPECT_DOUBLE_EQ(result.command.angular, 0.0);
  EXPECT_EQ(follower.follow_calls, 0u);
  EXPECT_EQ(follower.reset_calls, 1u);
}

TEST(PathFollower, SinglePosePathReportsGoalReachedAndResetsTheDerivedState)
{
  RecordingFollower follower;
  const FollowResult result = follower.follow(FollowerState{}, make_path(1), kDt);

  EXPECT_EQ(result.status, FollowStatus::GoalReached);
  EXPECT_DOUBLE_EQ(result.command.linear.x(), 0.0);
  EXPECT_EQ(follower.follow_calls, 0u);
  EXPECT_EQ(follower.reset_calls, 1u);
}

TEST(PathFollower, InvalidRuntimeInputThrows)
{
  RecordingFollower follower;
  const Path path = make_path(4);

  EXPECT_THROW(follower.follow(FollowerState{}, path, 0.0), std::invalid_argument);
  EXPECT_THROW(follower.follow(FollowerState{}, path, -kDt), std::invalid_argument);
  EXPECT_THROW(follower.follow(FollowerState{}, path, kNan), std::invalid_argument);
  EXPECT_THROW(follower.follow(FollowerState{}, path, kInf), std::invalid_argument);
  EXPECT_THROW(
    follower.follow(FollowerState{Pose2D{Vector2d{kNan, 0.0}, 0.0}}, path, kDt),
    std::invalid_argument);
  EXPECT_THROW(
    follower.follow(FollowerState{Pose2D{Vector2d{0.0, 0.0}, kInf}}, path, kDt),
    std::invalid_argument);
  EXPECT_THROW(
    follower.follow(
      FollowerState{Pose2D{}, Twist2D{Vector2d{kNan, 0.0}, 0.0}}, path, kDt),
    std::invalid_argument);
  EXPECT_THROW(
    follower.follow(FollowerState{Pose2D{}, Twist2D{Vector2d::Zero(), kNan}}, path, kDt),
    std::invalid_argument);
  EXPECT_EQ(follower.follow_calls, 0u);
}

TEST(PathFollower, ValidationRunsBeforeTheDegeneratePaths)
{
  RecordingFollower follower;
  EXPECT_THROW(follower.follow(FollowerState{}, Path{}, 0.0), std::invalid_argument);
  EXPECT_EQ(follower.reset_calls, 0u);
}

TEST(PathFollower, AbsentTwistIsSubstitutedByTheLastCommand)
{
  RecordingFollower follower;
  const Path path = make_path(4);

  static_cast<void>(follower.follow(FollowerState{}, path, kDt));
  EXPECT_DOUBLE_EQ(follower.seen_twist.linear.x(), 0.0);

  static_cast<void>(follower.follow(FollowerState{}, path, kDt));
  EXPECT_DOUBLE_EQ(follower.seen_twist.linear.x(), 0.1);

  static_cast<void>(follower.follow(FollowerState{}, path, kDt));
  EXPECT_DOUBLE_EQ(follower.seen_twist.linear.x(), 0.2);
}

TEST(PathFollower, MeasuredTwistWinsOverTheLastCommand)
{
  RecordingFollower follower;
  const Path path = make_path(4);

  static_cast<void>(follower.follow(FollowerState{}, path, kDt));
  static_cast<void>(
    follower.follow(FollowerState{Pose2D{}, Twist2D{Vector2d{0.7, 0.0}, 0.3}}, path, kDt));

  EXPECT_DOUBLE_EQ(follower.seen_twist.linear.x(), 0.7);
  EXPECT_DOUBLE_EQ(follower.seen_twist.angular, 0.3);
}

TEST(PathFollower, ResetClearsTheRememberedCommand)
{
  RecordingFollower follower;
  const Path path = make_path(4);

  static_cast<void>(follower.follow(FollowerState{}, path, kDt));
  static_cast<void>(follower.follow(FollowerState{}, path, kDt));
  follower.reset();
  static_cast<void>(follower.follow(FollowerState{}, path, kDt));

  EXPECT_DOUBLE_EQ(follower.seen_twist.linear.x(), 0.0);
  EXPECT_EQ(follower.reset_calls, 1u);
}

TEST(PathFollower, TheStateAndDtReachTheDerivedFollowerUnchanged)
{
  RecordingFollower follower;
  const Path path = make_path(6);
  const Pose2D robot{Vector2d{0.25, -0.5}, 1.2};

  static_cast<void>(follower.follow(FollowerState{robot}, path, kDt));

  EXPECT_DOUBLE_EQ(follower.seen_pose.position.x(), robot.position.x());
  EXPECT_DOUBLE_EQ(follower.seen_pose.position.y(), robot.position.y());
  EXPECT_DOUBLE_EQ(follower.seen_pose.yaw, robot.yaw);
  EXPECT_EQ(follower.seen_path_size, path.size());
  EXPECT_DOUBLE_EQ(follower.seen_dt, kDt);
}

TEST(PathFollower, FollowerTypeRoundTripsThroughItsName)
{
  for (const FollowerType type : {FollowerType::PurePursuit, FollowerType::Mpc}) {
    EXPECT_EQ(to_follower_type(name_of(type)), type);
  }
  EXPECT_FALSE(to_follower_type("teb").has_value());
  EXPECT_FALSE(to_follower_type("").has_value());
  EXPECT_FALSE(to_follower_type("PurePursuit").has_value());
}

TEST(PathFollower, EveryStatusHasAName)
{
  for (const FollowStatus status :
       {FollowStatus::NoPath, FollowStatus::Tracking, FollowStatus::GoalReached,
        FollowStatus::SolverFailed, FollowStatus::PathNotSupported}) {
    EXPECT_FALSE(std::string(to_string(status)).empty());
    EXPECT_NE(std::string(to_string(status)), "unknown");
  }
}

TEST(PathFollower, APathTheFollowerRefusesIsNeverPassedDown)
{
  ScriptedFollower follower;
  follower.accepts = false;
  const FollowResult result = follower.follow(FollowerState{}, make_path(6), kDt);

  EXPECT_EQ(result.status, FollowStatus::PathNotSupported);
  EXPECT_DOUBLE_EQ(result.command.linear.x(), 0.0);
  EXPECT_DOUBLE_EQ(result.command.angular, 0.0);
  EXPECT_EQ(follower.follow_calls, 0u);
}

TEST(PathFollower, ACommandThatWouldFlipSignInOneCycleIsReplacedByAStop)
{
  ScriptedFollower follower;
  const Path path = make_path(6);

  follower.next_command = Twist2D{Vector2d{0.4, 0.0}, 0.0};
  EXPECT_DOUBLE_EQ(follower.follow(FollowerState{}, path, kDt).command.linear.x(), 0.4);

  follower.next_command = Twist2D{Vector2d{-0.4, 0.0}, 0.7};
  const FollowResult stopped = follower.follow(FollowerState{}, path, kDt);
  EXPECT_DOUBLE_EQ(stopped.command.linear.x(), 0.0);
  // Only the linear part is held back; the body still has to turn towards the next run.
  EXPECT_DOUBLE_EQ(stopped.command.angular, 0.7);
  EXPECT_EQ(stopped.status, FollowStatus::Tracking);

  // Having passed through zero, the other direction is now allowed.
  const FollowResult reversing = follower.follow(FollowerState{}, path, kDt);
  EXPECT_DOUBLE_EQ(reversing.command.linear.x(), -0.4);
}

TEST(PathFollower, TheGuardDoesNotTouchACommandThatKeepsItsSign)
{
  ScriptedFollower follower;
  const Path path = make_path(6);

  for (const double speed : {0.4, 0.2, 0.0, 0.3}) {
    follower.next_command = Twist2D{Vector2d{speed, 0.0}, 0.0};
    EXPECT_DOUBLE_EQ(follower.follow(FollowerState{}, path, kDt).command.linear.x(), speed);
  }
}

TEST(PathFollower, TheProfileIsBuiltOnceAndRebuiltAfterReset)
{
  RecordingFollower follower{VelocityProfileParams{}};
  const Path path = make_path(40);

  EXPECT_FALSE(follower.velocity_limit_at(0.0).has_value());
  EXPECT_EQ(follower.limit_at_index(0), kInf);

  static_cast<void>(follower.follow(FollowerState{}, path, kDt));
  const std::optional<double> built = follower.velocity_limit_at(0.0);
  ASSERT_TRUE(built.has_value());
  EXPECT_DOUBLE_EQ(*built, VelocityProfileParams{}.max_linear_vel);
  EXPECT_DOUBLE_EQ(follower.limit_at_index(0), *built);

  static_cast<void>(follower.follow(FollowerState{}, path, kDt));
  EXPECT_DOUBLE_EQ(*follower.velocity_limit_at(0.0), *built);

  follower.reset();
  EXPECT_FALSE(follower.velocity_limit_at(0.0).has_value());
  static_cast<void>(follower.follow(FollowerState{}, path, kDt));
  EXPECT_TRUE(follower.velocity_limit_at(0.0).has_value());
}

TEST(PathFollower, WithoutAProfileEveryBoundIsInfinite)
{
  RecordingFollower follower;
  static_cast<void>(follower.follow(FollowerState{}, make_path(40), kDt));
  EXPECT_FALSE(follower.velocity_limit_at(0.0).has_value());
  EXPECT_EQ(follower.limit_at_index(0), kInf);
  EXPECT_EQ(follower.limit_at_arc(1.0), kInf);
}
