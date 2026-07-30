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

#include <eltanin/core/types.hpp>

#include <gtest/gtest.h>

#include <numbers>

namespace
{

using eltanin::interpolate_angle;
using eltanin::interpolate_pose;
using eltanin::Pose2D;
using eltanin::Transform2D;
using eltanin::Twist2D;
using Eigen::Vector2d;

constexpr double kPi = std::numbers::pi;
constexpr double kTol = 1e-12;

}  // namespace

TEST(Types, DefaultConstructionIsZero)
{
  const Pose2D pose;
  EXPECT_DOUBLE_EQ(pose.position.x(), 0.0);
  EXPECT_DOUBLE_EQ(pose.position.y(), 0.0);
  EXPECT_DOUBLE_EQ(pose.yaw, 0.0);

  const Twist2D twist;
  EXPECT_DOUBLE_EQ(twist.linear.x(), 0.0);
  EXPECT_DOUBLE_EQ(twist.linear.y(), 0.0);
  EXPECT_DOUBLE_EQ(twist.angular, 0.0);

  const Transform2D tf;
  EXPECT_DOUBLE_EQ(tf.translation().x(), 0.0);
  EXPECT_DOUBLE_EQ(tf.translation().y(), 0.0);
  EXPECT_DOUBLE_EQ(tf.rotation(), 0.0);
}

TEST(Types, AggregateInitialization)
{
  const Pose2D pose{Vector2d{1.0, 2.0}, 0.5};
  EXPECT_DOUBLE_EQ(pose.position.x(), 1.0);
  EXPECT_DOUBLE_EQ(pose.position.y(), 2.0);
  EXPECT_DOUBLE_EQ(pose.yaw, 0.5);

  const Twist2D twist{Vector2d{0.3, -0.1}, 0.2};
  EXPECT_DOUBLE_EQ(twist.linear.x(), 0.3);
  EXPECT_DOUBLE_EQ(twist.linear.y(), -0.1);
  EXPECT_DOUBLE_EQ(twist.angular, 0.2);
}

TEST(Types, InterpolatePoseHitsBothEndpoints)
{
  const Pose2D from{Vector2d{1.0, 2.0}, 0.3};
  const Pose2D to{Vector2d{-3.0, 5.0}, -1.2};

  const Pose2D at_zero = interpolate_pose(from, to, 0.0);
  EXPECT_DOUBLE_EQ(at_zero.position.x(), from.position.x());
  EXPECT_DOUBLE_EQ(at_zero.position.y(), from.position.y());
  EXPECT_NEAR(at_zero.yaw, from.yaw, kTol);

  const Pose2D at_one = interpolate_pose(from, to, 1.0);
  EXPECT_DOUBLE_EQ(at_one.position.x(), to.position.x());
  EXPECT_DOUBLE_EQ(at_one.position.y(), to.position.y());
  EXPECT_NEAR(at_one.yaw, to.yaw, kTol);
}

TEST(Types, InterpolatePosePositionIsLinear)
{
  const Pose2D from{Vector2d{1.0, 2.0}, 0.0};
  const Pose2D to{Vector2d{5.0, -2.0}, 0.0};
  for (const double t : {0.0, 0.25, 0.5, 0.75, 1.0}) {
    const Pose2D pose = interpolate_pose(from, to, t);
    EXPECT_NEAR(pose.position.x(), 1.0 + 4.0 * t, kTol) << "t=" << t;
    EXPECT_NEAR(pose.position.y(), 2.0 - 4.0 * t, kTol) << "t=" << t;
  }
}

TEST(Types, InterpolatePoseYawMatchesInterpolateAngle)
{
  const Pose2D from{Vector2d{0.0, 0.0}, 3.0};
  const Pose2D to{Vector2d{1.0, 1.0}, -3.0};
  for (const double t : {0.0, 0.2, 0.5, 0.8, 1.0}) {
    EXPECT_DOUBLE_EQ(interpolate_pose(from, to, t).yaw, interpolate_angle(3.0, -3.0, t))
      << "t=" << t;
  }
}

TEST(Types, InterpolatePoseClampsRatio)
{
  const Pose2D from{Vector2d{1.0, 2.0}, 0.3};
  const Pose2D to{Vector2d{-3.0, 5.0}, -1.2};

  const Pose2D below = interpolate_pose(from, to, -2.0);
  EXPECT_DOUBLE_EQ(below.position.x(), from.position.x());
  EXPECT_DOUBLE_EQ(below.position.y(), from.position.y());
  EXPECT_DOUBLE_EQ(below.yaw, interpolate_pose(from, to, 0.0).yaw);

  const Pose2D above = interpolate_pose(from, to, 3.0);
  EXPECT_DOUBLE_EQ(above.position.x(), to.position.x());
  EXPECT_DOUBLE_EQ(above.position.y(), to.position.y());
  EXPECT_DOUBLE_EQ(above.yaw, interpolate_pose(from, to, 1.0).yaw);
}

TEST(Types, TransformAppliedToPoint)
{
  const Transform2D tf{Vector2d{1.0, 2.0}, 0.5 * kPi};
  const Vector2d mapped = tf * Vector2d{1.0, 0.0};
  EXPECT_NEAR(mapped.x(), 1.0, kTol);
  EXPECT_NEAR(mapped.y(), 3.0, kTol);
}

TEST(Types, TransformCompositionIsAssociative)
{
  const Transform2D a{Vector2d{1.0, 2.0}, 0.3};
  const Transform2D b{Vector2d{-0.5, 0.7}, -1.1};
  const Transform2D c{Vector2d{2.0, -3.0}, 2.5};

  const Transform2D left = (a * b) * c;
  const Transform2D right = a * (b * c);

  EXPECT_NEAR(left.translation().x(), right.translation().x(), 1e-10);
  EXPECT_NEAR(left.translation().y(), right.translation().y(), 1e-10);
  EXPECT_NEAR(left.rotation(), right.rotation(), 1e-10);
}

TEST(Types, TransformCompositionMatchesSequentialApplication)
{
  const Transform2D a{Vector2d{1.0, 2.0}, 0.3};
  const Transform2D b{Vector2d{-0.5, 0.7}, -1.1};
  const Vector2d point{0.4, -0.2};

  const Vector2d composed = (a * b) * point;
  const Vector2d sequential = a * (b * point);
  EXPECT_NEAR(composed.x(), sequential.x(), 1e-12);
  EXPECT_NEAR(composed.y(), sequential.y(), 1e-12);
}

TEST(Types, TransformInverseRoundTrip)
{
  const Transform2D tf{Vector2d{1.5, -2.5}, 1.234};
  const Transform2D identity = tf.inverse() * tf;
  EXPECT_NEAR(identity.translation().x(), 0.0, 1e-12);
  EXPECT_NEAR(identity.translation().y(), 0.0, 1e-12);
  EXPECT_NEAR(identity.rotation(), 0.0, 1e-12);

  const Vector2d point{0.7, 0.9};
  const Vector2d round_tripped = tf.inverse() * (tf * point);
  EXPECT_NEAR(round_tripped.x(), point.x(), 1e-12);
  EXPECT_NEAR(round_tripped.y(), point.y(), 1e-12);
}

TEST(Types, TransformInverseAtPiRotation)
{
  const Transform2D tf{Vector2d{1.0, 0.0}, kPi};
  const Transform2D identity = tf.inverse() * tf;
  EXPECT_NEAR(identity.translation().norm(), 0.0, 1e-12);
  EXPECT_NEAR(identity.rotation(), 0.0, 1e-12);
}

TEST(Types, PoseTransformRoundTrip)
{
  const Pose2D pose{Vector2d{3.0, -1.0}, 0.75};
  const Pose2D recovered = Transform2D::from_pose(pose).to_pose();
  EXPECT_NEAR(recovered.position.x(), pose.position.x(), kTol);
  EXPECT_NEAR(recovered.position.y(), pose.position.y(), kTol);
  EXPECT_NEAR(recovered.yaw, pose.yaw, kTol);
}

TEST(Types, TransformAppliedToPoseRotatesOrientation)
{
  const Transform2D tf{Vector2d{0.0, 0.0}, 0.5 * kPi};
  const Pose2D pose{Vector2d{1.0, 0.0}, 0.25 * kPi};
  const Pose2D mapped = tf * pose;
  EXPECT_NEAR(mapped.position.x(), 0.0, kTol);
  EXPECT_NEAR(mapped.position.y(), 1.0, kTol);
  EXPECT_NEAR(mapped.yaw, 0.75 * kPi, kTol);
}

TEST(Types, RepeatedCompositionKeepsRotationNormalized)
{
  Transform2D accumulated;
  const Transform2D step{Vector2d{0.1, 0.0}, 1.0};
  for (int i = 0; i < 1000; ++i) {
    accumulated = accumulated * step;
    ASSERT_GT(accumulated.rotation(), -kPi);
    ASSERT_LE(accumulated.rotation(), kPi);
  }
}
