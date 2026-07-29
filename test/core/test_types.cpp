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

namespace
{

using eltanin::kPi;
using eltanin::Pose2D;
using eltanin::Transform2D;
using eltanin::Twist2D;
using eltanin::Vec2;

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
  const Pose2D pose{Vec2{1.0, 2.0}, 0.5};
  EXPECT_DOUBLE_EQ(pose.position.x(), 1.0);
  EXPECT_DOUBLE_EQ(pose.position.y(), 2.0);
  EXPECT_DOUBLE_EQ(pose.yaw, 0.5);

  const Twist2D twist{Vec2{0.3, -0.1}, 0.2};
  EXPECT_DOUBLE_EQ(twist.linear.x(), 0.3);
  EXPECT_DOUBLE_EQ(twist.linear.y(), -0.1);
  EXPECT_DOUBLE_EQ(twist.angular, 0.2);
}

TEST(Types, TransformAppliedToPoint)
{
  const Transform2D tf{Vec2{1.0, 2.0}, 0.5 * kPi};
  const Vec2 mapped = tf * Vec2{1.0, 0.0};
  EXPECT_NEAR(mapped.x(), 1.0, kTol);
  EXPECT_NEAR(mapped.y(), 3.0, kTol);
}

TEST(Types, TransformCompositionIsAssociative)
{
  const Transform2D a{Vec2{1.0, 2.0}, 0.3};
  const Transform2D b{Vec2{-0.5, 0.7}, -1.1};
  const Transform2D c{Vec2{2.0, -3.0}, 2.5};

  const Transform2D left = (a * b) * c;
  const Transform2D right = a * (b * c);

  EXPECT_NEAR(left.translation().x(), right.translation().x(), 1e-10);
  EXPECT_NEAR(left.translation().y(), right.translation().y(), 1e-10);
  EXPECT_NEAR(left.rotation(), right.rotation(), 1e-10);
}

TEST(Types, TransformCompositionMatchesSequentialApplication)
{
  const Transform2D a{Vec2{1.0, 2.0}, 0.3};
  const Transform2D b{Vec2{-0.5, 0.7}, -1.1};
  const Vec2 point{0.4, -0.2};

  const Vec2 composed = (a * b) * point;
  const Vec2 sequential = a * (b * point);
  EXPECT_NEAR(composed.x(), sequential.x(), 1e-12);
  EXPECT_NEAR(composed.y(), sequential.y(), 1e-12);
}

TEST(Types, TransformInverseRoundTrip)
{
  const Transform2D tf{Vec2{1.5, -2.5}, 1.234};
  const Transform2D identity = tf.inverse() * tf;
  EXPECT_NEAR(identity.translation().x(), 0.0, 1e-12);
  EXPECT_NEAR(identity.translation().y(), 0.0, 1e-12);
  EXPECT_NEAR(identity.rotation(), 0.0, 1e-12);

  const Vec2 point{0.7, 0.9};
  const Vec2 round_tripped = tf.inverse() * (tf * point);
  EXPECT_NEAR(round_tripped.x(), point.x(), 1e-12);
  EXPECT_NEAR(round_tripped.y(), point.y(), 1e-12);
}

TEST(Types, TransformInverseAtPiRotation)
{
  const Transform2D tf{Vec2{1.0, 0.0}, kPi};
  const Transform2D identity = tf.inverse() * tf;
  EXPECT_NEAR(identity.translation().norm(), 0.0, 1e-12);
  EXPECT_NEAR(identity.rotation(), 0.0, 1e-12);
}

TEST(Types, PoseTransformRoundTrip)
{
  const Pose2D pose{Vec2{3.0, -1.0}, 0.75};
  const Pose2D recovered = Transform2D::from_pose(pose).to_pose();
  EXPECT_NEAR(recovered.position.x(), pose.position.x(), kTol);
  EXPECT_NEAR(recovered.position.y(), pose.position.y(), kTol);
  EXPECT_NEAR(recovered.yaw, pose.yaw, kTol);
}

TEST(Types, TransformAppliedToPoseRotatesOrientation)
{
  const Transform2D tf{Vec2{0.0, 0.0}, 0.5 * kPi};
  const Pose2D pose{Vec2{1.0, 0.0}, 0.25 * kPi};
  const Pose2D mapped = tf * pose;
  EXPECT_NEAR(mapped.position.x(), 0.0, kTol);
  EXPECT_NEAR(mapped.position.y(), 1.0, kTol);
  EXPECT_NEAR(mapped.yaw, 0.75 * kPi, kTol);
}

TEST(Types, RepeatedCompositionKeepsRotationNormalized)
{
  Transform2D accumulated;
  const Transform2D step{Vec2{0.1, 0.0}, 1.0};
  for (int i = 0; i < 1000; ++i) {
    accumulated = accumulated * step;
    ASSERT_GT(accumulated.rotation(), -kPi);
    ASSERT_LE(accumulated.rotation(), kPi);
  }
}
