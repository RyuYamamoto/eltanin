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

#include <eltanin/sim/simple_simulator.hpp>

#include <eltanin/core/differential_drive.hpp>

#include <gtest/gtest.h>

namespace
{

using eltanin::integrate_differential_drive;
using eltanin::Pose2D;
using eltanin::Twist2D;
using eltanin::sim::SimpleSimulator;
using Eigen::Vector2d;

}  // namespace

TEST(SimpleSimulator, DefaultConstructionStartsAtTheOrigin)
{
  const SimpleSimulator simulator;
  EXPECT_DOUBLE_EQ(simulator.pose().position.x(), 0.0);
  EXPECT_DOUBLE_EQ(simulator.pose().position.y(), 0.0);
  EXPECT_DOUBLE_EQ(simulator.pose().yaw, 0.0);
}

TEST(SimpleSimulator, SetPoseTeleportsThePlant)
{
  SimpleSimulator simulator;
  simulator.set_pose(Pose2D{Vector2d{1.0, -2.0}, 0.5});

  EXPECT_DOUBLE_EQ(simulator.pose().position.x(), 1.0);
  EXPECT_DOUBLE_EQ(simulator.pose().position.y(), -2.0);
  EXPECT_DOUBLE_EQ(simulator.pose().yaw, 0.5);
}

TEST(SimpleSimulator, UpdateReturnsTheStoredPose)
{
  SimpleSimulator simulator(Pose2D{Vector2d{1.0, 2.0}, 0.3});
  const Pose2D & returned = simulator.update(Twist2D{Vector2d{0.5, 0.0}, 0.4}, 0.1);

  EXPECT_EQ(&returned, &simulator.pose());
  EXPECT_DOUBLE_EQ(returned.position.x(), simulator.pose().position.x());
  EXPECT_DOUBLE_EQ(returned.yaw, simulator.pose().yaw);
}

TEST(SimpleSimulator, UpdateMatchesTheSharedIntegratorExactly)
{
  const Pose2D start{Vector2d{1.0, 2.0}, 0.3};
  const Twist2D command{Vector2d{0.5, 0.0}, 0.4};
  constexpr double DT = 0.1;

  SimpleSimulator simulator(start);
  const Pose2D plant = simulator.update(command, DT);
  const Pose2D reference = integrate_differential_drive(start, command, DT);

  EXPECT_DOUBLE_EQ(plant.position.x(), reference.position.x());
  EXPECT_DOUBLE_EQ(plant.position.y(), reference.position.y());
  EXPECT_DOUBLE_EQ(plant.yaw, reference.yaw);
}

TEST(SimpleSimulator, RepeatedUpdatesAccumulate)
{
  const Pose2D start{Vector2d::Zero(), 0.0};
  const Twist2D command{Vector2d{0.5, 0.0}, 0.4};
  constexpr double DT = 0.1;

  SimpleSimulator simulator(start);
  Pose2D reference = start;
  for (int i = 0; i < 20; ++i) {
    simulator.update(command, DT);
    reference = integrate_differential_drive(reference, command, DT);
  }

  EXPECT_DOUBLE_EQ(simulator.pose().position.x(), reference.position.x());
  EXPECT_DOUBLE_EQ(simulator.pose().position.y(), reference.position.y());
  EXPECT_DOUBLE_EQ(simulator.pose().yaw, reference.yaw);
}
