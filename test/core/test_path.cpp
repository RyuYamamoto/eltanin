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

#include <eltanin/core/path.hpp>

#include <gtest/gtest.h>

#include <utility>
#include <vector>

namespace
{

using eltanin::Path;
using eltanin::path_length;
using eltanin::Pose2D;
using eltanin::Vec2;

constexpr double kTol = 1e-12;

}  // namespace

TEST(Path, DefaultIsEmpty)
{
  const Path path;
  EXPECT_TRUE(path.empty());
  EXPECT_EQ(path.size(), 0u);
  EXPECT_NEAR(path_length(path), 0.0, kTol);
}

TEST(Path, SinglePoseHasZeroLength)
{
  const Path path{Pose2D{Vec2{1.0, 2.0}, 0.5}};
  EXPECT_FALSE(path.empty());
  EXPECT_EQ(path.size(), 1u);
  EXPECT_NEAR(path_length(path), 0.0, kTol);
}

TEST(Path, StraightLineLength)
{
  const Path path{
    Pose2D{Vec2{0.0, 0.0}, 0.0}, Pose2D{Vec2{3.0, 0.0}, 0.0}, Pose2D{Vec2{5.0, 0.0}, 0.0}};
  EXPECT_NEAR(path_length(path), 5.0, kTol);
}

TEST(Path, PolylineLength)
{
  const Path path{
    Pose2D{Vec2{0.0, 0.0}, 0.0}, Pose2D{Vec2{3.0, 4.0}, 0.0}, Pose2D{Vec2{3.0, 0.0}, 0.0}};
  EXPECT_NEAR(path_length(path), 9.0, kTol);
}

TEST(Path, PushBackAndIndexing)
{
  Path path;
  path.push_back(Pose2D{Vec2{0.0, 0.0}, 0.0});
  path.push_back(Pose2D{Vec2{1.0, 0.0}, 0.25});
  ASSERT_EQ(path.size(), 2u);
  EXPECT_NEAR(path[1].position.x(), 1.0, kTol);
  EXPECT_NEAR(path[1].yaw, 0.25, kTol);

  path[1].yaw = 0.5;
  EXPECT_NEAR(path[1].yaw, 0.5, kTol);

  path.clear();
  EXPECT_TRUE(path.empty());
}

TEST(Path, IterationVisitsEveryPose)
{
  const Path path{
    Pose2D{Vec2{0.0, 0.0}, 0.0}, Pose2D{Vec2{1.0, 0.0}, 0.0}, Pose2D{Vec2{2.0, 0.0}, 0.0}};
  double sum_x = 0.0;
  for (const Pose2D & pose : path) {
    sum_x += pose.position.x();
  }
  EXPECT_NEAR(sum_x, 3.0, kTol);
}

TEST(Path, ConstructionFromVector)
{
  std::vector<Pose2D> poses{Pose2D{Vec2{0.0, 0.0}, 0.0}, Pose2D{Vec2{0.0, 2.0}, 0.0}};
  const Path path(std::move(poses));
  EXPECT_EQ(path.size(), 2u);
  EXPECT_NEAR(path_length(path), 2.0, kTol);
  EXPECT_EQ(path.poses().size(), 2u);
}
