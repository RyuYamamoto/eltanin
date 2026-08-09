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

#include <navigation_loop.hpp>

#include <eltanin/map/cost_values.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/map/layered_costmap.hpp>
#include <eltanin/map/layers/static_layer.hpp>
#include <eltanin/map_io/pgm.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iterator>
#include <optional>
#include <string>

namespace
{

using eltanin::Pose2D;
using eltanin::map::Costmap;
using eltanin::map::MapGeometry;
using eltanin_examples::NavigateConfig;
using eltanin_examples::NavigateOutcome;
using eltanin_examples::NavigateResult;
using eltanin_examples::RobotModel;
using eltanin_examples::Sample;

/// Half width that engulfs the automatic start, so no replan can get the robot moving [cells].
constexpr int BLOCKING_HALF_WIDTH_CELLS = 20;

/// Fraction that puts that block over the start rather than ahead of it.
constexpr double BLOCKING_FRACTION = 0.02;

/// Simulated time the determinism case runs for [s]; the full run is covered by the other cases.
constexpr double DETERMINISM_SIM_TIME = 20.0;

std::filesystem::path map_yaml()
{
  return std::filesystem::path(ELTANIN_TEST_MAP_DIR) / "map.yaml";
}

std::filesystem::path output_dir(const std::string & name)
{
  return std::filesystem::path(ELTANIN_TEST_TMP_DIR) / name;
}

std::string read_file(const std::filesystem::path & file)
{
  std::ifstream in(file, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

std::string first_line(const std::filesystem::path & file)
{
  std::ifstream in(file);
  std::string line;
  std::getline(in, line);
  return line;
}

bool meta_has_key(const std::filesystem::path & file, const std::string & key)
{
  std::ifstream in(file);
  std::string line;
  while (std::getline(in, line)) {
    if (line.rfind(key + " ", 0) == 0) {
      return true;
    }
  }
  return false;
}

bool same_sample(const Sample & lhs, const Sample & rhs)
{
  return lhs.leg == rhs.leg && lhs.t == rhs.t && lhs.pose.position == rhs.pose.position &&
         lhs.pose.yaw == rhs.pose.yaw && lhs.requested.linear == rhs.requested.linear &&
         lhs.requested.angular == rhs.requested.angular &&
         lhs.limited.linear == rhs.limited.linear && lhs.limited.angular == rhs.limited.angular &&
         lhs.collision_distance == rhs.collision_distance &&
         lhs.has_collision == rhs.has_collision && lhs.predicted_count == rhs.predicted_count;
}

/// Loads the reference map once: at -O0 with sanitizers the load plus inflation costs seconds.
class NavigateOnRealMap : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!std::filesystem::is_regular_file(map_yaml())) {
      return;
    }
    static_map_ = eltanin_examples::load_raw_map(map_yaml());
    robot_ = eltanin_examples::make_robot_model();
  }

  static void TearDownTestSuite()
  {
    static_map_.reset();
    robot_.reset();
  }

  static bool map_available() { return static_map_.has_value() && robot_.has_value(); }

  static const Costmap & static_map() { return *static_map_; }

  static const RobotModel & robot() { return *robot_; }

  static std::optional<Costmap> static_map_;
  static std::optional<RobotModel> robot_;
};

std::optional<Costmap> NavigateOnRealMap::static_map_;
std::optional<RobotModel> NavigateOnRealMap::robot_;

}  // namespace

TEST_F(NavigateOnRealMap, ReachesTheGoalOnTheCleanMap)
{
  if (!map_available()) {
    GTEST_SKIP() << "reference map not available at " << map_yaml();
  }
  NavigateConfig config;
  config.obstacle_fraction = 0.0;

  const NavigateResult result = eltanin_examples::navigate(static_map(), robot(), config);

  EXPECT_EQ(result.outcome, NavigateOutcome::Reached) << result.message;
  EXPECT_LE(result.final_position_error, config.goal_tolerance);
  EXPECT_EQ(result.colliding_poses, 0u);
  EXPECT_EQ(result.replans, 0u);
  EXPECT_EQ(result.legs.size(), 1u);
  EXPECT_TRUE(result.observations.empty());
  EXPECT_EQ(result.window_clamped_cycles, 0u);
  EXPECT_FALSE(result.samples.empty());
}

/// With the observation trigger the robot replans before the limiter has anything to limit.
TEST_F(NavigateOnRealMap, AvoidsAnObservedObstacleWithoutStopping)
{
  if (!map_available()) {
    GTEST_SKIP() << "reference map not available at " << map_yaml();
  }
  const NavigateConfig config;
  ASSERT_GT(config.obstacle_fraction, 0.0);
  ASSERT_TRUE(config.replan_on_blocked_path);

  const NavigateResult result = eltanin_examples::navigate(static_map(), robot(), config);

  EXPECT_EQ(result.outcome, NavigateOutcome::Reached) << result.message;
  EXPECT_LE(result.final_position_error, config.goal_tolerance);
  EXPECT_EQ(result.colliding_poses, 0u);
  EXPECT_GE(result.replans_on_blocked_path, 1u);
  EXPECT_EQ(result.replans_on_blocked_path, result.replans);
  EXPECT_FALSE(result.observations.empty());

  const bool stopped = std::any_of(
    result.samples.begin(), result.samples.end(), [](const Sample & sample) {
      return sample.limited.linear.x() == 0.0 && sample.limited.angular == 0.0;
    });
  EXPECT_FALSE(stopped) << "the robot came to a standstill although it replanned in time";
  for (const eltanin_examples::LegStats & stats : result.legs) {
    EXPECT_EQ(stats.limited_cycles, 0u) << "the limiter had to intervene";
  }
}

/// Without it the limiter is the only thing that stops the robot, which is what A-3 checks.
TEST_F(NavigateOnRealMap, StopsForAnUnknownObstacleThenReplansToTheGoal)
{
  if (!map_available()) {
    GTEST_SKIP() << "reference map not available at " << map_yaml();
  }
  NavigateConfig config;
  config.replan_on_blocked_path = false;
  ASSERT_GT(config.obstacle_fraction, 0.0);

  const NavigateResult result = eltanin_examples::navigate(static_map(), robot(), config);

  EXPECT_EQ(result.outcome, NavigateOutcome::Reached) << result.message;
  EXPECT_LE(result.final_position_error, config.goal_tolerance);
  EXPECT_EQ(result.colliding_poses, 0u);
  EXPECT_GE(result.replans, 1u);
  EXPECT_LE(result.replans, static_cast<std::size_t>(config.max_replans));
  EXPECT_EQ(result.legs.size(), result.replans + 1);
  EXPECT_FALSE(result.observations.empty());
  ASSERT_FALSE(result.legs.empty());
  EXPECT_GE(result.legs.front().limited_cycles, 1u);

  const bool stopped = std::any_of(
    result.samples.begin(), result.samples.end(), [](const Sample & sample) {
      return sample.limited.linear.x() == 0.0 && sample.limited.angular == 0.0;
    });
  EXPECT_TRUE(stopped) << "the limiter never brought the command to a full stop";
  EXPECT_EQ(result.replans_on_blocked_path, 0u);
  EXPECT_GE(result.stop_obstacle_clearance, config.limiter.collision_margin);
  EXPECT_GE(result.stop_clearance, config.limiter.collision_margin);
}

/// The stalling run is the cheap one that still has two legs, observations and a failed outcome.
TEST_F(NavigateOnRealMap, WritesEveryOutputFile)
{
  if (!map_available()) {
    GTEST_SKIP() << "reference map not available at " << map_yaml();
  }
  NavigateConfig config;
  config.obstacle_fraction = BLOCKING_FRACTION;
  config.obstacle_half_width_cells = BLOCKING_HALF_WIDTH_CELLS;
  const std::filesystem::path directory = output_dir("navigate_outputs");

  const NavigateResult result = eltanin_examples::navigate(static_map(), robot(), config);
  ASSERT_GE(result.leg_paths.size(), 2u) << result.message;
  ASSERT_FALSE(result.samples.empty());
  ASSERT_FALSE(result.observations.empty());
  ASSERT_TRUE(eltanin_examples::write_output_files(directory, config, robot(), result));

  for (const char * name :
       {"costmap.pgm", "traversed.pgm", "path.csv", "trajectory.csv", "obstacles.csv",
        "meta.txt"}) {
    EXPECT_TRUE(std::filesystem::is_regular_file(directory / name)) << name << " is missing";
  }

  EXPECT_EQ(first_line(directory / "path.csv"), "leg,index,x,y,yaw");
  EXPECT_EQ(
    first_line(directory / "trajectory.csv"),
    "t,leg,x,y,yaw,v_in,w_in,v_out,w_out,collision_distance,has_collision,predicted_poses");
  EXPECT_EQ(first_line(directory / "obstacles.csv"), "t,x,y");

  const eltanin::map_io::PgmImage costmap = eltanin::map_io::read_pgm(directory / "costmap.pgm");
  const eltanin::map_io::PgmImage traversed = eltanin::map_io::read_pgm(directory / "traversed.pgm");
  EXPECT_EQ(costmap.width, traversed.width);
  EXPECT_EQ(costmap.height, traversed.height);

  for (const char * key :
       {"resolution", "origin_x", "origin_y", "size_x", "size_y", "crop_offset_x", "crop_offset_y",
        "circumscribed_cost", "inscribed_radius", "circumscribed_radius", "inflation_radius",
        "planner",
        "control_dt", "sensor_decimation", "local_window_size", "lidar_beams", "lidar_range_max",
        "raycast_step", "prediction_steps", "prediction_time", "prediction_dt", "collision_margin",
        "max_deceleration", "goal_tolerance", "max_replans", "stop_cycles_to_replan",
        "stall_min_progress", "replan_on_blocked_path", "path_check_distance", "legs",
        "leg", "cycles", "replans", "replans_on_blocked_path", "global_updates",
        "window_clamped_cycles", "final_position_error", "colliding_poses", "outcome"}) {
    EXPECT_TRUE(meta_has_key(directory / "meta.txt", key)) << key << " is missing from meta.txt";
  }
}

TEST_F(NavigateOnRealMap, IsDeterministic)
{
  if (!map_available()) {
    GTEST_SKIP() << "reference map not available at " << map_yaml();
  }
  NavigateConfig config;
  config.obstacle_fraction = 0.0;
  config.max_sim_time = DETERMINISM_SIM_TIME;
  const std::filesystem::path first_dir = output_dir("navigate_determinism_a");
  const std::filesystem::path second_dir = output_dir("navigate_determinism_b");

  const NavigateResult first = eltanin_examples::navigate(static_map(), robot(), config);
  const NavigateResult second = eltanin_examples::navigate(static_map(), robot(), config);

  ASSERT_EQ(first.outcome, second.outcome);
  ASSERT_FALSE(first.samples.empty());
  ASSERT_EQ(first.samples.size(), second.samples.size());
  for (std::size_t i = 0; i < first.samples.size(); ++i) {
    EXPECT_TRUE(same_sample(first.samples[i], second.samples[i])) << "cycle " << i << " differs";
  }
  ASSERT_TRUE(eltanin_examples::write_output_files(first_dir, config, robot(), first));
  ASSERT_TRUE(eltanin_examples::write_output_files(second_dir, config, robot(), second));
  EXPECT_EQ(read_file(first_dir / "trajectory.csv"), read_file(second_dir / "trajectory.csv"));
}

TEST_F(NavigateOnRealMap, ReportsAStallWhenReplanningDoesNotHelp)
{
  if (!map_available()) {
    GTEST_SKIP() << "reference map not available at " << map_yaml();
  }
  NavigateConfig config;
  config.obstacle_fraction = BLOCKING_FRACTION;
  config.obstacle_half_width_cells = BLOCKING_HALF_WIDTH_CELLS;

  const NavigateResult result = eltanin_examples::navigate(static_map(), robot(), config);

  EXPECT_TRUE(result.outcome == NavigateOutcome::Stalled || result.outcome == NavigateOutcome::ReplanFailed)
    << eltanin_examples::outcome_name(result.outcome) << ": " << result.message;
  EXPECT_FALSE(result.message.empty());
  const std::size_t max_steps = static_cast<std::size_t>(config.max_sim_time / config.control_dt);
  EXPECT_LT(result.samples.size(), max_steps);
}

TEST(NavigationLoop, RejectsInvalidConfigurationWithoutStartingTheLoop)
{
  constexpr int cells = 20;
  constexpr double resolution = 0.05;
  const Costmap map(MapGeometry(cells, cells, resolution, Eigen::Vector2d::Zero()));
  const std::optional<RobotModel> robot = eltanin_examples::make_robot_model();
  ASSERT_TRUE(robot.has_value());

  NavigateConfig config;
  config.control_dt = 0.0;
  const NavigateResult result = eltanin_examples::navigate(map, *robot, config);

  EXPECT_EQ(result.outcome, NavigateOutcome::ModelFailed);
  EXPECT_FALSE(result.message.empty());
  EXPECT_TRUE(result.samples.empty());
}

/// An unreachable goal has to be reported with the failing stage, not run to the cycle limit.
TEST(NavigationLoop, ReportsAFailedPlanOnASplitMap)
{
  constexpr int cells = 160;
  constexpr double resolution = 0.05;
  Costmap split(MapGeometry(cells, cells, resolution, Eigen::Vector2d::Zero()));
  for (int my = 0; my < cells; ++my) {
    split(cells / 2, my) = eltanin::map::LETHAL_OBSTACLE;
  }
  const std::optional<RobotModel> robot = eltanin_examples::make_robot_model();
  ASSERT_TRUE(robot.has_value());

  NavigateConfig config;
  config.local_window_size = static_cast<double>(cells) * resolution / 2.0;
  config.start_goal = std::pair{
    Pose2D{Eigen::Vector2d{1.0, 4.0}, 0.0}, Pose2D{Eigen::Vector2d{7.0, 4.0}, 0.0}};

  const NavigateResult result = eltanin_examples::navigate(split, *robot, config);

  EXPECT_EQ(result.outcome, NavigateOutcome::PlanFailed);
  EXPECT_FALSE(result.message.empty());
  EXPECT_TRUE(result.samples.empty());
}

/// The window origin has to land on the static grid, or StaticLayer resamples a shifted map.
TEST(NavigationLoopWindow, SnapsTheLocalWindowOntoTheStaticGrid)
{
  constexpr int static_cells = 40;
  constexpr int window_cells = 8;
  constexpr double resolution = 0.05;
  const Eigen::Vector2d static_origin{-1.0, 2.0};

  Costmap source(MapGeometry(static_cells, static_cells, resolution, static_origin));
  for (int my = 0; my < static_cells; ++my) {
    for (int mx = 0; mx < static_cells; ++mx) {
      source(mx, my) = static_cast<std::uint8_t>((mx * 7 + my * 13) % 251);
    }
  }

  eltanin::map::LayeredCostmap window(
    MapGeometry(window_cells, window_cells, resolution, static_origin),
    eltanin::map::NO_INFORMATION);
  window.add_layer<eltanin::map::StaticLayer>(source);

  const double offsets[] = {0.0, 0.013, 0.0249, 0.025, 0.0251, 0.049};
  for (const double offset : offsets) {
    const Eigen::Vector2d robot = static_origin + Eigen::Vector2d{0.6 + offset, 0.7 + offset};
    bool clamped = true;
    const Eigen::Vector2d origin = eltanin_examples::detail::snapped_window_origin(
      source.geometry(), window_cells, robot, clamped);
    window.set_origin(origin);
    window.update();

    const auto lower_left = source.geometry().world_to_map(window.geometry().map_to_world(0, 0));
    ASSERT_TRUE(lower_left.has_value());
    EXPECT_FALSE(clamped);
    for (int my = 0; my < window_cells; ++my) {
      for (int mx = 0; mx < window_cells; ++mx) {
        EXPECT_EQ(window.costmap()(mx, my), source(lower_left->x + mx, lower_left->y + my))
          << "offset " << offset << " cell " << mx << ", " << my;
      }
    }
  }
}

/// Clamping keeps the window inside the static map even when the robot sits near an edge.
TEST(NavigationLoopWindow, ClampsTheWindowToTheStaticMap)
{
  constexpr int static_cells = 20;
  constexpr int window_cells = 8;
  constexpr double resolution = 0.05;
  const Eigen::Vector2d static_origin{0.0, 0.0};
  const Costmap source(MapGeometry(static_cells, static_cells, resolution, static_origin));

  bool clamped = false;
  const Eigen::Vector2d lower = eltanin_examples::detail::snapped_window_origin(
    source.geometry(), window_cells, Eigen::Vector2d{0.02, 0.02}, clamped);
  EXPECT_TRUE(clamped);
  EXPECT_EQ(lower, static_origin);

  clamped = false;
  const Eigen::Vector2d upper = eltanin_examples::detail::snapped_window_origin(
    source.geometry(), window_cells, Eigen::Vector2d{0.98, 0.98}, clamped);
  EXPECT_TRUE(clamped);
  const double last_origin = static_cast<double>(static_cells - window_cells) * resolution;
  EXPECT_DOUBLE_EQ(upper.x(), last_origin);
  EXPECT_DOUBLE_EQ(upper.y(), last_origin);
}

#ifdef ELTANIN_WITH_MPC
TEST_F(NavigateOnRealMap, TheMpcFollowerReachesTheGoalOnTheCleanMap)
{
  if (!map_available()) {
    GTEST_SKIP() << "reference map not available at " << map_yaml();
  }
  NavigateConfig config;
  config.obstacle_fraction = 0.0;
  config.follower.type = eltanin::control::FollowerType::Mpc;

  const NavigateResult result = eltanin_examples::navigate(static_map(), robot(), config);

  EXPECT_EQ(result.outcome, NavigateOutcome::Reached) << result.message;
  EXPECT_LE(result.final_position_error, config.goal_tolerance);
  EXPECT_EQ(result.colliding_poses, 0u);
  EXPECT_EQ(result.replans, 0u);
  EXPECT_FALSE(result.samples.empty());
}

TEST_F(NavigateOnRealMap, TheMpcFollowerIsDeterministic)
{
  if (!map_available()) {
    GTEST_SKIP() << "reference map not available at " << map_yaml();
  }
  NavigateConfig config;
  config.obstacle_fraction = 0.0;
  config.max_sim_time = DETERMINISM_SIM_TIME;
  config.follower.type = eltanin::control::FollowerType::Mpc;

  const NavigateResult first = eltanin_examples::navigate(static_map(), robot(), config);
  const NavigateResult second = eltanin_examples::navigate(static_map(), robot(), config);

  ASSERT_EQ(first.outcome, second.outcome);
  ASSERT_FALSE(first.samples.empty());
  ASSERT_EQ(first.samples.size(), second.samples.size());
  for (std::size_t i = 0; i < first.samples.size(); ++i) {
    EXPECT_TRUE(same_sample(first.samples[i], second.samples[i])) << "cycle " << i << " differs";
  }
}

TEST_F(NavigateOnRealMap, TheMpcFollowerReplansAroundAnUnknownObstacle)
{
  if (!map_available()) {
    GTEST_SKIP() << "reference map not available at " << map_yaml();
  }
  NavigateConfig config;
  config.follower.type = eltanin::control::FollowerType::Mpc;

  const NavigateResult result = eltanin_examples::navigate(static_map(), robot(), config);

  EXPECT_EQ(result.outcome, NavigateOutcome::Reached) << result.message;
  EXPECT_GE(result.replans, 1u);
  EXPECT_EQ(result.colliding_poses, 0u);
}
#endif
