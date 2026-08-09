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

#include <eltanin/collision/velocity_limiter.hpp>
#include <eltanin/control/goal_approach.hpp>
#include <eltanin/control/pure_pursuit.hpp>
#include <eltanin/core/angle.hpp>
#include <eltanin/core/footprint.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/map_io/map_loader.hpp>
#include <eltanin/planner/astar_planner.hpp>
#include <eltanin/planner/path_smoother.hpp>
#include <eltanin/sensor/scan_projection.hpp>
#include <eltanin/sim/simple_simulator.hpp>

#include <cstdlib>
#include <iostream>
#include <vector>

int main()
{
  const eltanin::map::MapGeometry geometry(20, 10, 0.05, Eigen::Vector2d{-1.0, -0.5});
  eltanin::map::Costmap costmap(geometry, eltanin::map::FREE_SPACE);
  costmap.fill(eltanin::map::LETHAL_OBSTACLE);

  const auto index = geometry.world_to_map(Eigen::Vector2d{-0.5, -0.25});
  if (!index.has_value()) {
    std::cerr << "world_to_map unexpectedly failed\n";
    return EXIT_FAILURE;
  }
  const Eigen::Vector2d center = geometry.map_to_world(index->x, index->y);

  const auto distance_model = eltanin::DistanceTraversabilityModel::from_radii(0.3, 0.5, 1.0);
  const auto model = eltanin::map::InflationCostModel::create(*distance_model, 3.0);
  const eltanin::map::CostTraversabilityModel traversability(model->circumscribed_cost());

  const eltanin::sensor::ScanData scan{0.0, 1.0, 0.1, 10.0, {1.0F, 2.0F, 3.0F}};
  std::vector<Eigen::Vector2d> scan_points;
  eltanin::sensor::project_scan(scan, eltanin::sensor::ScanFilter{}, scan_points);

  const eltanin::map::Costmap free_costmap(geometry, eltanin::map::FREE_SPACE);
  const eltanin::Pose2D start{geometry.map_to_world(0, 0), 0.0};
  const eltanin::Pose2D goal{geometry.map_to_world(19, 9), 0.5};
  const auto path = eltanin::planner::plan_astar(free_costmap, traversability, start, goal);
  if (!path) {
    std::cerr << "plan_astar unexpectedly failed: "
              << eltanin::planner::to_string(path.error()) << '\n';
    return EXIT_FAILURE;
  }
  const eltanin::Path smoothed =
    eltanin::planner::smooth(*path, free_costmap, traversability);

  auto tracker = eltanin::control::PurePursuit::create(eltanin::control::PurePursuitParams{});
  if (!tracker.has_value()) {
    std::cerr << "PurePursuit::create unexpectedly failed\n";
    return EXIT_FAILURE;
  }
  const eltanin::control::FollowResult command =
    tracker->follow(eltanin::control::FollowerState{start}, smoothed, 0.01);

  auto approach = eltanin::control::GoalApproach::create(eltanin::control::GoalApproachParams{});
  if (!approach.has_value()) {
    std::cerr << "GoalApproach::create unexpectedly failed\n";
    return EXIT_FAILURE;
  }
  const eltanin::control::GoalApproach::Result approach_result =
    approach->compute(start, smoothed, 0.05);
  const eltanin::Twist2D approach_limited =
    eltanin::control::detail::apply_linear_limit(command.command, approach_result.linear_vel_limit);

  auto limiter =
    eltanin::collision::VelocityLimiter::create(eltanin::collision::VelocityLimiterParams{});
  if (!limiter.has_value()) {
    std::cerr << "VelocityLimiter::create unexpectedly failed\n";
    return EXIT_FAILURE;
  }
  const eltanin::Pose2D limiter_pose{geometry.map_to_world(10, 5), 0.0};
  const eltanin::collision::VelocityLimiter::Result limited =
    limiter->limit(free_costmap, traversability, limiter_pose, command.command);

  eltanin::sim::SimpleSimulator plant(limiter_pose);
  plant.update(limited.command, 0.05);

  std::cout << "cell " << index->x << "," << index->y << " center (" << center.x() << ", "
            << center.y() << ") cost " << static_cast<int>(costmap(index->x, index->y))
            << " normalized angle " << eltanin::normalize_angle(7.0)
            << " classification " << static_cast<int>(traversability.classify(costmap[0]))
            << " error kind count "
            << static_cast<int>(eltanin::map_io::MapIoErrorKind::WriteFailed)
            << " scan points " << scan_points.size() << " path poses " << path->size()
            << " smoothed length " << eltanin::path_length(smoothed) << " arc length back "
            << eltanin::cumulative_arc_length(smoothed).back() << " tracker status "
            << static_cast<int>(command.status) << " tracker angular "
            << command.command.angular << " approach state "
            << static_cast<int>(approach_result.state) << " approach remaining arc "
            << approach_result.remaining_arc << " approach limited angular "
            << approach_limited.angular << " limited linear " << limited.command.linear.x()
            << " predicted poses " << limited.predicted_poses.size() << " plant yaw "
            << plant.pose().yaw << '\n';
  return EXIT_SUCCESS;
}
