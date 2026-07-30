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

#include <eltanin/core/angle.hpp>
#include <eltanin/core/footprint.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/grid_map.hpp>
#include <eltanin/map_io/map_loader.hpp>
#include <eltanin/planner/astar_planner.hpp>
#include <eltanin/planner/path_smoother.hpp>
#include <eltanin/sensor/scan_projection.hpp>

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

  const auto radii = eltanin::CollisionRadii::from_radii(0.3, 0.5, 1.0);
  const auto model = eltanin::map::InflationCostModel::create(*radii, 3.0);
  const eltanin::map::CostTraversabilityModel traversability(model->circumscribed_cost());

  const eltanin::sensor::ScanData scan{0.0, 1.0, 0.1, 10.0, {1.0F, 2.0F, 3.0F}};
  std::vector<Eigen::Vector2d> scan_points;
  eltanin::sensor::project_scan(scan, eltanin::sensor::ScanFilter{}, scan_points);

  const eltanin::map::Costmap free_costmap(geometry, eltanin::map::FREE_SPACE);
  const eltanin::Pose2D start{geometry.map_to_world(0, 0), 0.0};
  const eltanin::Pose2D goal{geometry.map_to_world(19, 9), 0.5};
  const auto path = eltanin::planner::plan(free_costmap, traversability, start, goal);
  if (!path.has_value()) {
    std::cerr << "plan unexpectedly failed\n";
    return EXIT_FAILURE;
  }
  const eltanin::Path smoothed =
    eltanin::planner::smooth(*path, free_costmap, traversability);

  std::cout << "cell " << index->x << "," << index->y << " center (" << center.x() << ", "
            << center.y() << ") cost " << static_cast<int>(costmap(index->x, index->y))
            << " normalized angle " << eltanin::normalize_angle(7.0)
            << " classification " << static_cast<int>(traversability.classify(costmap[0]))
            << " error kind count "
            << static_cast<int>(eltanin::map_io::MapIoErrorKind::WriteFailed)
            << " scan points " << scan_points.size() << " path poses " << path->size()
            << " smoothed length " << eltanin::path_length(smoothed) << '\n';
  return EXIT_SUCCESS;
}
