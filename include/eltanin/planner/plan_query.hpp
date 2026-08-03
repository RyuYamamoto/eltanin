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

#ifndef ELTANIN__PLANNER__PLAN_QUERY_HPP_
#define ELTANIN__PLANNER__PLAN_QUERY_HPP_

#include <eltanin/core/types.hpp>
#include <eltanin/map/map_geometry.hpp>
#include <eltanin/planner/traversability_view.hpp>

namespace eltanin::planner
{

/// Input to a search core. Which members a planner reads is part of that planner's contract.
struct PlanQuery
{
  TraversabilityView grid;
  /// Rescued start cell; grid.free() holds for it.
  map::MapIndex start_index;
  /// Requested goal cell; grid.free() holds for it.
  map::MapIndex goal_index;
  /// Rescued start pose; the position moves to the cell center only when the rescue fired.
  Pose2D start;
  /// Requested goal pose, position and yaw unchanged.
  Pose2D goal;
};

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__PLAN_QUERY_HPP_
