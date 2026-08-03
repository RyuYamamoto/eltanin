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

#ifndef ELTANIN__PLANNER__PLAN_RESULT_HPP_
#define ELTANIN__PLANNER__PLAN_RESULT_HPP_

#include <eltanin/core/path.hpp>

#include <cassert>
#include <optional>
#include <utility>

namespace eltanin::planner
{

/// Why a plan request produced no path. Planner::plan() reports these in declaration order.
enum class PlannerError
{
  None,                       ///< the request succeeded
  InvalidMap,                 ///< non-positive resolution or no cells
  StartOutsideMap,            ///< the start position is outside the map or non-finite
  GoalOutsideMap,             ///< the goal position is outside the map or non-finite
  NonFiniteYaw,               ///< the start or goal yaw is not finite
  GoalBlocked,                ///< the goal cell is not Traversability::Free; never rescued
  StartRescueFailed,          ///< no free cell inside the start search radius
  Unreachable,                ///< the search exhausted every reachable state
  ExpansionLimitReached,      ///< the search hit max_expansions
  StateSpaceTooLarge,         ///< the search state arrays exceed the allowed size
  ParamsIncompatibleWithMap,  ///< the parameters cannot make progress on this resolution
};

constexpr const char * to_string(PlannerError error) noexcept
{
  switch (error) {
    case PlannerError::None:
      return "none";
    case PlannerError::InvalidMap:
      return "invalid map";
    case PlannerError::StartOutsideMap:
      return "start outside map";
    case PlannerError::GoalOutsideMap:
      return "goal outside map";
    case PlannerError::NonFiniteYaw:
      return "non-finite yaw";
    case PlannerError::GoalBlocked:
      return "goal blocked";
    case PlannerError::StartRescueFailed:
      return "start rescue failed";
    case PlannerError::Unreachable:
      return "unreachable";
    case PlannerError::ExpansionLimitReached:
      return "expansion limit reached";
    case PlannerError::StateSpaceTooLarge:
      return "state space too large";
    case PlannerError::ParamsIncompatibleWithMap:
      return "params incompatible with map";
  }
  return "unknown";
}

/// A path or the reason there is none; the std::optional-compatible surface keeps `if (!r)` working.
class PlanResult
{
public:
  PlanResult(Path path) : path_(std::move(path)) {}

  explicit PlanResult(PlannerError error) : error_(error)
  {
    assert(error != PlannerError::None && "a failed PlanResult needs a reason");
  }

  [[nodiscard]] bool has_value() const noexcept { return path_.has_value(); }

  explicit operator bool() const noexcept { return has_value(); }

  /// Precondition: has_value().
  const Path & operator*() const
  {
    assert(has_value());
    return *path_;
  }

  /// Precondition: has_value().
  const Path * operator->() const
  {
    assert(has_value());
    return &*path_;
  }

  /// PlannerError::None on success.
  [[nodiscard]] PlannerError error() const noexcept { return error_; }

  /// For callers that hold the outcome as a std::optional<Path>.
  [[nodiscard]] const std::optional<Path> & path() const noexcept { return path_; }

private:
  std::optional<Path> path_;
  PlannerError error_{PlannerError::None};
};

}  // namespace eltanin::planner

#endif  // ELTANIN__PLANNER__PLAN_RESULT_HPP_
