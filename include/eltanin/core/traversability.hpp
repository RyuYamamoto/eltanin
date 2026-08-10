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

#ifndef ELTANIN__CORE__TRAVERSABILITY_HPP_
#define ELTANIN__CORE__TRAVERSABILITY_HPP_

#include <concepts>

namespace eltanin
{

/// Named after the nav2 distance bands the cell falls into.
enum class Traversability
{
  Free,           ///< no collision at any heading
  Circumscribed,  ///< a collision is possible depending on the heading
  Inscribed       ///< a collision at every heading
};

/// Contract shared by every traversability model: classify one cell value.
template <class Model, class Cell>
concept TraversabilityModel = requires(const Model & model, Cell cell) {
  { model.classify(cell) } -> std::same_as<Traversability>;
};

/// Contract for models that also answer "is this cell occupied", used by the exact footprint check.
template <class Model, class Cell>
concept ObstacleModel = requires(const Model & model, Cell cell) {
  { model.is_obstacle(cell) } -> std::same_as<bool>;
};

/// Optional contract for models whose cell value carries the obstacle distance [m] itself.
template <class Model, class Cell>
concept ClearanceModel = requires(const Model & model, Cell cell) {
  { model.clearance(cell) } -> std::same_as<double>;
};

}  // namespace eltanin

#endif  // ELTANIN__CORE__TRAVERSABILITY_HPP_
