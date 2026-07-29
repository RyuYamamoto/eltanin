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

#ifndef ELTANIN__MAP__COST_VALUES_HPP_
#define ELTANIN__MAP__COST_VALUES_HPP_

#include <cstdint>

namespace eltanin::map
{

inline constexpr std::uint8_t FREE_SPACE = 0;
inline constexpr std::uint8_t MAX_NON_OBSTACLE = 252;
inline constexpr std::uint8_t INSCRIBED_INFLATED_OBSTACLE = 253;
inline constexpr std::uint8_t LETHAL_OBSTACLE = 254;
inline constexpr std::uint8_t NO_INFORMATION = 255;

}  // namespace eltanin::map

#endif  // ELTANIN__MAP__COST_VALUES_HPP_
