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

#ifndef ELTANIN__CORE__GEOMETRY_HPP_
#define ELTANIN__CORE__GEOMETRY_HPP_

#include <eltanin/core/types.hpp>

namespace eltanin
{

/// Closest point of segment [a, b] to `p`; parameter is clamped so the result is on the segment.
Vec2 closest_point_on_segment(const Vec2 & p, const Vec2 & a, const Vec2 & b);

/// Distance from `p` to segment [a, b]. A degenerate segment (a == b) measures to `a`.
double distance_to_segment(const Vec2 & p, const Vec2 & a, const Vec2 & b);

}  // namespace eltanin

#endif  // ELTANIN__CORE__GEOMETRY_HPP_
