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

#include <eltanin/collision/velocity_governor.hpp>

#include <cmath>

namespace eltanin::collision
{

std::optional<VelocityGovernor> VelocityGovernor::create(const VelocityGovernorParams & params)
{
  if (!std::isfinite(params.release_time) || params.release_time <= 0.0) {
    return std::nullopt;
  }
  const std::optional<VelocityLimiter> limiter = VelocityLimiter::create(params.limiter);
  if (!limiter.has_value()) {
    return std::nullopt;
  }

  VelocityGovernorParams normalized = params;
  normalized.limiter = limiter->params();
  return VelocityGovernor(normalized, *limiter);
}

}  // namespace eltanin::collision
