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

#include <eltanin/control/follower_factory.hpp>

#include <memory>
#include <optional>
#include <utility>

namespace eltanin::control
{

FollowerResult make_path_follower(const FollowerFactoryParams & params)
{
  switch (params.type) {
    case FollowerType::PurePursuit: {
      std::optional<PurePursuit> follower = PurePursuit::create(params.pure_pursuit);
      if (!follower.has_value()) {
        return FollowerResult{FollowerError::InvalidParams};
      }
      return FollowerResult{std::make_unique<PurePursuit>(std::move(*follower))};
    }
    case FollowerType::Mpc: {
#ifdef ELTANIN_WITH_MPC
      std::optional<MpcFollower> follower = MpcFollower::create(params.mpc);
      if (!follower.has_value()) {
        return FollowerResult{FollowerError::InvalidParams};
      }
      return FollowerResult{std::make_unique<MpcFollower>(std::move(*follower))};
#else
      return FollowerResult{FollowerError::MpcNotBuilt};
#endif
    }
  }
  return FollowerResult{FollowerError::UnknownType};
}

}  // namespace eltanin::control
