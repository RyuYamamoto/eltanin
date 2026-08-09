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

#ifndef ELTANIN_TEST__CONTROL__MPC_FIXTURE_HPP_
#define ELTANIN_TEST__CONTROL__MPC_FIXTURE_HPP_

#include <eltanin/control/mpc_follower.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/core/types.hpp>

#include <cassert>
#include <optional>
#include <utility>

namespace eltanin_test
{

/// Adapts MpcFollower to the simulate() command signature, feeding back its own last command.
class MpcDriver
{
public:
  MpcDriver(eltanin::control::MpcFollower follower, const eltanin::Path & path)
  : follower_(std::move(follower)), path_(&path)
  {
  }

  std::optional<eltanin::Twist2D> operator()(const eltanin::Pose2D & robot, double dt)
  {
    const eltanin::control::FollowResult result =
      follower_.follow(eltanin::control::FollowerState{robot, last_}, *path_, dt);
    if (result.status != eltanin::control::FollowStatus::Tracking) {
      return std::nullopt;
    }
    last_ = result.command;
    return result.command;
  }

private:
  eltanin::control::MpcFollower follower_;
  const eltanin::Path * path_;
  std::optional<eltanin::Twist2D> last_{};
};

inline eltanin::control::MpcFollower make_mpc(
  const eltanin::control::MpcFollowerParams & params =
    eltanin::control::MpcFollowerParams{})
{
  auto follower = eltanin::control::MpcFollower::create(params);
  assert(follower.has_value());
  return std::move(*follower);
}

}  // namespace eltanin_test

#endif  // ELTANIN_TEST__CONTROL__MPC_FIXTURE_HPP_
