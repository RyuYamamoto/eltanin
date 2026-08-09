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

#ifndef ELTANIN__CONTROL__FOLLOWER_FACTORY_HPP_
#define ELTANIN__CONTROL__FOLLOWER_FACTORY_HPP_

#include <eltanin/control/path_follower.hpp>
#include <eltanin/control/pure_pursuit.hpp>

#ifdef ELTANIN_WITH_MPC
#include <eltanin/control/mpc_follower.hpp>
#endif

#include <cassert>
#include <memory>
#include <utility>

namespace eltanin::control
{

/// Why no follower came back; MpcNotBuilt is what tells a caller its build option is off.
enum class FollowerError
{
  None,
  UnknownType,
  InvalidParams,
  MpcNotBuilt
};

constexpr const char * to_string(FollowerError error) noexcept
{
  switch (error) {
    case FollowerError::None:
      return "none";
    case FollowerError::UnknownType:
      return "unknown follower type";
    case FollowerError::InvalidParams:
      return "invalid follower parameters";
    case FollowerError::MpcNotBuilt:
      return "mpc not built: configure with ELTANIN_ENABLE_MPC=ON";
  }
  return "unknown";
}

/// Every follower's parameters in one place, so the caller picks a type without picking a struct.
struct FollowerFactoryParams
{
  FollowerType type{FollowerType::PurePursuit};
  PurePursuitParams pure_pursuit{};
#ifdef ELTANIN_WITH_MPC
  MpcFollowerParams mpc{};
#endif
};

/// A follower or the reason there is none; the same surface planner::PlanResult offers.
class FollowerResult
{
public:
  explicit FollowerResult(std::unique_ptr<PathFollower> follower)
  : follower_(std::move(follower))
  {
    assert(follower_ != nullptr && "a successful FollowerResult needs a follower");
  }

  explicit FollowerResult(FollowerError error) : error_(error)
  {
    assert(error != FollowerError::None && "a failed FollowerResult needs a reason");
  }

  [[nodiscard]] bool has_value() const noexcept { return follower_ != nullptr; }

  explicit operator bool() const noexcept { return has_value(); }

  /// Precondition: has_value().
  PathFollower & operator*() const
  {
    assert(has_value());
    return *follower_;
  }

  /// Precondition: has_value().
  PathFollower * operator->() const
  {
    assert(has_value());
    return follower_.get();
  }

  /// FollowerError::None on success.
  [[nodiscard]] FollowerError error() const noexcept { return error_; }

  /// Hands ownership to the caller; the result is empty afterwards.
  std::unique_ptr<PathFollower> take() noexcept { return std::move(follower_); }

private:
  std::unique_ptr<PathFollower> follower_;
  FollowerError error_{FollowerError::None};
};

/// Builds the follower `params.type` names, or says why it could not.
FollowerResult make_path_follower(const FollowerFactoryParams & params);

}  // namespace eltanin::control

#endif  // ELTANIN__CONTROL__FOLLOWER_FACTORY_HPP_
