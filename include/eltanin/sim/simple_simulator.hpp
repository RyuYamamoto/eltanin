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

#ifndef ELTANIN__SIM__SIMPLE_SIMULATOR_HPP_
#define ELTANIN__SIM__SIMPLE_SIMULATOR_HPP_

#include <eltanin/core/types.hpp>

namespace eltanin::sim
{

/// Deterministic differential-drive plant; carries no noise, covariance, joint states or transforms.
class SimpleSimulator
{
public:
  SimpleSimulator() = default;

  explicit SimpleSimulator(const Pose2D & initial_pose) : pose_(initial_pose) {}

  /// Integrates one step with integrate_differential_drive(). Precondition: dt > 0.
  const Pose2D & update(const Twist2D & command, double dt);

  const Pose2D & pose() const noexcept { return pose_; }

  void set_pose(const Pose2D & pose) noexcept { pose_ = pose; }

private:
  Pose2D pose_{};
};

}  // namespace eltanin::sim

#endif  // ELTANIN__SIM__SIMPLE_SIMULATOR_HPP_
