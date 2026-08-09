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

#ifndef ELTANIN__CONTROL__MPC_FOLLOWER_HPP_
#define ELTANIN__CONTROL__MPC_FOLLOWER_HPP_

#include <eltanin/control/path_follower.hpp>
#include <eltanin/control/qp_solver_params.hpp>
#include <eltanin/control/velocity_profile.hpp>
#include <eltanin/core/types.hpp>

#include <optional>

namespace eltanin::control
{

struct MpcFollowerParams
{
  /// Steps in the prediction horizon; 20 at 0.1 s looks 1 m ahead at the default speed.
  int prediction_horizon{20};
  /// Step of the prediction [s]; unrelated to the control period the caller passes to follow().
  double prediction_dt{0.1};
  /// Forward velocity bound [m/s]; matches PurePursuitParams::desired_linear_vel.
  double max_linear_vel{0.5};
  /// Lower velocity bound [m/s]; 0 forbids reversing, as every follower here does.
  double min_linear_vel{0.0};
  /// Symmetric bound on the commanded angular velocity [rad/s].
  double max_angular_vel{1.0};
  /// Bound on how fast the commanded velocity may change [m/s^2].
  double max_linear_accel{1.0};
  /// Same for the angular velocity [rad/s^2]; unmeasured on hardware, so this one is a guess.
  double max_angular_accel{3.0};
  /// Weight on the error across the reference heading; the quantity the tracking tests measure.
  double weight_lateral{10.0};
  /// Weight on the error along the reference heading.
  double weight_longitudinal{1.0};
  double weight_yaw{5.0};
  double weight_linear_vel{1.0};
  double weight_angular_vel{1.0};
  double weight_linear_vel_rate{10.0};
  double weight_angular_vel_rate{10.0};
  /// Multiplies the state weights on the last horizon step.
  double terminal_weight_scale{10.0};
  /// Heading error below which the initial in-place alignment ends [rad].
  double yaw_tolerance{0.07};
  /// Above this the linearisation about the reference is not trustworthy, so the body turns [rad].
  double max_heading_error{0.785};
  /// Consecutive solver failures after which the command becomes exactly zero.
  int max_consecutive_failures{3};
  QpSolverParams solver{};
  /// nullopt keeps the reference speed at max_linear_vel everywhere.
  std::optional<VelocityProfileParams> velocity_profile{};
};

}  // namespace eltanin::control

#endif  // ELTANIN__CONTROL__MPC_FOLLOWER_HPP_
