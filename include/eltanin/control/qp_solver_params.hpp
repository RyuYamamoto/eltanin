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

#ifndef ELTANIN__CONTROL__QP_SOLVER_PARAMS_HPP_
#define ELTANIN__CONTROL__QP_SOLVER_PARAMS_HPP_

namespace eltanin::control
{

/// Solver knobs the MPC exposes; the backend itself stays out of the public headers.
struct QpSolverParams
{
  /// Iteration cap; the follower falls back rather than commanding an unconverged solution.
  int max_iterations{200};
  /// Absolute residual tolerance; 1e-3 is coarse for a command in m/s.
  double eps_abs{1e-4};
  double eps_rel{1e-4};
  /// Iterations between penalty adaptations; the interval is fixed so the run stays reproducible.
  int adaptive_rho_interval{50};
  /// Starts from the shifted previous solution; reset() drops it so history cannot leak in.
  bool warm_start{true};
  /// Refinement pass after convergence; off until the tolerances prove too coarse.
  bool polish{false};
};

}  // namespace eltanin::control

#endif  // ELTANIN__CONTROL__QP_SOLVER_PARAMS_HPP_
