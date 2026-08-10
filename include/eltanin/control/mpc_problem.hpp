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

#ifndef ELTANIN__CONTROL__MPC_PROBLEM_HPP_
#define ELTANIN__CONTROL__MPC_PROBLEM_HPP_

#include <eltanin/control/mpc_follower.hpp>
#include <eltanin/control/qp_solver.hpp>
#include <eltanin/core/path.hpp>
#include <eltanin/core/types.hpp>

#include <cstddef>
#include <limits>
#include <span>
#include <vector>

namespace eltanin::control::detail
{

/// Where on the path the robot is, measured along the polyline rather than snapped to a pose.
struct PathProjection
{
  /// Index of the segment start; monotone across calls so a loop cannot send progress backwards.
  std::size_t index{0};
  double arc{0.0};
};

/// Pose at `arc` along the polyline; the ends are clamped and the yaw takes the shortest rotation.
Pose2D pose_at_arc(const Path & path, std::span<const double> arc_lengths, double arc);

/// Nearest point of any segment between `from` and `to`; ties keep the earlier segment.
PathProjection project_on_path(
  const Path & path, std::span<const double> arc_lengths, const Eigen::Vector2d & position,
  std::size_t from, std::size_t to = std::numeric_limits<std::size_t>::max());

/// What the horizon is asked to follow: N+1 states and the N inputs that connect them.
struct MpcReference
{
  std::vector<Pose2D> states;
  std::vector<double> arc;
  std::vector<double> linear_vel;
  std::vector<double> angular_vel;

  void resize(int horizon);
};

/// The stretch of path the horizon may consume: the reference stops dead at `end_arc`.
struct ReferenceRun
{
  double end_arc{std::numeric_limits<double>::infinity()};
  Direction direction{Direction::Forward};
};

/// Fills the horizon by walking the path at the reference speed; a null profile means max speed.
void build_reference(
  const Path & path, std::span<const double> arc_lengths, const PathProjection & start,
  double prediction_dt, double run_speed, const VelocityProfile * profile, const ReferenceRun & run,
  MpcReference & reference);

/// The sparse QP of §4.2: states and inputs are both variables, dynamics are equality rows.
class MpcProblem
{
public:
  explicit MpcProblem(const MpcFollowerParams & params);

  [[nodiscard]] const QpStructure & structure() const noexcept { return structure_; }

  [[nodiscard]] int horizon() const noexcept { return horizon_; }

  [[nodiscard]] std::size_t state_index(int step) const noexcept
  {
    return static_cast<std::size_t>(3 * step);
  }

  [[nodiscard]] std::size_t input_index(int step) const noexcept
  {
    return static_cast<std::size_t>(3 * (horizon_ + 1) + 2 * step);
  }

  /// Rewrites every value array in place; `measured` must already sit inside the input box.
  void update(const MpcReference & reference, const Pose2D & robot, const Twist2D & measured);

  [[nodiscard]] std::span<const double> p_values() const noexcept { return p_values_; }
  [[nodiscard]] std::span<const double> q() const noexcept { return q_; }
  [[nodiscard]] std::span<const double> a_values() const noexcept { return a_values_; }
  [[nodiscard]] std::span<const double> lower() const noexcept { return lower_; }
  [[nodiscard]] std::span<const double> upper() const noexcept { return upper_; }

private:
  void build_structure();

  MpcFollowerParams params_{};
  int horizon_{0};
  QpStructure structure_{};
  std::vector<double> p_values_;
  std::vector<double> q_;
  std::vector<double> a_values_;
  std::vector<double> lower_;
  std::vector<double> upper_;
};

}  // namespace eltanin::control::detail

#endif  // ELTANIN__CONTROL__MPC_PROBLEM_HPP_
