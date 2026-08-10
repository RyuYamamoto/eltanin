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

#include <eltanin/control/mpc_follower.hpp>

#include <eltanin/control/goal_approach.hpp>
#include <eltanin/control/mpc_problem.hpp>
#include <eltanin/control/qp_solver.hpp>
#include <eltanin/core/angle.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <utility>
#include <vector>

namespace eltanin::control
{

namespace
{

/// Proportional gain of the in-place alignment [1/s]; the same law GoalApproach turns with.
constexpr double YAW_ALIGN_GAIN = 2.0;

/// Below this speed the ratio scaling of the fallback has no curvature left to preserve [m/s].
constexpr double MIN_LINEAR_VEL = 1e-9;

bool weights_are_valid(const MpcFollowerParams & params)
{
  const bool states_finite = std::isfinite(params.weight_lateral) &&
                             std::isfinite(params.weight_longitudinal) &&
                             std::isfinite(params.weight_yaw) &&
                             std::isfinite(params.terminal_weight_scale);
  const bool inputs_finite =
    std::isfinite(params.weight_linear_vel) && std::isfinite(params.weight_angular_vel) &&
    std::isfinite(params.weight_linear_vel_rate) && std::isfinite(params.weight_angular_vel_rate);
  if (!states_finite || !inputs_finite) {
    return false;
  }
  if (
    params.weight_lateral < 0.0 || params.weight_longitudinal < 0.0 || params.weight_yaw < 0.0 ||
    params.terminal_weight_scale <= 0.0) {
    return false;
  }
  // The input weights have to be strictly positive, or the QP has no unique minimiser.
  return params.weight_linear_vel > 0.0 && params.weight_angular_vel > 0.0 &&
         params.weight_linear_vel_rate >= 0.0 && params.weight_angular_vel_rate >= 0.0;
}

bool limits_are_valid(const MpcFollowerParams & params)
{
  const bool all_finite =
    std::isfinite(params.prediction_dt) && std::isfinite(params.max_linear_vel) &&
    std::isfinite(params.min_linear_vel) && std::isfinite(params.max_angular_vel) &&
    std::isfinite(params.max_linear_accel) && std::isfinite(params.max_angular_accel) &&
    std::isfinite(params.yaw_tolerance) && std::isfinite(params.max_heading_error);
  if (!all_finite) {
    return false;
  }
  if (params.prediction_horizon <= 0 || params.prediction_dt <= 0.0) {
    return false;
  }
  if (params.max_linear_vel <= 0.0 || params.min_linear_vel > params.max_linear_vel) {
    return false;
  }
  // max_linear_vel bounds the speed in both directions, exactly as the robot profile declares.
  if (params.min_linear_vel < -params.max_linear_vel) {
    return false;
  }
  if (params.max_angular_vel <= 0.0 || params.max_linear_accel <= 0.0 ||
      params.max_angular_accel <= 0.0) {
    return false;
  }
  if (params.yaw_tolerance <= 0.0 || params.yaw_tolerance >= std::numbers::pi) {
    return false;
  }
  if (params.max_heading_error <= 0.0 || params.max_heading_error >= std::numbers::pi) {
    return false;
  }
  return params.max_consecutive_failures >= 0 &&
         params.yaw_tolerance <= params.max_heading_error;
}

Twist2D alignment_command(double heading_error, double max_angular_vel)
{
  return Twist2D{
    Eigen::Vector2d::Zero(),
    std::clamp(-YAW_ALIGN_GAIN * heading_error, -max_angular_vel, max_angular_vel)};
}

/// One deceleration step that keeps the commanded curvature, or a hard stop once failures pile up.
Twist2D decelerate(
  const Twist2D & current, const MpcFollowerParams & params, double dt, int failures)
{
  if (failures > params.max_consecutive_failures) {
    return Twist2D{};
  }
  const double speed = std::abs(current.linear.x());
  const double next = std::max(0.0, speed - params.max_linear_accel * dt);
  const double ratio = speed > MIN_LINEAR_VEL ? next / speed : 0.0;
  return Twist2D{
    Eigen::Vector2d{std::copysign(next, current.linear.x()), 0.0}, current.angular * ratio};
}

}  // namespace

struct MpcFollower::Impl
{
  MpcFollowerParams params{};
  detail::MpcProblem problem;
  std::unique_ptr<detail::QpSolver> solver;
  detail::MpcReference reference{};
  /// The path with the terminal heading replaced; the caller's copy is never touched.
  Path working_path{};
  std::vector<double> arc_lengths;
  std::vector<double> solution;
  std::vector<Pose2D> predicted;
  MpcPrediction observation{};
  MpcSolverStats stats{};
  MpcRunProgress run{};
  std::size_t progress{0};
  /// First and last pose of the run being tracked; a run ends at a cusp or at the goal.
  std::size_t run_first{0};
  std::size_t run_last{0};
  /// Half the spacing before the cusp: how close counts as having arrived at it [m].
  double run_end_slack{0.0};
  bool arc_valid{false};
  bool yaw_aligned{false};

  explicit Impl(const MpcFollowerParams & given)
  : params(given), problem(given), solver(detail::make_osqp_solver(problem.structure(), given.solver))
  {
    reference.resize(params.prediction_horizon);
    solution.assign(static_cast<std::size_t>(problem.structure().variables), 0.0);
    predicted.assign(static_cast<std::size_t>(params.prediction_horizon) + 1, Pose2D{});
    arc_lengths.reserve(1024);
  }

  /// Points the window at the run opening at pose `first`; pose 0 opens the first run of the path.
  void take_up_run(std::size_t first)
  {
    const std::pair<std::size_t, std::size_t> bounds = working_path.run_bounds(first);
    run_first = bounds.first;
    run_last = bounds.second;
    run.direction = working_path.direction_of(run_first);
    run.has_cusp = run_last + 1 < working_path.size();
    run.cusp_index = run.has_cusp ? run_last : 0;
    run.run_index = first == 0 ? 0 : run.run_index + 1;
    run_end_slack =
      run_last > 0 ? 0.5 * (arc_lengths[run_last] - arc_lengths[run_last - 1]) : 0.0;
  }
};

MpcFollower::MpcFollower(std::unique_ptr<Impl> impl, std::optional<VelocityProfile> profile)
: PathFollower(std::move(profile)), impl_(std::move(impl))
{
}

MpcFollower::MpcFollower(MpcFollower &&) noexcept = default;
MpcFollower & MpcFollower::operator=(MpcFollower &&) noexcept = default;
MpcFollower::~MpcFollower() = default;

std::optional<MpcFollower> MpcFollower::create(const MpcFollowerParams & params)
{
  if (!limits_are_valid(params) || !weights_are_valid(params)) {
    return std::nullopt;
  }

  std::optional<VelocityProfile> profile;
  if (params.velocity_profile.has_value()) {
    profile = VelocityProfile::create(*params.velocity_profile);
    if (!profile.has_value()) {
      return std::nullopt;
    }
  }

  auto impl = std::make_unique<Impl>(params);
  if (!impl->solver) {
    return std::nullopt;
  }
  return MpcFollower(std::move(impl), std::move(profile));
}

const MpcPrediction & MpcFollower::prediction() const noexcept { return impl_->observation; }

const MpcSolverStats & MpcFollower::solver_stats() const noexcept { return impl_->stats; }

const MpcFollowerParams & MpcFollower::params() const noexcept { return impl_->params; }

const MpcRunProgress & MpcFollower::run_progress() const noexcept { return impl_->run; }

bool MpcFollower::supports(const Path & path) const noexcept
{
  return !path.has_reverse() || impl_->params.min_linear_vel < 0.0;
}

void MpcFollower::reset_derived() noexcept
{
  impl_->progress = 0;
  impl_->run = MpcRunProgress{};
  impl_->run_first = 0;
  impl_->run_last = 0;
  impl_->run_end_slack = 0.0;
  impl_->arc_valid = false;
  impl_->yaw_aligned = false;
  impl_->stats = MpcSolverStats{};
  impl_->observation = MpcPrediction{};
  std::fill(impl_->solution.begin(), impl_->solution.end(), 0.0);
  if (impl_->solver) {
    impl_->solver->reset();
  }
}

FollowResult MpcFollower::follow_on_path(
  const FollowerState & state, const Path & path, double dt)
{
  Impl & impl = *impl_;
  const MpcFollowerParams & params = impl.params;

  if (!impl.arc_valid) {
    impl.working_path = path;
    // The last pose carries the goal orientation, which is GoalApproach's business, not a heading
    // to steer towards; the tangent of the final segment is what the body actually travels along.
    const std::size_t last = impl.working_path.size() - 1;
    const Direction final_direction = impl.working_path.direction_of(last - 1);
    const Eigen::Vector2d final_step =
      impl.working_path[last].position - impl.working_path[last - 1].position;
    if (final_step.squaredNorm() > 0.0 && final_direction != Direction::InPlace) {
      const double tangent = std::atan2(final_step.y(), final_step.x());
      impl.working_path[last].yaw = final_direction == Direction::Reverse
                                      ? normalize_angle(tangent + std::numbers::pi)
                                      : tangent;
    }
    impl.arc_lengths = cumulative_arc_length(impl.working_path);
    impl.arc_valid = true;
    impl.take_up_run(0);
  }
  const Path & route = impl.working_path;

  const double terminal_spacing =
    (route[route.size() - 1].position - route[route.size() - 2].position).norm();
  // A path that doubles back passes over its own goal, so only the final run may end the follow.
  if (
    !impl.run.has_cusp &&
    (route[route.size() - 1].position - state.pose.position).norm() <= 0.5 * terminal_spacing) {
    reset();
    return FollowResult{Twist2D{}, FollowStatus::GoalReached};
  }

  impl.progress = std::max(impl.progress, impl.run_first);
  detail::PathProjection projection = detail::project_on_path(
    route, impl.arc_lengths, state.pose.position, impl.progress, impl.run_last);
  impl.progress = projection.index;

  // A cusp is a full stop: the body gives up its speed before it is handed the next run.
  if (impl.run.has_cusp && projection.arc >= impl.arc_lengths[impl.run_last] - impl.run_end_slack) {
    const double speed = twist_of(state).linear.x();
    if (std::abs(speed) > STOPPED_SPEED) {
      const double magnitude = std::max(0.0, std::abs(speed) - params.max_linear_accel * dt);
      return FollowResult{
        Twist2D{Eigen::Vector2d{std::copysign(magnitude, speed), 0.0}, 0.0},
        FollowStatus::Tracking};
    }
    while (impl.run.has_cusp &&
           projection.arc >= impl.arc_lengths[impl.run_last] - impl.run_end_slack) {
      impl.take_up_run(impl.run_last);
      projection = detail::project_on_path(
        route, impl.arc_lengths, state.pose.position, impl.run_first, impl.run_last);
      impl.progress = projection.index;
    }
  }

  const double run_speed = impl.run.direction == Direction::Reverse
                             ? std::min(params.max_linear_vel, -params.min_linear_vel)
                             : params.max_linear_vel;
  detail::build_reference(
    route, impl.arc_lengths, projection, params.prediction_dt, run_speed, profile(),
    detail::ReferenceRun{impl.arc_lengths[impl.run_last], impl.run.direction}, impl.reference);

  const Pose2D & anchor = impl.reference.states.front();
  const double heading_error = shortest_angular_distance(anchor.yaw, state.pose.yaw);
  impl.observation.reference = impl.reference.states;
  impl.observation.predicted = std::span<const Pose2D>{};
  impl.observation.yaw_error = heading_error;
  const Eigen::Vector2d offset = state.pose.position - anchor.position;
  impl.observation.lateral_error =
    -offset.x() * std::sin(anchor.yaw) + offset.y() * std::cos(anchor.yaw);

  if (!impl.yaw_aligned) {
    if (std::abs(heading_error) <= params.yaw_tolerance) {
      impl.yaw_aligned = true;
    } else {
      return FollowResult{
        alignment_command(heading_error, params.max_angular_vel), FollowStatus::Tracking};
    }
  } else if (std::abs(heading_error) > params.max_heading_error) {
    // Past this the first-order expansion about the reference is not a model of anything.
    impl.yaw_aligned = false;
    return FollowResult{
      alignment_command(heading_error, params.max_angular_vel), FollowStatus::Tracking};
  }

  const Twist2D & measured = twist_of(state);
  const Twist2D clamped{
    Eigen::Vector2d{
      std::clamp(measured.linear.x(), params.min_linear_vel, params.max_linear_vel), 0.0},
    std::clamp(measured.angular, -params.max_angular_vel, params.max_angular_vel)};

  impl.problem.update(impl.reference, state.pose, clamped);
  const detail::QpStats stats = impl.solver->solve(
    impl.problem.p_values(), impl.problem.q(), impl.problem.a_values(), impl.problem.lower(),
    impl.problem.upper(), impl.solution);

  impl.stats.status = detail::to_string(stats.status);
  impl.stats.iterations = stats.iterations;
  impl.stats.objective = stats.objective;
  impl.stats.solve_time = stats.solve_time;

  if (stats.status != detail::QpStatus::Solved && stats.status != detail::QpStatus::SolvedInaccurate) {
    ++impl.stats.consecutive_failures;
    return FollowResult{
      decelerate(twist_of(state), params, dt, impl.stats.consecutive_failures),
      FollowStatus::SolverFailed};
  }
  impl.stats.consecutive_failures = 0;

  const std::size_t input = impl.problem.input_index(0);
  const double linear = std::clamp(
    impl.reference.linear_vel.front() + impl.solution[input], params.min_linear_vel,
    params.max_linear_vel);
  const double angular = std::clamp(
    impl.reference.angular_vel.front() + impl.solution[input + 1], -params.max_angular_vel,
    params.max_angular_vel);
  if (!std::isfinite(linear) || !std::isfinite(angular)) {
    ++impl.stats.consecutive_failures;
    return FollowResult{
      decelerate(twist_of(state), params, dt, impl.stats.consecutive_failures),
      FollowStatus::SolverFailed};
  }

  for (int k = 0; k <= params.prediction_horizon; ++k) {
    const auto index = static_cast<std::size_t>(k);
    const std::size_t base = impl.problem.state_index(k);
    const Pose2D & planned = impl.reference.states[index];
    impl.predicted[index] = Pose2D{
      planned.position + Eigen::Vector2d{impl.solution[base], impl.solution[base + 1]},
      normalize_angle(planned.yaw + impl.solution[base + 2])};
  }
  impl.observation.predicted = impl.predicted;

  const Twist2D command{Eigen::Vector2d{linear, 0.0}, angular};
  return FollowResult{
    detail::apply_linear_limit(command, limit_at_arc(projection.arc)), FollowStatus::Tracking};
}

}  // namespace eltanin::control
