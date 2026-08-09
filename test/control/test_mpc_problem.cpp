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

#include <eltanin/control/mpc_problem.hpp>

#include <control/tracking_fixture.hpp>
#include <eltanin/control/mpc_follower.hpp>
#include <eltanin/core/angle.hpp>
#include <eltanin/core/differential_drive.hpp>
#include <eltanin/core/path.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

namespace
{

using eltanin::cumulative_arc_length;
using eltanin::integrate_differential_drive;
using eltanin::Path;
using eltanin::Pose2D;
using eltanin::shortest_angular_distance;
using eltanin::Twist2D;
using eltanin::control::MpcFollowerParams;
using eltanin::control::detail::build_reference;
using eltanin::control::detail::MpcProblem;
using eltanin::control::detail::MpcReference;
using eltanin::control::detail::PathProjection;
using eltanin::control::detail::pose_at_arc;
using eltanin::control::detail::project_on_path;
using eltanin::control::detail::QpStructure;
using eltanin_test::make_arc_path;
using eltanin_test::make_straight_path;
using Eigen::Vector2d;

constexpr double kTol = 1e-12;

MpcFollowerParams small_params()
{
  MpcFollowerParams params;
  params.prediction_horizon = 4;
  params.prediction_dt = 0.1;
  return params;
}

/// Dense row of A, read out of the compressed columns the problem hands to the solver.
std::vector<double> dense_row(const MpcProblem & problem, int row)
{
  const QpStructure & structure = problem.structure();
  std::vector<double> values(static_cast<std::size_t>(structure.variables), 0.0);
  for (int column = 0; column < structure.variables; ++column) {
    for (int k = structure.a_indptr[static_cast<std::size_t>(column)];
         k < structure.a_indptr[static_cast<std::size_t>(column) + 1]; ++k) {
      if (structure.a_indices[static_cast<std::size_t>(k)] == row) {
        values[static_cast<std::size_t>(column)] = problem.a_values()[static_cast<std::size_t>(k)];
      }
    }
  }
  return values;
}

double dot(const std::vector<double> & row, const std::vector<double> & z)
{
  double sum = 0.0;
  for (std::size_t i = 0; i < row.size(); ++i) {
    sum += row[i] * z[i];
  }
  return sum;
}

double dense_p(const MpcProblem & problem, int row, int column)
{
  const QpStructure & structure = problem.structure();
  for (int k = structure.p_indptr[static_cast<std::size_t>(column)];
       k < structure.p_indptr[static_cast<std::size_t>(column) + 1]; ++k) {
    if (structure.p_indices[static_cast<std::size_t>(k)] == row) {
      return problem.p_values()[static_cast<std::size_t>(k)];
    }
  }
  return 0.0;
}

MpcReference reference_on(
  const Path & path, const MpcFollowerParams & params, const Vector2d & robot)
{
  const std::vector<double> arc = cumulative_arc_length(path);
  const PathProjection start = project_on_path(path, arc, robot, 0);
  MpcReference reference;
  reference.resize(params.prediction_horizon);
  build_reference(
    path, arc, start, params.prediction_dt, params.max_linear_vel, nullptr, reference);
  return reference;
}

}  // namespace

TEST(MpcProblem, PoseAtArcInterpolatesAndClamps)
{
  const Path path = make_straight_path(1.0, 0.5);
  const std::vector<double> arc = cumulative_arc_length(path);

  EXPECT_NEAR(pose_at_arc(path, arc, -1.0).position.x(), 0.0, kTol);
  EXPECT_NEAR(pose_at_arc(path, arc, 0.0).position.x(), 0.0, kTol);
  EXPECT_NEAR(pose_at_arc(path, arc, 0.25).position.x(), 0.25, kTol);
  EXPECT_NEAR(pose_at_arc(path, arc, 1.0).position.x(), 1.0, kTol);
  EXPECT_NEAR(pose_at_arc(path, arc, 5.0).position.x(), 1.0, kTol);
}

TEST(MpcProblem, PoseAtArcTakesTheShortestRotation)
{
  const Path path{
    Pose2D{Vector2d{0.0, 0.0}, 3.0}, Pose2D{Vector2d{1.0, 0.0}, -3.0}};
  const std::vector<double> arc = cumulative_arc_length(path);
  const double middle = pose_at_arc(path, arc, 0.5).yaw;
  EXPECT_NEAR(std::abs(middle), std::numbers::pi, 1e-9);
}

TEST(MpcProblem, ProjectionLandsOnTheSegmentNotThePose)
{
  const Path path = make_straight_path(1.0, 0.5);
  const std::vector<double> arc = cumulative_arc_length(path);

  const PathProjection middle = project_on_path(path, arc, Vector2d{0.37, 0.2}, 0);
  EXPECT_EQ(middle.index, 0u);
  EXPECT_NEAR(middle.arc, 0.37, kTol);

  const PathProjection before = project_on_path(path, arc, Vector2d{-1.0, 0.0}, 0);
  EXPECT_NEAR(before.arc, 0.0, kTol);

  const PathProjection after = project_on_path(path, arc, Vector2d{3.0, 0.0}, 0);
  EXPECT_NEAR(after.arc, 1.0, kTol);
}

TEST(MpcProblem, ProjectionNeverGoesBackwards)
{
  const Path path = make_straight_path(1.0, 0.1);
  const std::vector<double> arc = cumulative_arc_length(path);

  const PathProjection ahead = project_on_path(path, arc, Vector2d{0.8, 0.0}, 0);
  const PathProjection behind = project_on_path(path, arc, Vector2d{0.1, 0.0}, ahead.index);
  EXPECT_GE(behind.arc, arc[ahead.index] - kTol);
}

TEST(MpcProblem, TheReferenceAdvancesByTheReferenceSpeed)
{
  const MpcFollowerParams params = small_params();
  const Path path = make_straight_path(5.0, 0.05);
  const MpcReference reference = reference_on(path, params, Vector2d{0.0, 0.0});

  ASSERT_EQ(reference.states.size(), static_cast<std::size_t>(params.prediction_horizon) + 1);
  for (int k = 0; k < params.prediction_horizon; ++k) {
    const auto index = static_cast<std::size_t>(k);
    EXPECT_NEAR(reference.linear_vel[index], params.max_linear_vel, kTol) << "step " << k;
    EXPECT_NEAR(
      reference.arc[index + 1] - reference.arc[index],
      params.max_linear_vel * params.prediction_dt, kTol)
      << "step " << k;
    EXPECT_NEAR(reference.angular_vel[index], 0.0, 1e-9) << "step " << k;
  }
}

TEST(MpcProblem, TheReferenceStopsOnTheLastPoseWhenTheHorizonRunsOff)
{
  MpcFollowerParams params = small_params();
  params.prediction_horizon = 20;
  const Path path = make_straight_path(0.5, 0.05);
  const MpcReference reference = reference_on(path, params, Vector2d{0.0, 0.0});

  const Pose2D & last = reference.states.back();
  EXPECT_NEAR(last.position.x(), 0.5, kTol);
  for (int k = 0; k < params.prediction_horizon; ++k) {
    const auto index = static_cast<std::size_t>(k);
    if (reference.arc[index] < 0.5) {
      continue;
    }
    EXPECT_NEAR(reference.linear_vel[index], 0.0, kTol) << "step " << k;
    EXPECT_NEAR(reference.angular_vel[index], 0.0, kTol) << "step " << k;
  }
}

TEST(MpcProblem, TheReferenceTurnsWithTheArc)
{
  const MpcFollowerParams params = small_params();
  const Path path = make_arc_path(2.0, std::numbers::pi, 0.05, 1.0);
  const MpcReference reference = reference_on(path, params, Vector2d{0.0, 0.0});

  for (int k = 0; k < params.prediction_horizon; ++k) {
    const auto index = static_cast<std::size_t>(k);
    EXPECT_NEAR(
      reference.angular_vel[index], reference.linear_vel[index] / 2.0, 1e-3)
      << "step " << k;
  }
}

TEST(MpcProblem, TheStructureIsAWellFormedQp)
{
  const MpcFollowerParams params = small_params();
  const MpcProblem problem(params);
  const QpStructure & structure = problem.structure();

  const int n = params.prediction_horizon;
  EXPECT_EQ(structure.variables, 3 * (n + 1) + 2 * n);
  EXPECT_EQ(structure.constraints, 3 + 3 * n + 2 * n + 2 * n);
  EXPECT_TRUE(structure.valid());
}

TEST(MpcProblem, ThePatternDoesNotMoveWhenTheValuesDo)
{
  const MpcFollowerParams params = small_params();
  MpcProblem problem(params);
  const Path straight = make_straight_path(5.0, 0.05);
  const Path curved = make_arc_path(0.5, std::numbers::pi, 0.05, -1.0);

  problem.update(
    reference_on(straight, params, Vector2d{0.0, 0.0}), Pose2D{}, Twist2D{});
  const std::vector<int> p_indices = problem.structure().p_indices;
  const std::vector<int> a_indices = problem.structure().a_indices;
  const std::size_t p_size = problem.p_values().size();
  const std::size_t a_size = problem.a_values().size();

  problem.update(
    reference_on(curved, params, Vector2d{0.1, -0.2}),
    Pose2D{Vector2d{0.1, -0.2}, 0.4}, Twist2D{Vector2d{0.3, 0.0}, 0.2});

  EXPECT_EQ(problem.structure().p_indices, p_indices);
  EXPECT_EQ(problem.structure().a_indices, a_indices);
  EXPECT_EQ(problem.p_values().size(), p_size);
  EXPECT_EQ(problem.a_values().size(), a_size);
  EXPECT_TRUE(problem.structure().valid());
}

TEST(MpcProblem, TheDynamicsRowsAcceptTheRolledOutReference)
{
  const MpcFollowerParams params = small_params();
  MpcProblem problem(params);
  const Path path = make_arc_path(1.0, std::numbers::pi, 0.05, 1.0);
  const MpcReference reference = reference_on(path, params, Vector2d{0.02, -0.03});
  const Pose2D robot{Vector2d{0.02, -0.03}, 0.05};
  problem.update(reference, robot, Twist2D{Vector2d{0.2, 0.0}, 0.1});

  const int n = params.prediction_horizon;
  std::vector<double> z(static_cast<std::size_t>(problem.structure().variables), 0.0);
  Eigen::Vector3d deviation{
    robot.position.x() - reference.states[0].position.x(),
    robot.position.y() - reference.states[0].position.y(),
    shortest_angular_distance(reference.states[0].yaw, robot.yaw)};
  for (int i = 0; i < 3; ++i) {
    z[problem.state_index(0) + static_cast<std::size_t>(i)] = deviation[i];
  }

  // With every input deviation at zero the state deviation propagates through A_k alone.
  for (int k = 0; k < n; ++k) {
    const auto index = static_cast<std::size_t>(k);
    const double yaw = reference.states[index].yaw;
    const double speed = reference.linear_vel[index];
    Eigen::Vector3d next = deviation;
    next.x() += -params.prediction_dt * speed * std::sin(yaw) * deviation.z();
    next.y() += params.prediction_dt * speed * std::cos(yaw) * deviation.z();

    const Twist2D input{Eigen::Vector2d{speed, 0.0}, reference.angular_vel[index]};
    const Pose2D rolled = integrate_differential_drive(reference.states[index], input, params.prediction_dt);
    const Pose2D & planned = reference.states[index + 1];
    next.x() += rolled.position.x() - planned.position.x();
    next.y() += rolled.position.y() - planned.position.y();
    next.z() += shortest_angular_distance(planned.yaw, rolled.yaw);

    deviation = next;
    for (int i = 0; i < 3; ++i) {
      z[problem.state_index(k + 1) + static_cast<std::size_t>(i)] = deviation[i];
    }
  }

  for (int row = 0; row < 3 + 3 * n; ++row) {
    const double value = dot(dense_row(problem, row), z);
    EXPECT_NEAR(value, problem.lower()[static_cast<std::size_t>(row)], 1e-9) << "row " << row;
    EXPECT_NEAR(value, problem.upper()[static_cast<std::size_t>(row)], 1e-9) << "row " << row;
  }
}

TEST(MpcProblem, TheInputBoxIsTheBodyLimitMinusTheReference)
{
  const MpcFollowerParams params = small_params();
  MpcProblem problem(params);
  const Path path = make_arc_path(1.0, std::numbers::pi, 0.05, 1.0);
  const MpcReference reference = reference_on(path, params, Vector2d{0.0, 0.0});
  problem.update(reference, Pose2D{}, Twist2D{});

  const int n = params.prediction_horizon;
  const auto box = static_cast<std::size_t>(3 + 3 * n);
  for (int j = 0; j < n; ++j) {
    const auto index = static_cast<std::size_t>(j);
    const auto linear = box + static_cast<std::size_t>(2 * j);
    EXPECT_NEAR(problem.lower()[linear], params.min_linear_vel - reference.linear_vel[index], kTol);
    EXPECT_NEAR(problem.upper()[linear], params.max_linear_vel - reference.linear_vel[index], kTol);

    const auto angular = linear + 1;
    EXPECT_NEAR(
      problem.lower()[angular], -params.max_angular_vel - reference.angular_vel[index], kTol);
    EXPECT_NEAR(
      problem.upper()[angular], params.max_angular_vel - reference.angular_vel[index], kTol);
  }
}

TEST(MpcProblem, TheFirstRateRowIsMeasuredAgainstTheMeasuredCommand)
{
  const MpcFollowerParams params = small_params();
  MpcProblem problem(params);
  const Path path = make_straight_path(5.0, 0.05);
  const MpcReference reference = reference_on(path, params, Vector2d{0.0, 0.0});
  const Twist2D measured{Vector2d{0.2, 0.0}, -0.1};
  problem.update(reference, Pose2D{}, measured);

  const int n = params.prediction_horizon;
  const auto rate = static_cast<std::size_t>(3 + 5 * n);
  const double linear_step = params.max_linear_accel * params.prediction_dt;
  const double drift = reference.linear_vel[0] - measured.linear.x();
  EXPECT_NEAR(problem.lower()[rate], -linear_step - drift, kTol);
  EXPECT_NEAR(problem.upper()[rate], linear_step - drift, kTol);

  const double angular_step = params.max_angular_accel * params.prediction_dt;
  const double angular_drift = reference.angular_vel[0] - measured.angular;
  EXPECT_NEAR(problem.lower()[rate + 1], -angular_step - angular_drift, kTol);
  EXPECT_NEAR(problem.upper()[rate + 1], angular_step - angular_drift, kTol);
}

TEST(MpcProblem, TheStateWeightRotatesWithTheReferenceHeading)
{
  MpcFollowerParams params = small_params();
  params.terminal_weight_scale = 1.0;
  MpcProblem problem(params);
  const Path path = make_straight_path(5.0, 0.05);
  problem.update(reference_on(path, params, Vector2d{0.0, 0.0}), Pose2D{}, Twist2D{});

  const int first = static_cast<int>(problem.state_index(1));
  EXPECT_NEAR(dense_p(problem, first, first), 2.0 * params.weight_longitudinal, 1e-9);
  EXPECT_NEAR(dense_p(problem, first, first + 1), 0.0, 1e-9);
  EXPECT_NEAR(dense_p(problem, first + 1, first + 1), 2.0 * params.weight_lateral, 1e-9);
  EXPECT_NEAR(dense_p(problem, first + 2, first + 2), 2.0 * params.weight_yaw, 1e-9);

  const Path sideways{
    Pose2D{Vector2d{0.0, 0.0}, 0.5 * std::numbers::pi},
    Pose2D{Vector2d{0.0, 5.0}, 0.5 * std::numbers::pi}};
  problem.update(reference_on(sideways, params, Vector2d{0.0, 0.0}), Pose2D{}, Twist2D{});
  EXPECT_NEAR(dense_p(problem, first, first), 2.0 * params.weight_lateral, 1e-9);
  EXPECT_NEAR(dense_p(problem, first + 1, first + 1), 2.0 * params.weight_longitudinal, 1e-9);
}

TEST(MpcProblem, TheTerminalStateCarriesTheScaledWeight)
{
  const MpcFollowerParams params = small_params();
  MpcProblem problem(params);
  const Path path = make_straight_path(5.0, 0.05);
  problem.update(reference_on(path, params, Vector2d{0.0, 0.0}), Pose2D{}, Twist2D{});

  const int terminal = static_cast<int>(problem.state_index(params.prediction_horizon));
  EXPECT_NEAR(
    dense_p(problem, terminal, terminal),
    2.0 * params.terminal_weight_scale * params.weight_longitudinal, 1e-9);
  EXPECT_NEAR(
    dense_p(problem, terminal + 2, terminal + 2),
    2.0 * params.terminal_weight_scale * params.weight_yaw, 1e-9);
}

TEST(MpcProblem, TheInputBlockCarriesTheRateCoupling)
{
  const MpcFollowerParams params = small_params();
  MpcProblem problem(params);
  const Path path = make_straight_path(5.0, 0.05);
  problem.update(reference_on(path, params, Vector2d{0.0, 0.0}), Pose2D{}, Twist2D{});

  const int n = params.prediction_horizon;
  const int first = static_cast<int>(problem.input_index(0));
  const int second = static_cast<int>(problem.input_index(1));
  EXPECT_NEAR(
    dense_p(problem, first, first),
    2.0 * (params.weight_linear_vel + 2.0 * params.weight_linear_vel_rate), 1e-9);
  EXPECT_NEAR(dense_p(problem, first, second), -2.0 * params.weight_linear_vel_rate, 1e-9);

  const int last = static_cast<int>(problem.input_index(n - 1));
  EXPECT_NEAR(
    dense_p(problem, last, last),
    2.0 * (params.weight_linear_vel + params.weight_linear_vel_rate), 1e-9);
}
