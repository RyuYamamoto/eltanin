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

#include <qp_solver.hpp>

#include <eltanin/control/qp_solver_params.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{

using eltanin::control::QpSolverParams;
using eltanin::control::detail::make_osqp_solver;
using eltanin::control::detail::QpSolver;
using eltanin::control::detail::QpStats;
using eltanin::control::detail::QpStatus;
using eltanin::control::detail::QpStructure;
using eltanin::control::detail::to_string;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

/// The reference problem from the OSQP documentation; its optimum is (0.3, 0.7).
QpStructure reference_structure()
{
  QpStructure structure;
  structure.variables = 2;
  structure.constraints = 3;
  structure.p_indptr = {0, 1, 3};
  structure.p_indices = {0, 0, 1};
  structure.a_indptr = {0, 2, 4};
  structure.a_indices = {0, 1, 0, 2};
  return structure;
}

struct ReferenceData
{
  std::vector<double> p_values{4.0, 1.0, 2.0};
  std::vector<double> q{1.0, 1.0};
  std::vector<double> a_values{1.0, 1.0, 1.0, 1.0};
  std::vector<double> l{1.0, 0.0, 0.0};
  std::vector<double> u{1.0, 0.7, 0.7};
};

QpStats solve_with(QpSolver & solver, const ReferenceData & data, std::vector<double> & primal)
{
  primal.assign(2, 0.0);
  return solver.solve(data.p_values, data.q, data.a_values, data.l, data.u, primal);
}

}  // namespace

TEST(QpSolver, SolvesTheReferenceProblem)
{
  const auto solver = make_osqp_solver(reference_structure(), QpSolverParams{});
  ASSERT_NE(solver, nullptr);

  std::vector<double> primal;
  const QpStats stats = solve_with(*solver, ReferenceData{}, primal);

  EXPECT_EQ(stats.status, QpStatus::Solved);
  EXPECT_GT(stats.iterations, 0);
  EXPECT_NEAR(primal[0], 0.3, 1e-3);
  EXPECT_NEAR(primal[1], 0.7, 1e-3);
}

TEST(QpSolver, TwoFreshSolversAgreeBitForBit)
{
  const auto first = make_osqp_solver(reference_structure(), QpSolverParams{});
  const auto second = make_osqp_solver(reference_structure(), QpSolverParams{});
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);

  std::vector<double> left;
  std::vector<double> right;
  const QpStats left_stats = solve_with(*first, ReferenceData{}, left);
  const QpStats right_stats = solve_with(*second, ReferenceData{}, right);

  EXPECT_EQ(left_stats.status, right_stats.status);
  EXPECT_EQ(left_stats.iterations, right_stats.iterations);
  EXPECT_EQ(left[0], right[0]);
  EXPECT_EQ(left[1], right[1]);
}

TEST(QpSolver, ResetDropsTheWarmStart)
{
  const auto reused = make_osqp_solver(reference_structure(), QpSolverParams{});
  const auto fresh = make_osqp_solver(reference_structure(), QpSolverParams{});
  ASSERT_NE(reused, nullptr);
  ASSERT_NE(fresh, nullptr);

  std::vector<double> warm;
  std::vector<double> cold;
  std::vector<double> reference;
  static_cast<void>(solve_with(*reused, ReferenceData{}, warm));
  const QpStats warm_stats = solve_with(*reused, ReferenceData{}, warm);
  reused->reset();
  const QpStats cold_stats = solve_with(*reused, ReferenceData{}, cold);
  const QpStats reference_stats = solve_with(*fresh, ReferenceData{}, reference);

  EXPECT_EQ(cold_stats.iterations, reference_stats.iterations);
  EXPECT_EQ(cold[0], reference[0]);
  EXPECT_EQ(cold[1], reference[1]);
  EXPECT_LT(warm_stats.iterations, cold_stats.iterations);
}

TEST(QpSolver, ContradictoryBoundsAreReportedAsInfeasible)
{
  const auto solver = make_osqp_solver(reference_structure(), QpSolverParams{});
  ASSERT_NE(solver, nullptr);

  ReferenceData data;
  // Rows 2 and 3 read the same variable, so demanding x1 >= 1 and x1 <= -1 has no solution.
  data.l = {-10.0, 1.0, 0.0};
  data.u = {10.0, 1.0, 0.0};
  data.a_values = {1.0, 1.0, 1.0, 0.0};
  data.p_values = {4.0, 1.0, 2.0};
  data.l[2] = 5.0;
  data.u[2] = 5.0;

  std::vector<double> primal;
  const QpStats stats = solve_with(*solver, data, primal);
  EXPECT_EQ(stats.status, QpStatus::Infeasible);
}

TEST(QpSolver, AnIterationCapOfOneIsReportedAsMaxIterations)
{
  QpSolverParams params;
  params.max_iterations = 1;
  params.eps_abs = 1e-12;
  params.eps_rel = 1e-12;
  const auto solver = make_osqp_solver(reference_structure(), params);
  ASSERT_NE(solver, nullptr);

  std::vector<double> primal;
  const QpStats stats = solve_with(*solver, ReferenceData{}, primal);
  EXPECT_EQ(stats.status, QpStatus::MaxIterations);
}

TEST(QpSolver, MalformedStructuresAreRefused)
{
  EXPECT_EQ(make_osqp_solver(QpStructure{}, QpSolverParams{}), nullptr);

  QpStructure structure = reference_structure();
  structure.p_indptr = {0, 1};
  EXPECT_EQ(make_osqp_solver(structure, QpSolverParams{}), nullptr);

  structure = reference_structure();
  structure.a_indices = {0, 1, 0, 9};
  EXPECT_EQ(make_osqp_solver(structure, QpSolverParams{}), nullptr);

  structure = reference_structure();
  // A row index below the diagonal is a lower-triangle entry, which OSQP never reads.
  structure.p_indices = {1, 0, 1};
  EXPECT_EQ(make_osqp_solver(structure, QpSolverParams{}), nullptr);

  structure = reference_structure();
  structure.a_indices = {1, 0, 0, 2};
  EXPECT_EQ(make_osqp_solver(structure, QpSolverParams{}), nullptr);
}

TEST(QpSolver, BadSolverParamsAreRefused)
{
  QpSolverParams params;
  params.max_iterations = 0;
  EXPECT_EQ(make_osqp_solver(reference_structure(), params), nullptr);

  params = QpSolverParams{};
  params.adaptive_rho_interval = 0;
  EXPECT_EQ(make_osqp_solver(reference_structure(), params), nullptr);

  params = QpSolverParams{};
  params.eps_abs = kNan;
  EXPECT_EQ(make_osqp_solver(reference_structure(), params), nullptr);

  params = QpSolverParams{};
  params.eps_rel = -1.0;
  EXPECT_EQ(make_osqp_solver(reference_structure(), params), nullptr);
}

TEST(QpSolver, MismatchedArrayLengthsAreRefused)
{
  const auto solver = make_osqp_solver(reference_structure(), QpSolverParams{});
  ASSERT_NE(solver, nullptr);

  const ReferenceData data;
  std::vector<double> primal(2, 0.0);
  const std::vector<double> short_q{1.0};
  EXPECT_EQ(
    solver->solve(data.p_values, short_q, data.a_values, data.l, data.u, primal).status,
    QpStatus::SolverError);

  std::vector<double> short_primal(1, 0.0);
  EXPECT_EQ(
    solver->solve(data.p_values, data.q, data.a_values, data.l, data.u, short_primal).status,
    QpStatus::SolverError);
}

TEST(QpSolver, EveryStatusHasAName)
{
  for (const QpStatus status :
       {QpStatus::Solved, QpStatus::SolvedInaccurate, QpStatus::MaxIterations,
        QpStatus::Infeasible, QpStatus::NonFinite, QpStatus::SolverError}) {
    EXPECT_NE(std::string(to_string(status)), "unknown");
  }
}
