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

#include "qp_solver.hpp"

#include <osqp.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace eltanin::control::detail
{

namespace
{

/// v1.0.0 rejects time_limit <= 0, so "no limit" is spelled as a bound no solve can reach [s].
constexpr double NO_TIME_LIMIT = 1e10;

QpStatus status_of(OSQPInt value) noexcept
{
  switch (value) {
    case OSQP_SOLVED:
      return QpStatus::Solved;
    case OSQP_SOLVED_INACCURATE:
      return QpStatus::SolvedInaccurate;
    case OSQP_PRIMAL_INFEASIBLE:
    case OSQP_PRIMAL_INFEASIBLE_INACCURATE:
    case OSQP_DUAL_INFEASIBLE:
    case OSQP_DUAL_INFEASIBLE_INACCURATE:
      return QpStatus::Infeasible;
    case OSQP_MAX_ITER_REACHED:
    case OSQP_TIME_LIMIT_REACHED:
      return QpStatus::MaxIterations;
    default:
      return QpStatus::SolverError;
  }
}

/// Owns every array OSQP reads, so a solve only ever overwrites values already in place.
class OsqpSolver final : public QpSolver
{
public:
  OsqpSolver(const QpStructure & structure, const QpSolverParams & params)
  : variables_(structure.variables), constraints_(structure.constraints)
  {
    p_indptr_.assign(structure.p_indptr.begin(), structure.p_indptr.end());
    p_indices_.assign(structure.p_indices.begin(), structure.p_indices.end());
    p_values_.assign(structure.p_nonzeros(), 0.0);
    a_indptr_.assign(structure.a_indptr.begin(), structure.a_indptr.end());
    a_indices_.assign(structure.a_indices.begin(), structure.a_indices.end());
    a_values_.assign(structure.a_nonzeros(), 0.0);
    q_.assign(static_cast<std::size_t>(variables_), 0.0);
    l_.assign(static_cast<std::size_t>(constraints_), 0.0);
    u_.assign(static_cast<std::size_t>(constraints_), 0.0);

    osqp_set_default_settings(&settings_);
    settings_.verbose = 0;
    settings_.warm_starting = params.warm_start ? 1 : 0;
    settings_.polishing = params.polish ? 1 : 0;
    settings_.max_iter = params.max_iterations;
    settings_.eps_abs = params.eps_abs;
    settings_.eps_rel = params.eps_rel;
    // Pinned rather than left at the default: a time-based policy would make runs irreproducible.
    settings_.adaptive_rho = OSQP_ADAPTIVE_RHO_UPDATE_ITERATIONS;
    settings_.adaptive_rho_interval = params.adaptive_rho_interval;
    settings_.time_limit = NO_TIME_LIMIT;

    setup();
  }

  [[nodiscard]] bool usable() const noexcept { return solver_ != nullptr; }

  QpStats solve(
    std::span<const double> p_values, std::span<const double> q, std::span<const double> a_values,
    std::span<const double> l, std::span<const double> u, std::span<double> primal_out) override
  {
    QpStats stats;
    if (
      !solver_ || p_values.size() != p_values_.size() || a_values.size() != a_values_.size() ||
      q.size() != q_.size() || l.size() != l_.size() || u.size() != u_.size() ||
      primal_out.size() != q_.size()) {
      return stats;
    }

    std::copy(p_values.begin(), p_values.end(), p_values_.begin());
    std::copy(a_values.begin(), a_values.end(), a_values_.begin());
    std::copy(q.begin(), q.end(), q_.begin());
    std::copy(l.begin(), l.end(), l_.begin());
    std::copy(u.begin(), u.end(), u_.begin());

    if (osqp_update_data_mat(
          solver_.get(), p_values_.data(), OSQP_NULL,
          static_cast<OSQPInt>(p_values_.size()), a_values_.data(), OSQP_NULL,
          static_cast<OSQPInt>(a_values_.size())) != 0) {
      return stats;
    }
    if (osqp_update_data_vec(solver_.get(), q_.data(), l_.data(), u_.data()) != 0) {
      return stats;
    }
    if (osqp_solve(solver_.get()) != 0) {
      return stats;
    }

    const OSQPInfo & info = *solver_->info;
    stats.status = status_of(info.status_val);
    stats.iterations = static_cast<int>(info.iter);
    stats.objective = info.obj_val;
    stats.solve_time = info.run_time;
    if (stats.status != QpStatus::Solved && stats.status != QpStatus::SolvedInaccurate) {
      return stats;
    }

    const OSQPFloat * x = solver_->solution->x;
    for (std::size_t i = 0; i < primal_out.size(); ++i) {
      if (!std::isfinite(x[i])) {
        stats.status = QpStatus::NonFinite;
        return stats;
      }
      primal_out[i] = x[i];
    }
    return stats;
  }

  /// A full re-setup, because osqp_cold_start() leaves the adapted penalty from earlier solves.
  void reset() noexcept override { setup(); }

private:
  struct Cleanup
  {
    void operator()(OSQPSolver * solver) const noexcept { osqp_cleanup(solver); }
  };

  /// Rebuilds the workspace from zeroed data, so the state after it never depends on history.
  void setup() noexcept
  {
    solver_.reset();
    std::fill(p_values_.begin(), p_values_.end(), 0.0);
    std::fill(a_values_.begin(), a_values_.end(), 0.0);
    std::fill(q_.begin(), q_.end(), 0.0);
    std::fill(l_.begin(), l_.end(), 0.0);
    std::fill(u_.begin(), u_.end(), 0.0);

    OSQPCscMatrix p_matrix{};
    OSQPCscMatrix a_matrix{};
    OSQPCscMatrix_set_data(
      &p_matrix, variables_, variables_, static_cast<OSQPInt>(p_values_.size()), p_values_.data(),
      p_indices_.data(), p_indptr_.data());
    OSQPCscMatrix_set_data(
      &a_matrix, constraints_, variables_, static_cast<OSQPInt>(a_values_.size()),
      a_values_.data(), a_indices_.data(), a_indptr_.data());

    OSQPSolver * raw = nullptr;
    const OSQPInt flag = osqp_setup(
      &raw, &p_matrix, q_.data(), &a_matrix, l_.data(), u_.data(), constraints_, variables_,
      &settings_);
    if (flag == 0 && raw != nullptr) {
      solver_.reset(raw);
    } else if (raw != nullptr) {
      osqp_cleanup(raw);
    }
  }

  OSQPInt variables_{0};
  OSQPInt constraints_{0};
  std::vector<OSQPInt> p_indptr_;
  std::vector<OSQPInt> p_indices_;
  std::vector<OSQPFloat> p_values_;
  std::vector<OSQPInt> a_indptr_;
  std::vector<OSQPInt> a_indices_;
  std::vector<OSQPFloat> a_values_;
  std::vector<OSQPFloat> q_;
  std::vector<OSQPFloat> l_;
  std::vector<OSQPFloat> u_;
  OSQPSettings settings_{};
  std::unique_ptr<OSQPSolver, Cleanup> solver_;
};

}  // namespace

std::unique_ptr<QpSolver> make_osqp_solver(
  const QpStructure & structure, const QpSolverParams & params)
{
  if (!structure.valid()) {
    return nullptr;
  }
  if (
    params.max_iterations <= 0 || params.adaptive_rho_interval <= 0 ||
    !std::isfinite(params.eps_abs) || params.eps_abs <= 0.0 || !std::isfinite(params.eps_rel) ||
    params.eps_rel <= 0.0) {
    return nullptr;
  }
  auto solver = std::make_unique<OsqpSolver>(structure, params);
  if (!solver->usable()) {
    return nullptr;
  }
  return solver;
}

}  // namespace eltanin::control::detail
