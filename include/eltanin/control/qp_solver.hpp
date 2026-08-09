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

#ifndef ELTANIN__CONTROL__QP_SOLVER_HPP_
#define ELTANIN__CONTROL__QP_SOLVER_HPP_

#include <eltanin/control/qp_solver_params.hpp>

#include <cstddef>
#include <memory>
#include <span>
#include <vector>

namespace eltanin::control::detail
{

/// How a solve ended; everything but the first two means the caller must not use the solution.
enum class QpStatus
{
  Solved,
  SolvedInaccurate,
  MaxIterations,
  Infeasible,
  NonFinite,
  SolverError
};

constexpr const char * to_string(QpStatus status) noexcept
{
  switch (status) {
    case QpStatus::Solved:
      return "solved";
    case QpStatus::SolvedInaccurate:
      return "solved inaccurate";
    case QpStatus::MaxIterations:
      return "max iterations";
    case QpStatus::Infeasible:
      return "infeasible";
    case QpStatus::NonFinite:
      return "non-finite";
    case QpStatus::SolverError:
      return "solver error";
  }
  return "unknown";
}

struct QpStats
{
  QpStatus status{QpStatus::SolverError};
  int iterations{0};
  double objective{0.0};
  /// Wall time the backend spent inside solve() [s]; recorded, never acted on.
  double solve_time{0.0};
};

/// min 0.5 x'Px + q'x subject to l <= Ax <= u, with both sparsity patterns fixed for the run.
struct QpStructure
{
  int variables{0};
  int constraints{0};
  /// Upper triangle of P in compressed sparse column form.
  std::vector<int> p_indptr;
  std::vector<int> p_indices;
  std::vector<int> a_indptr;
  std::vector<int> a_indices;

  [[nodiscard]] std::size_t p_nonzeros() const noexcept
  {
    return p_indices.size();
  }

  [[nodiscard]] std::size_t a_nonzeros() const noexcept
  {
    return a_indices.size();
  }

  /// Column pointers consistent with the row indices, and both dimensions positive.
  [[nodiscard]] bool valid() const noexcept;
};

/// The seam a future in-house solver replaces; nothing above it names a third-party type.
class QpSolver
{
public:
  virtual ~QpSolver() = default;

  QpSolver(const QpSolver &) = delete;
  QpSolver & operator=(const QpSolver &) = delete;
  QpSolver(QpSolver &&) = delete;
  QpSolver & operator=(QpSolver &&) = delete;

  /// Values only; the index arrays handed to the factory stay in force for the whole run.
  virtual QpStats solve(
    std::span<const double> p_values, std::span<const double> q, std::span<const double> a_values,
    std::span<const double> l, std::span<const double> u, std::span<double> primal_out) = 0;

  /// Drops the warm start so the next solve depends on its inputs alone.
  virtual void reset() noexcept = 0;

protected:
  QpSolver() = default;
};

/// nullptr when the structure is malformed or the backend refuses it; the only user of osqp.h.
std::unique_ptr<QpSolver> make_osqp_solver(
  const QpStructure & structure, const QpSolverParams & params);

}  // namespace eltanin::control::detail

#endif  // ELTANIN__CONTROL__QP_SOLVER_HPP_
