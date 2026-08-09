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

namespace eltanin::control::detail
{

namespace
{

/// One compressed-sparse-column pattern: column pointers in order, row indices sorted inside each.
bool columns_are_well_formed(
  const std::vector<int> & indptr, const std::vector<int> & indices, int columns, int rows,
  bool upper_triangle_only)
{
  if (indptr.size() != static_cast<std::size_t>(columns) + 1 || indptr.front() != 0) {
    return false;
  }
  if (indptr.back() != static_cast<int>(indices.size())) {
    return false;
  }
  for (int column = 0; column < columns; ++column) {
    if (indptr[static_cast<std::size_t>(column) + 1] < indptr[static_cast<std::size_t>(column)]) {
      return false;
    }
    for (int k = indptr[static_cast<std::size_t>(column)];
         k < indptr[static_cast<std::size_t>(column) + 1]; ++k) {
      const int row = indices[static_cast<std::size_t>(k)];
      if (row < 0 || row >= rows) {
        return false;
      }
      if (upper_triangle_only && row > column) {
        return false;
      }
      if (k > indptr[static_cast<std::size_t>(column)] &&
          row <= indices[static_cast<std::size_t>(k) - 1]) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

bool QpStructure::valid() const noexcept
{
  if (variables <= 0 || constraints <= 0) {
    return false;
  }
  return columns_are_well_formed(p_indptr, p_indices, variables, variables, true) &&
         columns_are_well_formed(a_indptr, a_indices, variables, constraints, false);
}

}  // namespace eltanin::control::detail
