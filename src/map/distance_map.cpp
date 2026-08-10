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

#include <eltanin/map/distance_map.hpp>

#include <cstddef>
#include <limits>
#include <vector>

namespace eltanin::map::detail
{

namespace
{

/// Working buffers of one separable pass, kept across rows so the transform allocates once.
struct EnvelopeBuffers
{
  std::vector<double> source;
  std::vector<double> transformed;
  std::vector<int> vertices;
  std::vector<double> boundaries;
};

/// Abscissa where the parabola rooted at `q` drops below the one rooted at `p`.
double intersection(const std::vector<double> & f, int q, int p) noexcept
{
  const double dq = static_cast<double>(q);
  const double dp = static_cast<double>(p);
  return ((f[static_cast<std::size_t>(q)] + dq * dq) - (f[static_cast<std::size_t>(p)] + dp * dp)) /
         (2.0 * dq - 2.0 * dp);
}

/// Lower envelope of the parabolas rooted at every sample of one row or column.
void transform_1d(EnvelopeBuffers & buffers, int n)
{
  const std::vector<double> & f = buffers.source;
  std::vector<double> & d = buffers.transformed;
  std::vector<int> & v = buffers.vertices;
  std::vector<double> & z = buffers.boundaries;
  constexpr double UNBOUNDED = std::numeric_limits<double>::infinity();

  int k = 0;
  v[0] = 0;
  z[0] = -UNBOUNDED;
  z[1] = UNBOUNDED;
  for (int q = 1; q < n; ++q) {
    double s = intersection(f, q, v[static_cast<std::size_t>(k)]);
    while (s <= z[static_cast<std::size_t>(k)]) {
      --k;
      s = intersection(f, q, v[static_cast<std::size_t>(k)]);
    }
    ++k;
    v[static_cast<std::size_t>(k)] = q;
    z[static_cast<std::size_t>(k)] = s;
    z[static_cast<std::size_t>(k) + 1] = UNBOUNDED;
  }

  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[static_cast<std::size_t>(k) + 1] < static_cast<double>(q)) {
      ++k;
    }
    const double offset = static_cast<double>(q - v[static_cast<std::size_t>(k)]);
    d[static_cast<std::size_t>(q)] = offset * offset + f[static_cast<std::size_t>(v[static_cast<std::size_t>(k)])];
  }
}

void resize_buffers(EnvelopeBuffers & buffers, int n)
{
  const std::size_t length = static_cast<std::size_t>(n);
  buffers.source.assign(length, 0.0);
  buffers.transformed.assign(length, 0.0);
  buffers.vertices.assign(length, 0);
  buffers.boundaries.assign(length + 1, 0.0);
}

}  // namespace

double unreachable_squared(int size_x, int size_y) noexcept
{
  const double dx = static_cast<double>(size_x);
  const double dy = static_cast<double>(size_y);
  return dx * dx + dy * dy + 1.0;
}

void distance_transform_squared(std::vector<double> & squared, int size_x, int size_y)
{
  if (size_x <= 0 || size_y <= 0) {
    return;
  }

  EnvelopeBuffers buffers;
  resize_buffers(buffers, size_x);
  for (int my = 0; my < size_y; ++my) {
    const std::size_t row = static_cast<std::size_t>(my) * static_cast<std::size_t>(size_x);
    for (int mx = 0; mx < size_x; ++mx) {
      buffers.source[static_cast<std::size_t>(mx)] = squared[row + static_cast<std::size_t>(mx)];
    }
    transform_1d(buffers, size_x);
    for (int mx = 0; mx < size_x; ++mx) {
      squared[row + static_cast<std::size_t>(mx)] =
        buffers.transformed[static_cast<std::size_t>(mx)];
    }
  }

  resize_buffers(buffers, size_y);
  for (int mx = 0; mx < size_x; ++mx) {
    for (int my = 0; my < size_y; ++my) {
      buffers.source[static_cast<std::size_t>(my)] =
        squared[static_cast<std::size_t>(my) * static_cast<std::size_t>(size_x) +
                static_cast<std::size_t>(mx)];
    }
    transform_1d(buffers, size_y);
    for (int my = 0; my < size_y; ++my) {
      squared
        [static_cast<std::size_t>(my) * static_cast<std::size_t>(size_x) +
         static_cast<std::size_t>(mx)] = buffers.transformed[static_cast<std::size_t>(my)];
    }
  }
}

}  // namespace eltanin::map::detail
