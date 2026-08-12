# Build and test

Everything about configuring, building, testing and installing eltanin. The [README](https://github.com/RyuYamamoto/eltanin#build-and-test)
carries the three commands that cover the common case; this page covers the rest.

This guide states procedures and contracts. Measured numbers and the reasoning behind the design
live in the Japanese design notes, referenced by section number where relevant.

## Build type and `assert()`

```bash
cmake -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

`CMAKE_BUILD_TYPE` is intentionally left empty by default so that `NDEBUG` is not defined and the
`assert()`-based precondition checks stay active. The library uses `assert()` for preconditions that
a caller must not violate — map/world conversion bounds, footprint validity, parameter ranges — so
the default build is the one that catches misuse.

Pass `-DCMAKE_BUILD_TYPE=Release` for an optimized build, and note that this disables those checks:

```bash
cmake -B build-release -DCMAKE_BUILD_TYPE=Release
```

Benchmark-style measurements are only meaningful in a `Release` build; correctness runs should use
the default.

## Options

| Option | Default | Effect |
|---|---|---|
| `ELTANIN_BUILD_TESTS` | ON at top level | Build the GoogleTest suites. Defaults to OFF when eltanin is consumed via `add_subdirectory()` |
| `ELTANIN_BUILD_EXAMPLES` | OFF | Build `examples/` (see [Examples](examples.md)) |
| `ELTANIN_ENABLE_WERROR` | OFF | Add `-Werror` to the per-target warning flags (`-Wall -Wextra -Wpedantic`) |
| `ELTANIN_ENABLE_ASAN` | OFF | AddressSanitizer |
| `ELTANIN_ENABLE_UBSAN` | OFF | UndefinedBehaviorSanitizer |
| `ELTANIN_ENABLE_MPC` | OFF | Build the MPC path follower; adds an OSQP dependency to `eltanin_control` |
| `ELTANIN_MPC_SOLVER_PROVIDER` | `fetch` | Where OSQP comes from when the MPC is on: `fetch` or `package` |
| `ELTANIN_TEST_MAP_DIR` | navyu map directory | Directory holding `map.pgm` / `map.yaml` for the real-map tests and the demo default |

Warning flags are attached per target as `PRIVATE`, so they never leak into a consuming project.
Sanitizer flags are applied with directory scope for the same reason: putting them on the target
interface would force `install(EXPORT)` to export an extra `INTERFACE` library.

## MPC build

The MPC follower is off by default so that the default build of `eltanin_control` stays on the
standard library and Eigen alone.

```bash
cmake -B build-mpc -DELTANIN_ENABLE_MPC=ON
cmake --build build-mpc -j
ctest --test-dir build-mpc --output-on-failure
```

`ELTANIN_MPC_SOLVER_PROVIDER` selects where OSQP comes from:

- `fetch` (default) pulls OSQP v1.0.0 with `FetchContent`. **Needs network access** on the first
  configure. The vendored build is forced to a static library with the builtin algebra backend and
  no demo, tests or printing, so a stale cache from an earlier configure cannot turn those back on.
- `package` calls `find_package(osqp REQUIRED)` and links `osqp::osqpstatic`. This is what an
  offline CI wants, and what a ROS workspace that already provides `osqp-vendor` should use.

Any other value fails the configure with a `FATAL_ERROR` rather than silently building without the
MPC.

The MPC formulation, its horizon and weights, and the tracking measurements are in
[control-design.md](https://github.com/RyuYamamoto/eltanin/blob/main/docs/control-design.md).

## Sanitizer build

```bash
cmake -B build-asan -DELTANIN_ENABLE_ASAN=ON -DELTANIN_ENABLE_UBSAN=ON
cmake --build build-asan -j
ctest --test-dir build-asan --output-on-failure
```

Both sanitizers can be enabled together. Because the default build type keeps `assert()` active, a
sanitizer run also exercises every precondition check.

## The reference map

Several tests and the navigation demo need a real occupancy map. `ELTANIN_TEST_MAP_DIR` points at a
directory holding `map.pgm` and `map.yaml`:

```bash
cmake -B build -DELTANIN_TEST_MAP_DIR=/path/to/map/dir
```

The default points at a navyu map directory on the author's machine. **Tests that need the reference
map are skipped, not failed, when the directory does not hold one**, so a fresh clone still gets a
green `ctest` run. Skipped cases report themselves through GoogleTest's `GTEST_SKIP()`, so the
absence is visible in the test output rather than silent.

Tests that do not depend on the map — synthetic grids, geometry, the cases that pin the local window
snapping — always run.

## Integration tests

The full navigation pipeline is a regression test, labelled `integration`:

```bash
cmake -B build-asan -DELTANIN_ENABLE_ASAN=ON -DELTANIN_ENABLE_UBSAN=ON
cmake --build build-asan -j
ctest --test-dir build-asan -L integration --output-on-failure
```

Under both sanitizers that takes about 95 s and needs about 500 MB, which is why it carries its own
label: `ctest -LE integration` gives a fast run during development.

The cases that use the reference map are skipped when `ELTANIN_TEST_MAP_DIR` does not hold one; the
two that fix the local window snapping always run.

What the integration test asserts, and what running it revealed, is in
[integration-design.md](https://github.com/RyuYamamoto/eltanin/blob/main/docs/integration-design.md).

## Install and use from another project

```bash
cmake -B build
cmake --build build -j
cmake --install build --prefix /path/to/prefix
```

This installs the headers, the static libraries, `eltaninTargets.cmake` under the `eltanin::`
namespace, and a config package with `SameMajorVersion` compatibility. In the consuming project:

```cmake
find_package(eltanin REQUIRED)
target_link_libraries(my_target PRIVATE eltanin::core eltanin::map eltanin::map_io)
```

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/prefix
```

Link only the modules you use. `eltanin::core` and `eltanin::map` carry no dependency beyond Eigen;
`eltanin::map_io` adds yaml-cpp, and `eltanin::control` adds OSQP only when built with
`ELTANIN_ENABLE_MPC=ON`.

### Verifying the installed package

`test/package_test/` is a minimal standalone project that consumes the installed package the same
way an external project would. It is deliberately **not** part of the main build, so that it cannot
accidentally pick up the source tree instead of the install tree:

```bash
cmake -S test/package_test -B /tmp/pkgtest -DCMAKE_PREFIX_PATH=/path/to/prefix
cmake --build /tmp/pkgtest
/tmp/pkgtest/eltanin_package_test
```

A failure here means the export set or the config package is broken even though the in-tree build
passes — usually a missing `install()` for a new header directory, or a private dependency that
leaked into a target's `INTERFACE`.
