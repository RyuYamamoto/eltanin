# eltanin

A non-ROS C++20 library for 2D autonomous navigation.

Core planning, control and map logic depends only on the C++ standard library and Eigen. Anything
that needs ROS 2, visualization or file I/O lives in a separate target.

## Architecture

<!-- Keep this table in sync with the one in docs/index.md. -->

| Target | Alias | Depends on | Contents |
|---|---|---|---|
| `eltanin_core` | `eltanin::core` | Eigen | Poses and twists, 2D geometry, footprint radii, `Traversability`, `Path`, differential-drive integration |
| `eltanin_map` | `eltanin::map` | `eltanin::core` | `MapGeometry` (the only world/map conversion), `GridMap<T>`, nav2-scale cost constants, cost model |
| `eltanin_map_io` | `eltanin::map_io` | `eltanin::map`, yaml-cpp | PGM + YAML map loading, PGM debug dump |
| `eltanin_sensor` | `eltanin::sensor` | `eltanin::core` | Laser scan projection into planar points |
| `eltanin_planner` | `eltanin::planner` | `eltanin::core`, `eltanin::map` | A\*, Hybrid A\* over (x, y, yaw), Dubins and Reeds-Shepp paths, clearance map, path smoother |
| `eltanin_control` | `eltanin::control` | `eltanin::core` (+ OSQP with `ELTANIN_ENABLE_MPC=ON`) | `PathFollower` interface: `PurePursuit` and `MpcFollower`, `VelocityProfile`, `GoalApproach` |
| `eltanin_sim` | `eltanin::sim` | `eltanin::core` | Deterministic differential-drive plant (`SimpleSimulator`) |
| `eltanin_collision` | `eltanin::collision` | `eltanin::core`, `eltanin::map` | Two-stage and exact footprint collision checking, `VelocityLimiter` |

Dependencies point one way only: `core` and `map` know nothing about the modules above them, and no
core module knows about ROS 2, a visualizer or a file format.

`eltanin_navigate_on_real_map` wires all of it into one process, without ROS:

```
map_io::load_map
  -> LayeredCostmap (static + obstacle + inflation), global for planning and local for the limiter
  -> synthetic LiDAR -> sensor::project_scan -> ObstacleLayer
  -> selected global planner (A* + smoothing, or Hybrid A* in an A*-guided corridor)
  -> control::PathFollower + control::GoalApproach
  -> collision::VelocityLimiter (footprint prediction, braking-distance law)
  -> sim::SimpleSimulator (differential-drive integration)
  -> repeat, replanning once the observations block the path ahead
```

There is no local planner yet, so a detour comes from global replanning only. See
[docs/navigation-demo.md](docs/navigation-demo.md).

## Requirements

- CMake 3.20 or newer
- A C++20 compiler (verified with g++ 13.3)
- Eigen 3.4 or newer
- yaml-cpp (only for `eltanin_map_io`)
- GoogleTest (only when building tests)

## Build and test

```bash
cmake -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

`CMAKE_BUILD_TYPE` is intentionally left empty by default so that `NDEBUG` is not defined and the
`assert()`-based precondition checks stay active. Pass `-DCMAKE_BUILD_TYPE=Release` for an optimized
build; note that this disables those checks.

### Options

| Option | Default | Effect |
|---|---|---|
| `ELTANIN_BUILD_TESTS` | ON at top level | Build the GoogleTest suites |
| `ELTANIN_BUILD_EXAMPLES` | OFF | Build `examples/` |
| `ELTANIN_ENABLE_WERROR` | OFF | Add `-Werror` |
| `ELTANIN_ENABLE_ASAN` | OFF | AddressSanitizer |
| `ELTANIN_ENABLE_UBSAN` | OFF | UndefinedBehaviorSanitizer |
| `ELTANIN_ENABLE_MPC` | OFF | Build the MPC path follower; adds an OSQP dependency to `eltanin_control` |
| `ELTANIN_MPC_SOLVER_PROVIDER` | `fetch` | Where OSQP comes from when the MPC is on: `fetch` or `package` |
| `ELTANIN_TEST_MAP_DIR` | navyu map directory | Directory holding `map.pgm` / `map.yaml` for the real-map tests and the demo default |

[docs/build-and-test.md](docs/build-and-test.md) covers the MPC and sanitizer builds, the reference
map and how tests skip without it, the `integration` test label, and installing the package.

## Use from another project

```cmake
find_package(eltanin REQUIRED)
target_link_libraries(my_target PRIVATE eltanin::core eltanin::map eltanin::map_io)
```

See [docs/build-and-test.md](docs/build-and-test.md#install-and-use-from-another-project) for the
install step and for `test/package_test/`, which verifies this path.

## Documentation

Usage guides in English and per-module design notes in Japanese live under `docs/`.

## License

Apache License 2.0. See [LICENSE](LICENSE).
