# eltanin

A non-ROS C++20 library for 2D autonomous navigation.

Core planning, control and map logic depends only on the C++ standard library and Eigen. Anything
that needs ROS 2, visualization or file I/O lives in a separate target.

## Modules

| Target | Alias | Depends on | Contents |
|---|---|---|---|
| `eltanin_core` | `eltanin::core` | Eigen | `Pose2D` / `Twist2D` / `Transform2D`, angle normalization, point-segment geometry, `Polygon2D`, footprint radii, `Traversability`, `Path`, differential-drive integration |
| `eltanin_map` | `eltanin::map` | `eltanin::core` | `MapGeometry` (the only world/map conversion), `GridMap<T>`, nav2-scale cost constants, cost model |
| `eltanin_map_io` | `eltanin::map_io` | `eltanin::map`, yaml-cpp | PGM + YAML map loading, PGM debug dump |
| `eltanin_sensor` | `eltanin::sensor` | `eltanin::core` | Laser scan projection into planar points (`ScanData` / `ScanFilter` / `project_scan`) |
| `eltanin_planner` | `eltanin::planner` | `eltanin::core`, `eltanin::map` | 8-connected A\* global planner, nearest traversable cell search, iterative path smoother |
| `eltanin_control` | `eltanin::control` | `eltanin::core` | Pure pursuit path tracking (`PurePursuit`) |
| `eltanin_sim` | `eltanin::sim` | `eltanin::core` | Deterministic differential-drive plant (`SimpleSimulator`) |
| `eltanin_collision` | `eltanin::collision` | `eltanin::core`, `eltanin::map` | Two-stage footprint collision checking, braking-distance velocity limiting (`VelocityLimiter`) |

Design decisions for the costmap, collision classification, error reporting and module layout are
recorded in [docs/costmap-design.md](docs/costmap-design.md). Per-module design notes live in
[docs/sensor-design.md](docs/sensor-design.md), [docs/planner-design.md](docs/planner-design.md),
[docs/control-design.md](docs/control-design.md) and
[docs/collision-design.md](docs/collision-design.md);
[docs/integration-design.md](docs/integration-design.md) covers the demo that runs all of them
together.

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
| `ELTANIN_TEST_MAP_DIR` | navyu map directory | Directory holding `map.pgm` / `map.yaml` used by the real-map tests and as the default map of `eltanin_navigate_on_real_map`; those tests are skipped when it is absent |

### Sanitizer build

```bash
cmake -B build-asan -DELTANIN_ENABLE_ASAN=ON -DELTANIN_ENABLE_UBSAN=ON
cmake --build build-asan -j
ctest --test-dir build-asan --output-on-failure
```

## Install and use from another project

```bash
cmake -B build
cmake --build build -j
cmake --install build --prefix /path/to/prefix
```

Then, in the consuming project:

```cmake
find_package(eltanin REQUIRED)
target_link_libraries(my_target PRIVATE eltanin::core eltanin::map eltanin::map_io)
```

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/prefix
```

`test/package_test/` is a minimal standalone project used to verify this path. It is deliberately
not part of the main build:

```bash
cmake -S test/package_test -B /tmp/pkgtest -DCMAKE_PREFIX_PATH=/path/to/prefix
cmake --build /tmp/pkgtest
/tmp/pkgtest/eltanin_package_test
```

## Examples

```bash
cmake -B build -DELTANIN_BUILD_EXAMPLES=ON
cmake --build build -j
./build/examples/eltanin_load_map_summary path/to/map.yaml
```

Prints the map geometry and a histogram of cost values, which is handy for checking that a map was
loaded with the expected resolution, origin and occupancy thresholds.

```bash
./build/examples/eltanin_plan_on_real_map path/to/map.yaml out_dir
./build/examples/eltanin_track_on_real_map path/to/map.yaml out_dir
```

Both inflate a real map and run the planner on it; the second one then tracks the resulting path with
`PurePursuit`. They write a cropped `crop.pgm` plus CSV files and a `meta.txt` summary, so the path
and the traced trajectory can be plotted over the costmap. The tracking measurements taken this way
are recorded in [docs/control-design.md](docs/control-design.md) §5.

```bash
./build/examples/eltanin_limiter_profile out_dir
./build/examples/eltanin_limit_on_real_map path/to/map.yaml out_dir
```

The first one needs no map: it runs `VelocityLimiter` on synthetic wall and single-obstacle maps and
writes the limited speed against the gap to the wall for a forward **and** a reverse command, next to
what navyu's `std::min()` would have produced, plus the collision verdict over a full turn. The
second plans and tracks on a real map, stamps an obstacle onto the planned path after planning and
dumps the predicted poses and the world-frame footprint.

Both write CSV and PGM only; `examples/plot_collision_results.py` turns those into figures.

```bash
python3 examples/plot_collision_results.py --synthetic out_dir --real out_dir --out plots
```

It is a developer tool, deliberately not wired into CMake: nothing in the build refers to it, so
Python never becomes a build dependency. It needs matplotlib and numpy. What each figure shows is
described in [docs/collision-design.md](docs/collision-design.md) §11.

## Running the whole stack without ROS

`eltanin_navigate_on_real_map` closes the loop over every module in a single process. No ROS node, no
tf, no topic, no simulator process:

```
map_io::load_map
  -> LayeredCostmap (static + obstacle + inflation), global for planning and local for the limiter
  -> synthetic LiDAR -> sensor::project_scan -> ObstacleLayer
  -> planner::plan (A*) -> planner::smooth
  -> control::PurePursuit
  -> collision::VelocityLimiter (footprint prediction, braking-distance law)
  -> sim::SimpleSimulator (differential-drive integration)
  -> repeat, replanning once the observations block the path ahead
```

An obstacle beyond the 3 m sensor range cannot be seen, so the first path runs straight through it.
Only when the robot gets close enough do the cells land in its belief, and that is what triggers the
replan — the robot curves around without ever stopping. Pass `--replan-on-stop-only` to hold the
replan back until `VelocityLimiter` has brought the robot to a standstill instead, which is what
exercises the limiter.

```bash
cmake -B build -DELTANIN_BUILD_EXAMPLES=ON
cmake --build build -j
./build/examples/eltanin_navigate_on_real_map out_dir
```

The map defaults to `${ELTANIN_TEST_MAP_DIR}/map.yaml`, so the command above runs as it is; pass
`--map path/to/map.yaml` for another one. Start and goal are picked from the map, so no coordinate is
hard coded. An unknown obstacle is stamped onto the ground truth halfway along the planned path
*after* planning, so the robot has to discover it; `--obstacle-fraction 0` gives a clean run instead.
`--help` is not a flag, but any wrong argument prints the full option list.

The run prints the per-leg statistics, the total cycle count, the final position error and whether
the goal was reached, then writes six files into `out_dir`:

| File | Contents |
|---|---|
| `costmap.pgm` | Cropped global costmap, raw cost values |
| `traversed.pgm` | Same crop and geometry, cells the robot actually occupied |
| `path.csv` | `leg,index,x,y,yaw` for every planned path, replans included |
| `trajectory.csv` | One row per control cycle: pose, requested and limited command, collision distance |
| `obstacles.csv` | `t,x,y` of the cells found occupied that the static map calls free |
| `meta.txt` | Crop geometry, radii, every parameter, per-leg statistics, outcome, stop clearance |

The exit code is 0 only when the goal was reached; otherwise the failing stage is named on stderr.

```bash
python3 examples/plot_navigation_results.py --run out_dir --out plots
```

Turns those files into four figures: the whole route over the costmap, a zoom on the unknown obstacle,
the requested against the limited command over time, and the traversed cells over the costmap. Add
`--animate` for a `navigation.gif` that plays the run back next to the local window the limiter sees.
Like `plot_collision_results.py` it is a developer tool that CMake never refers to. What each figure
shows, and what reading them revealed, is in
[docs/integration-design.md](docs/integration-design.md) §15.

The same pipeline is a regression test, so `ctest` covers it. Under both sanitizers:

```bash
cmake -B build-asan -DELTANIN_ENABLE_ASAN=ON -DELTANIN_ENABLE_UBSAN=ON
cmake --build build-asan -j
ctest --test-dir build-asan -L integration --output-on-failure
```

That takes about 95 s and needs about 500 MB. The cases that use the reference map are skipped when
`ELTANIN_TEST_MAP_DIR` does not hold one; the two that fix the local window snapping always run.

Known gaps, since the demo is honest about them: there is no local planner (a detour comes from
global replanning only), and nothing decelerates on the final approach to the goal, so the requested
command is still 0.5 m/s when `PurePursuit` reports `GoalReached` and the goal yaw does not converge.
[docs/integration-design.md](docs/integration-design.md) records the design decisions and the
measured numbers.

## License

Apache License 2.0. See [LICENSE](LICENSE).
