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
| `eltanin_planner` | `eltanin::planner` | `eltanin::core`, `eltanin::map` | 8-connected A\*, forward-only Hybrid A\*, Dubins path, nearest traversable cell search, iterative path smoother |
| `eltanin_control` | `eltanin::control` | `eltanin::core` | Pure pursuit path tracking (`PurePursuit`), goal approach deceleration and final yaw alignment (`GoalApproach`) |
| `eltanin_sim` | `eltanin::sim` | `eltanin::core` | Deterministic differential-drive plant (`SimpleSimulator`) |
| `eltanin_collision` | `eltanin::collision` | `eltanin::core`, `eltanin::map` | Two-stage and exact footprint collision checking, braking-distance velocity limiting (`VelocityLimiter`) |

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

The Hybrid A* example loads and inflates a YAML/PGM map for the configured robot footprint. It uses
an A* path only to crop the large input map, then runs Hybrid A* on that real-map corridor. Start and
goal include yaw in radians:

```bash
./build/examples/eltanin_hybrid_astar_demo \
  path/to/map.yaml /tmp/eltanin-hybrid \
  36.175 -15.925 1.5708 34.675 -9.025 1.5708
python3 examples/plot_hybrid_astar.py /tmp/eltanin-hybrid \
  --out /tmp/eltanin-hybrid/hybrid_astar.png --animate
```

The static figure draws the footprint at every path sample, producing the dense swept-path view
commonly used for Hybrid A*. `--footprint-step N` can thin the outlines for a very long path.
`--animate` also writes `hybrid_astar.gif`, with the footprint moving over the map and a following
local view. The script needs matplotlib, numpy and Pillow. They are developer-only dependencies and
are not linked into `eltanin_planner` or any other library target.

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

The planner module provides a runtime-polymorphic seam while keeping map/model adaptation
compile-time checked. Algorithm-specific parameters stay in each concrete planner:

```cpp
#include <eltanin/planner/astar_planner.hpp>
#include <eltanin/planner/hybrid_astar_planner.hpp>

#include <memory>

std::unique_ptr<eltanin::planner::Planner> planner =
  std::make_unique<eltanin::planner::AStarPlanner>();
auto path = planner->plan(map, model, start, goal);

planner = std::make_unique<eltanin::planner::HybridAStarPlanner>(hybrid_params);
path = planner->plan(map, model, start, goal);
```

`planner::plan_astar(...)` and `planner::plan_hybrid_astar(...)` are the matching convenience free
functions. Both return a `PlanResult`: an `std::optional`-compatible value that also carries a
`PlannerError` explaining why there is no path (`to_string(result.error())` names it).

Whichever planner is selected, the returned path is directly followable — **do not call `smooth()` on
it**. `plan_astar()` already smooths, reusing the classified grid it built for the search, so a second
`smooth()` only adds a full-map classification pass worth roughly one search. Pass
`AStarParams::smoother = std::nullopt` when the raw cell-center path is what you want, and keep
`smooth()` for paths that did not come from a planner. Hybrid A* is never smoothed, because that would
break the curvature bound.

Hybrid A* tries its analytic Dubins connection at the first expanded node and then on a throttle that
tightens as the goal gets closer, so on an open map the returned path *is* the optimal Dubins path
rather than a heading-quantized weave. `analytic_expansion_ratio` controls the throttle; raising it
above the default tries more often, which measurably degrades detours around obstacles.
`motion_model` declares what the vehicle can do, and the returned path contains nothing else:
`Dubins` is forward arcs at or above the turning radius, `Differential` adds turning on the spot,
which is what a differential drive actually does, and `ReedsShepp` adds driving in reverse. The
difference is not cosmetic — on the reference map a requested goal heading is reachable in 3 of 8
directions under `Dubins` and in 8 of 8 under `Differential`, both with the same
`1 / minimum_turning_radius` bound on every travelling segment. A turn on the spot appears as its own
pose, so two consecutive poses can share a position; that is the one place the spacing contract below
does not apply. Under `ReedsShepp` every cusp is a pose of its own, so a follower can see where the
gear changes; `reverse_penalty` multiplies the cost of reversing and `direction_change_penalty` charges
each gear change, and both apply to the analytic tail as well as to the search.

Both planners charge for travelling close to a wall through `ClearanceCost{penalty, distance}`, which
adds `penalty * max(0, 1 - clearance / distance)` per metre travelled. It is **on for Hybrid A***
(`{0.3, 0.2}`, which runs on a corridor where the distance field is cheap) and **off for A***, because
A* runs on the full map and the distance field there costs more than the search. The path smoother
carries the same idea locally: `weight_obstacle` / `obstacle_distance` push points off walls and
`weight_curvature` penalises bending, using a distance field built only over a window around the path.
`docs/planner-design.md` §14.4 and §14.5 record what each weight buys and what it costs.

When the `Free` cells leave start and goal disconnected — a doorway the robot barely fits through —
the base planner retries once with `Traversability::Circumscribed` opened up, charging
`NarrowPassageFallback::penalty` per metre of band used so it crosses where the band is thinnest. A
path found that way reports `PlanResult::narrow_passage()`. Set `narrow_passage.enabled = false` to
keep the strict single-pass behaviour.

Hybrid A* holds `(cell, heading_bin)` search state, so its memory grows with cells times
`heading_bins`: about 8.125 bytes per state, or 25 MB for a 10 m square map at 0.05 m and 72 bins.
`max_state_memory_bytes` (256 MiB by default) rejects anything larger with `StateSpaceTooLarge` rather
than throwing, which means **a full 4000 x 4000 map does not fit** — crop a corridor around a raw A*
guide first with `map::crop_around()`, as `eltanin_hybrid_astar_demo` and the navigation loop do. On the
4000 x 4000 reference map that turns 1.15e9 states into 5.8e6 and plans in about 380 ms.
`docs/planner-design.md` §13 and §14 record the measurements behind these numbers.

`eltanin_navigate_on_real_map` closes the loop over every module in a single process. No ROS node, no
tf, no topic, no simulator process:

```
map_io::load_map
  -> LayeredCostmap (static + obstacle + inflation), global for planning and local for the limiter
  -> synthetic LiDAR -> sensor::project_scan -> ObstacleLayer
  -> selected global planner (A* + smoothing, or Hybrid A* in an A*-guided corridor)
  -> control::PurePursuit + control::GoalApproach
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
./build/examples/eltanin_navigate_on_real_map out_dir --planner hybrid-astar
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
For a Hybrid A* run, the overview, obstacle zoom and animation draw the footprint at every planned
pose; no footprint thinning is applied.
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
global replanning only), and the injected obstacle is stationary after planning rather than a moving
obstacle. `GoalApproach` handles final deceleration and goal-yaw alignment.
[docs/integration-design.md](docs/integration-design.md) records the design decisions and the
measured numbers.

## License

Apache License 2.0. See [LICENSE](LICENSE).
