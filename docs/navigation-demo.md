# Navigation demo

`eltanin_navigate_on_real_map` closes the loop over every module in a single process. No ROS node, no
tf, no topic, no simulator process.

The design decisions behind the loop and the measured numbers are in the Japanese design note
[integration-design.md](https://github.com/RyuYamamoto/eltanin/blob/main/docs/integration-design.md).

## The pipeline

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

Two costmaps, not one: the global one is what the planner searches, the local one is the window the
limiter predicts the footprint through.

## What the run demonstrates

An obstacle beyond the 3 m sensor range cannot be seen, so the first path runs straight through it.
Only when the robot gets close enough do the cells land in its belief, and that is what triggers the
replan — the robot curves around **without ever stopping**.

Pass `--replan-on-stop-only` to hold the replan back until `VelocityLimiter` has brought the robot to
a standstill instead. That is the mode that exercises the limiter: the interesting output is then the
stop clearance rather than the detour.

## Running it

```bash
cmake -B build -DELTANIN_BUILD_EXAMPLES=ON
cmake --build build -j
./build/examples/eltanin_navigate_on_real_map out_dir
./build/examples/eltanin_navigate_on_real_map out_dir --planner hybrid-astar
./build/examples/eltanin_navigate_on_real_map out_dir --follower mpc --velocity-profile
```

The map defaults to `${ELTANIN_TEST_MAP_DIR}/map.yaml`, so the first command runs as it is. Start and
goal are picked from the map, so no coordinate is hard coded.

An unknown obstacle is stamped onto the ground truth halfway along the planned path *after* planning,
so the robot has to discover it; `--obstacle-fraction 0` gives a clean run instead.

`--help` is not a flag, but any wrong argument prints the full option list.

### Options

| Option | Effect |
|---|---|
| `--map <map.yaml>` | Map to load; defaults to `${ELTANIN_TEST_MAP_DIR}/map.yaml` |
| `--start <x> <y>` | Explicit start; picked from the map when omitted |
| `--goal <x> <y>` | Explicit goal; picked from the map when omitted |
| `--planner <name>` | `astar` (default) or `hybrid-astar` |
| `--follower <name>` | `pure_pursuit` (default) or `mpc` (needs `ELTANIN_ENABLE_MPC=ON`) |
| `--velocity-profile` | Cap the speed by the path curvature |
| `--obstacle-fraction <f>` | Where the unknown obstacle sits along the path; `0` disables it |
| `--obstacle-half-width <n>` | Half width of that obstacle [cells] |
| `--dt <s>` | Control period |
| `--sensor-decimation <n>` | Control cycles between two scans |
| `--replan-on-stop-only` | Replan only after the limiter has stopped the robot, not when the path is blocked |

`--start` and `--goal` are used as a pair; giving only one falls back to picking both from the map.

## Output

The run prints the per-leg statistics, the total cycle count, the final position error and whether the
goal was reached, then writes six files into `out_dir`:

| File | Contents |
|---|---|
| `costmap.pgm` | Cropped global costmap, raw cost values |
| `traversed.pgm` | Same crop and geometry, cells the robot actually occupied |
| `path.csv` | `leg,index,x,y,yaw` for every planned path, replans included |
| `trajectory.csv` | One row per control cycle: pose, requested and limited command, collision distance |
| `obstacles.csv` | `t,x,y` of the cells found occupied that the static map calls free |
| `meta.txt` | Crop geometry, radii, every parameter, per-leg statistics, outcome, stop clearance |

`traversed.pgm` shares the crop and geometry of `costmap.pgm` on purpose, so the two can be diffed or
overlaid pixel for pixel.

**The exit code is 0 only when the goal was reached**; otherwise the failing stage is named on stderr.
That makes the demo usable as a regression test, which is what the `integration` label in
[Build and test](build-and-test.md#integration-tests) runs.

## Plotting

```bash
python3 examples/plot_navigation_results.py --run out_dir --out plots
```

Turns those files into four figures: the whole route over the costmap, a zoom on the unknown obstacle,
the requested against the limited command over time, and the traversed cells over the costmap. Add
`--animate` for a `navigation.gif` that plays the run back next to the local window the limiter sees.

For a Hybrid A\* run, the overview, obstacle zoom and animation draw the footprint at every planned
pose; no footprint thinning is applied.

Like the other plotting scripts it is a developer tool that CMake never refers to, so Python never
becomes a build dependency. What each figure shows, and what reading them revealed, is in
[integration-design.md](https://github.com/RyuYamamoto/eltanin/blob/main/docs/integration-design.md) §15.

## Known gaps

The demo is honest about what it does not do:

- **No local planner.** A detour comes from global replanning only. A blocked path is re-searched from
  scratch rather than locally deformed.
- **The injected obstacle is stationary** after planning, rather than a moving obstacle.

`GoalApproach` handles final deceleration and goal-yaw alignment, so the goal pose is reached rather
than merely passed near.
