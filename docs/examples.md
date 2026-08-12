# Examples

Six example programs under `examples/`, plus three Python plotting scripts. They are the fastest way
to see a module work on a real map, and every measurement recorded in the design notes was taken
through them.

This guide states what each program takes and writes. The measured results are in the Japanese design
notes, referenced by section number.

## Building the examples

```bash
cmake -B build -DELTANIN_BUILD_EXAMPLES=ON
cmake --build build -j
```

The binaries land in `build/examples/`. Programs that take a `<map.yaml>` need a PGM + YAML occupancy
map; see [Build and test](build-and-test.md#the-reference-map) for the reference map used here.

## `eltanin_load_map_summary`

```bash
./build/examples/eltanin_load_map_summary path/to/map.yaml
```

Prints the map geometry and a histogram of cost values. Use it to check that a map was loaded with
the expected resolution, origin and occupancy thresholds before spending time on a planning run —
a wrong `negate` or threshold in the YAML shows up here as an obviously wrong histogram.

## `eltanin_hybrid_astar_demo`

Loads and inflates a YAML/PGM map for the configured robot footprint, uses an A\* path only to crop
the large input map, then runs Hybrid A\* on that real-map corridor. Start and goal include yaw in
radians:

```bash
./build/examples/eltanin_hybrid_astar_demo \
  path/to/map.yaml /tmp/eltanin-hybrid \
  36.175 -15.925 1.5708 34.675 -9.025 1.5708 [allow_reverse 0|1] [allow_turn_in_place 0|1]
```

The two optional flags select the motion model: with both off the path is Dubins, `allow_reverse`
makes it Reeds-Shepp and `allow_turn_in_place` adds turning on the spot. See
[Using the planners](planner-usage.md#motion-model) for what each one changes about reachability.

Cropping first is not an optimization here, it is required — see
[the memory bound](planner-usage.md#memory-and-cropping).

```bash
python3 examples/plot_hybrid_astar.py /tmp/eltanin-hybrid \
  --out /tmp/eltanin-hybrid/hybrid_astar.png --animate
```

The static figure draws the footprint at every path sample, producing the dense swept-path view
commonly used for Hybrid A\*. `--footprint-step N` thins the outlines for a very long path.
`--animate` also writes `hybrid_astar.gif`, with the footprint moving over the map and a following
local view.

## `eltanin_plan_on_real_map`

```bash
./build/examples/eltanin_plan_on_real_map path/to/map.yaml out_dir
./build/examples/eltanin_plan_on_real_map path/to/map.yaml out_dir 36.175 -15.925 34.675 -9.025
```

Inflates a real map and runs A\* on it. Without explicit poses a reachable start/goal pair is picked
from the map, so no coordinate has to be hard coded. Writes `crop.pgm`, `raw.csv`, `smoothed.csv` and
`meta.txt`, which is enough to see what the smoother did to the raw cell-center path.

## `eltanin_track_on_real_map`

```bash
./build/examples/eltanin_track_on_real_map path/to/map.yaml out_dir
./build/examples/eltanin_track_on_real_map --follower mpc --velocity-profile path/to/map.yaml out_dir
```

Plans on a real map, then tracks the resulting path with the follower named by `--follower`
(`pure_pursuit` by default, `mpc` requires `ELTANIN_ENABLE_MPC=ON`).

| Argument | Effect |
|---|---|
| `--follower <name>` | `pure_pursuit` (default) or `mpc` |
| `--velocity-profile` | Cap the speed by the path curvature |
| `--vmax <v>` | Maximum linear speed [m/s] |
| `--curvature-window <m>` | Sweep the window the curvature estimator that bound is built from uses |
| `[start_x start_y goal_x goal_y]` | Explicit poses; picked from the map when omitted |
| `[lateral_offset]` | Displace the robot sideways from the path start [m] |

`lateral_offset` is the useful one for follower comparison: without it the run only shows the
steady-state error, with it the transient is visible too.

Writes `crop.pgm`, `path.csv`, `trajectory.csv` and `meta.txt`, so the path and the traced trajectory
can be plotted over the costmap. The tracking measurements taken this way are in
[control-design.md](https://github.com/RyuYamamoto/eltanin/blob/main/docs/control-design.md) §5 and §13.8.

## `eltanin_limiter_profile`

```bash
./build/examples/eltanin_limiter_profile out_dir
```

Needs no map. Runs `VelocityLimiter` on synthetic wall and single-obstacle maps and writes:

| File | Contents |
|---|---|
| `velocity_profile.csv` | Limited speed against the gap to the wall, forward **and** reverse, next to what navyu's `std::min()` would have produced |
| `distance_profile.csv` | The terms the limit is built from at each gap: collision distance, time to collision, clearance, proximity scale |
| `closed_loop_*.csv` | The stopping run driven through `SimpleSimulator` |
| `heading_sweep_*.csv` | Collision verdict over a full turn at one pose |
| `bands.pgm` | Per-cell `Free` / `Circumscribed` / `Inscribed` classification |
| `meta.txt` | Parameters and the derived `distance_model` |

The forward/reverse pair is the point of the comparison: the braking-distance law has to bound both
directions, which a plain `std::min()` over the forward distance does not.

## `eltanin_limit_on_real_map`

```bash
./build/examples/eltanin_limit_on_real_map path/to/map.yaml out_dir
./build/examples/eltanin_limit_on_real_map path/to/map.yaml out_dir 36.175 -15.925 34.675 -9.025 0.5
```

Plans a path, tracks it with `PurePursuit`, passes every command through `VelocityLimiter` and drives
`SimpleSimulator` with the limited command. The trailing `obstacle_fraction` stamps a lethal blob
that far along the planned path *after* planning, so the limiter meets an obstacle the planner never
saw; `0` disables it.

Writes `crop.pgm`, `path.csv`, `trajectory.csv`, `predicted.csv`, `footprint.csv` and `meta.txt` —
the last two being the predicted poses and the world-frame footprint the checker actually tested.

## Plotting

Both collision examples write CSV and PGM only; the figures come from a separate script:

```bash
python3 examples/plot_collision_results.py --synthetic out_dir --real out_dir --out plots
```

What each figure shows is described in [collision-design.md](https://github.com/RyuYamamoto/eltanin/blob/main/docs/collision-design.md) §11.

The three scripts — `plot_hybrid_astar.py`, `plot_collision_results.py` and
`plot_navigation_results.py` — need matplotlib, numpy and Pillow (Pillow only for the GIF output).

!!! note "Python is not a build dependency"
    The plotting scripts are deliberately not wired into CMake. Nothing in the build refers to them,
    so a consumer of the library never needs a Python interpreter, and matplotlib/numpy/Pillow stay
    developer-only dependencies that are not linked into any library target.

`plot_navigation_results.py` belongs to the integration demo and is documented in
[Navigation demo](navigation-demo.md#plotting).
