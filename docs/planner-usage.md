# Using the planners

How to select a planner, what its output guarantees, and the four contracts that are easy to get
wrong. The design reasoning and the measurements behind every number here are in the Japanese design
note [planner-design.md](https://github.com/RyuYamamoto/eltanin/blob/main/docs/planner-design.md),
referenced by section.

!!! warning "`TraversabilityFallback` is unsafe to enable blindly"
    It can return a path that puts the footprint through a wall. Hybrid A\* with a footprint is the
    correct answer for tight doorways. See [Tight doorways](#tight-doorways) before switching it on.

## Selecting a planner

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
functions — use them when the planner is fixed at the call site and the interface buys nothing.

Both return a `PlanResult`: an `std::optional`-compatible value that also carries a `PlannerError`
explaining why there is no path. `to_string(result.error())` names it, so a failed plan can be logged
without a table of error codes at the call site.

Which one to reach for:

| | A\* | Hybrid A\* |
|---|---|---|
| State | 8-connected cell | `(x, y, yaw)` |
| Output respects a turning radius | No | Yes |
| Goal heading | Ignored | Honoured |
| Runs on a full map | Yes | **No** — [needs a crop](#memory-and-cropping) |
| Smoothed | Yes, by default | Never |

## Do not call `smooth()` on a planner result

Whichever planner is selected, the returned path is directly followable — **do not call `smooth()` on
it**.

`plan_astar()` already smooths, reusing the classified grid it built for the search, so a second
`smooth()` only adds a full-map classification pass worth roughly one search. Pass
`AStarParams::smoother = std::nullopt` when the raw cell-center path is what you want.

Hybrid A\* is never smoothed, because that would break the curvature bound its whole search exists to
respect.

`smooth()` remains the right call for paths that did not come from a planner — a recorded trajectory,
a hand-authored waypoint list.

## Hybrid A\*

### Analytic expansion

Hybrid A\* tries its analytic Dubins connection at the first expanded node and then on a throttle
that tightens as the goal gets closer, so on an open map the returned path *is* the optimal Dubins
path rather than a heading-quantized weave.

`analytic_expansion_ratio` controls the throttle. Raising it above the default tries more often, which
measurably degrades detours around obstacles — the analytic tail is attempted from nodes where it
cannot succeed, and the search budget goes with it.

### Motion model

`motion_model` declares what the vehicle can do, and the returned path contains nothing else:

| Value | Contains |
|---|---|
| `Dubins` | Forward arcs at or above the turning radius |
| `Differential` | Adds turning on the spot, which is what a differential drive actually does |
| `ReedsShepp` | Adds driving in reverse |

The difference is not cosmetic. On the reference map a requested goal heading is reachable in 3 of 8
directions under `Dubins` and in 8 of 8 under `Differential`, both with the same
`1 / minimum_turning_radius` bound on every travelling segment.

Two consequences for a follower:

- A turn on the spot appears as its own pose, so two consecutive poses can share a position. That is
  the one place the pose-spacing contract does not apply.
- Under `ReedsShepp` every cusp is a pose of its own, so a follower can see where the gear changes.
  `reverse_penalty` multiplies the cost of reversing and `direction_change_penalty` charges each gear
  change; both apply to the analytic tail as well as to the search.

### Clearance cost and smoother weights

Both planners charge for travelling close to a wall through `ClearanceCost{penalty, distance}`, which
adds `penalty * max(0, 1 - clearance / distance)` per metre travelled.

It is **on for Hybrid A\*** (`{0.3, 0.2}`, which runs on a corridor where the distance field is
cheap) and **off for A\***, because A\* runs on the full map and the distance field there costs more
than the search itself.

The path smoother carries the same idea locally: `weight_obstacle` / `obstacle_distance` push points
off walls and `weight_curvature` penalises bending, using a distance field built only over a window
around the path. [planner-design.md](https://github.com/RyuYamamoto/eltanin/blob/main/docs/planner-design.md) §14.4 and §14.5 record what each weight buys
and what it costs.

## Tight doorways

When the `Free` cells leave start and goal disconnected — a doorway the robot barely fits through —
there are two answers, and only one of them is safe.

### `TraversabilityFallback` (unsafe)

It retries once with `Traversability::Circumscribed` opened up, charging its `penalty` per metre of
band used so it crosses where the band is thinnest, and marks the result with `PlanResult::relaxed()`.

!!! danger "Off by default, and unsafe to switch on blindly"
    `Circumscribed` means a collision is *possible depending on heading*, and A\* only checks the
    vehicle reference point. The returned path can therefore put the footprint through a wall.

    Measured on a 0.44 x 0.30 m robot, a 0.30 m gap yields a path where **10 of 101 poses (A\*) or
    9 of 72 (Hybrid A\*) collide**.

    If you enable it anyway, verify every pose with `collision::check_footprint_exact()` before
    following. [planner-design.md](https://github.com/RyuYamamoto/eltanin/blob/main/docs/planner-design.md) §14.6 has the numbers.

### Footprint-aware Hybrid A\* (preferred)

Give `HybridAStarParams::footprint` the body outline and the search stops treating the inflation band
as a wall:

- a `Free` cell is taken as before,
- a `Circumscribed` cell is accepted only when the outline *at that heading* clears every obstacle
  cell,
- an `Inscribed` cell is always refused.

Since `Free` needs `2 x circumscribed_radius` of corridor, this is the difference between planning and
not planning. For a 0.387 x 0.240 m body (`circumscribed_radius` 0.266 m) a 0.30 m corridor is
`Unreachable` without the outline and drivable with **zero** footprint collisions with it, while a
0.20 m corridor stays `Unreachable` because the body genuinely does not fit.

On an open map the path is bit-identical either way, so the check costs nothing where it is not
needed. This needs no fallback at all — prefer it.

## Memory and cropping

Hybrid A\* holds `(cell, heading_bin)` search state, so its memory grows with cells times
`heading_bins`: about 8.125 bytes per state, or 25 MB for a 10 m square map at 0.05 m and 72 bins.

`max_state_memory_bytes` (256 MiB by default) rejects anything larger with `StateSpaceTooLarge` rather
than throwing, which means **a full 4000 x 4000 map does not fit**.

Crop a corridor around a raw A\* guide first with `map::crop_around()`, as
`eltanin_hybrid_astar_demo` and the navigation loop both do. On the 4000 x 4000 reference map that
turns 1.15e9 states into 5.8e6 and plans in about 380 ms.

[planner-design.md](https://github.com/RyuYamamoto/eltanin/blob/main/docs/planner-design.md) §13 and §14 record the measurements behind these numbers.
