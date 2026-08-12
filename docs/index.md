# eltanin

A non-ROS C++20 library for 2D autonomous navigation.

Core planning, control and map logic depends only on the C++ standard library and Eigen. Anything
that needs ROS 2, visualization or file I/O lives in a separate target.

Source: [github.com/RyuYamamoto/eltanin](https://github.com/RyuYamamoto/eltanin)

## Modules

<!-- Keep this table in sync with the one in README.md. -->

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

## Getting started

```bash
cmake -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Then read [Build and test](build-and-test.md) for the options, and [Examples](examples.md) to see a
module run on a real map.

## Guides

Written in English, for anyone using the library.

- [Build and test](build-and-test.md) — build types, all CMake options, MPC and sanitizer builds,
  tests, install and `find_package`
- [Examples](examples.md) — the six example programs and the Python plotting scripts
- [Using the planners](planner-usage.md) — planner selection, motion models, the smoothing and
  footprint contracts, and the memory bound Hybrid A\* imposes
- [Navigation demo](navigation-demo.md) — `eltanin_navigate_on_real_map`: options, output, known gaps

## Design notes

These record *why* each decision was made and what was measured, in the order the work happened —
not how to use the API. **They are written in Japanese and are not part of this site**; they live in
the repository and are read on GitHub. They will be added here once translated.

- [costmap-design.md](https://github.com/RyuYamamoto/eltanin/blob/main/docs/costmap-design.md) — costmap, collision classification, error reporting, module
  layout
- [sensor-design.md](https://github.com/RyuYamamoto/eltanin/blob/main/docs/sensor-design.md) — scan projection and filtering
- [planner-design.md](https://github.com/RyuYamamoto/eltanin/blob/main/docs/planner-design.md) — A\*, Hybrid A\*, smoothing, clearance cost
- [control-design.md](https://github.com/RyuYamamoto/eltanin/blob/main/docs/control-design.md) — pure pursuit, MPC, velocity profile, goal approach
- [collision-design.md](https://github.com/RyuYamamoto/eltanin/blob/main/docs/collision-design.md) — footprint checking and the velocity limiter
- [integration-design.md](https://github.com/RyuYamamoto/eltanin/blob/main/docs/integration-design.md) — the demo that runs all of them together

## License

Apache License 2.0. See
[LICENSE](https://github.com/RyuYamamoto/eltanin/blob/main/LICENSE).
