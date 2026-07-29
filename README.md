# eltanin

A non-ROS C++20 library for 2D autonomous navigation.

Core planning, control and map logic depends only on the C++ standard library and Eigen. Anything
that needs ROS 2, visualization or file I/O lives in a separate target.

## Modules

| Target | Alias | Depends on | Contents |
|---|---|---|---|
| `eltanin_core` | `eltanin::core` | Eigen | `Pose2D` / `Twist2D` / `Transform2D`, angle normalization, point-segment geometry, `Polygon2D`, footprint radii, `Traversability`, `Path` |
| `eltanin_map` | `eltanin::map` | `eltanin::core` | `MapGeometry` (the only world/map conversion), `GridMap<T>`, nav2-scale cost constants, cost model |
| `eltanin_map_io` | `eltanin::map_io` | `eltanin::map`, yaml-cpp | PGM + YAML map loading, PGM debug dump |

Design decisions for the costmap, collision classification, error reporting and module layout are
recorded in [docs/costmap-design.md](docs/costmap-design.md).

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
| `ELTANIN_TEST_MAP_DIR` | navyu map directory | Directory holding `map.pgm` / `map.yaml` for the real-map test; the test is skipped when absent |

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

## License

Apache License 2.0. See [LICENSE](LICENSE).
