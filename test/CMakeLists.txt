find_package(GTest REQUIRED)
include(GoogleTest)

set(ELTANIN_TEST_MAP_DIR
  "/home/pfr-ryu/workspace/eltanin_ws/src/navyu/navyu_navigation/map"
  CACHE PATH "Directory containing map.pgm / map.yaml used by the real-map test")

# UBSan does not abort by default; without this a test with UB would still pass.
set(ELTANIN_TEST_ENVIRONMENT "UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1")

add_subdirectory(core)
add_subdirectory(map)
add_subdirectory(map_io)
add_subdirectory(sensor)
add_subdirectory(planner)
add_subdirectory(control)
