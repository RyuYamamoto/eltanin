add_executable(eltanin_test_map
  test_map_geometry.cpp
  test_crop.cpp
  test_grid_map.cpp
  test_cost_model.cpp
  test_static_layer.cpp
  test_obstacle_layer.cpp
  test_raytrace_layer.cpp
  test_inflation_layer.cpp
  test_layered_costmap.cpp
  test_inflation_real_map.cpp)

target_link_libraries(eltanin_test_map PRIVATE
  eltanin::map eltanin::map_io GTest::gtest_main)
target_include_directories(eltanin_test_map PRIVATE ${PROJECT_SOURCE_DIR}/test)
target_compile_options(eltanin_test_map PRIVATE ${ELTANIN_WARNING_FLAGS})
target_compile_definitions(eltanin_test_map PRIVATE
  ELTANIN_TEST_TMP_DIR="${CMAKE_CURRENT_BINARY_DIR}"
  ELTANIN_TEST_MAP_DIR="${ELTANIN_TEST_MAP_DIR}")

gtest_discover_tests(eltanin_test_map
  PROPERTIES ENVIRONMENT "${ELTANIN_TEST_ENVIRONMENT}")
