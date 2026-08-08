add_executable(eltanin_test_planner
  test_traversable_search.cpp
  test_astar_planner.cpp
  test_astar_bounds.cpp
  test_dubins_path.cpp
  test_reeds_shepp_path.cpp
  test_hybrid_astar_planner.cpp
  test_planner_errors.cpp
  test_path_smoother.cpp
  test_planner_seam.cpp
  test_planner_real_map.cpp)

target_link_libraries(eltanin_test_planner PRIVATE
  eltanin::planner eltanin::map_io GTest::gtest_main)
target_include_directories(eltanin_test_planner PRIVATE ${PROJECT_SOURCE_DIR}/test)
target_compile_options(eltanin_test_planner PRIVATE ${ELTANIN_WARNING_FLAGS})
target_compile_definitions(eltanin_test_planner PRIVATE
  ELTANIN_TEST_MAP_DIR="${ELTANIN_TEST_MAP_DIR}")

gtest_discover_tests(eltanin_test_planner
  PROPERTIES ENVIRONMENT "${ELTANIN_TEST_ENVIRONMENT}")
