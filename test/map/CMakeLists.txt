add_executable(eltanin_test_map
  test_map_geometry.cpp
  test_grid_map.cpp
  test_cost_model.cpp)

target_link_libraries(eltanin_test_map PRIVATE eltanin::map GTest::gtest_main)
target_compile_options(eltanin_test_map PRIVATE ${ELTANIN_WARNING_FLAGS})

gtest_discover_tests(eltanin_test_map
  PROPERTIES ENVIRONMENT "${ELTANIN_TEST_ENVIRONMENT}")
