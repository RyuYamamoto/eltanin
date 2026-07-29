add_executable(eltanin_test_core
  test_angle.cpp
  test_types.cpp
  test_geometry.cpp
  test_polygon.cpp
  test_footprint.cpp
  test_path.cpp)

target_link_libraries(eltanin_test_core PRIVATE eltanin::core GTest::gtest_main)
target_compile_options(eltanin_test_core PRIVATE ${ELTANIN_WARNING_FLAGS})

gtest_discover_tests(eltanin_test_core
  PROPERTIES ENVIRONMENT "${ELTANIN_TEST_ENVIRONMENT}")
