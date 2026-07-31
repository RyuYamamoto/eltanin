add_executable(eltanin_test_integration test_navigate_demo.cpp)

target_link_libraries(eltanin_test_integration PRIVATE
  eltanin::map_io eltanin::map eltanin::sensor eltanin::planner
  eltanin::control eltanin::collision eltanin::sim GTest::gtest_main)
target_include_directories(eltanin_test_integration PRIVATE
  ${PROJECT_SOURCE_DIR}/test ${PROJECT_SOURCE_DIR}/examples)
target_compile_options(eltanin_test_integration PRIVATE ${ELTANIN_WARNING_FLAGS})
target_compile_definitions(eltanin_test_integration PRIVATE
  ELTANIN_TEST_MAP_DIR="${ELTANIN_TEST_MAP_DIR}"
  ELTANIN_TEST_TMP_DIR="${CMAKE_CURRENT_BINARY_DIR}")

# The suite drives the whole stack on a 16 M cell map, so it is minutes rather than milliseconds.
gtest_discover_tests(eltanin_test_integration
  PROPERTIES ENVIRONMENT "${ELTANIN_TEST_ENVIRONMENT}" TIMEOUT 300)
