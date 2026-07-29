add_executable(eltanin_test_map_io
  test_pgm.cpp
  test_map_loader.cpp
  test_real_map.cpp)

target_link_libraries(eltanin_test_map_io PRIVATE eltanin::map_io GTest::gtest_main)
target_compile_options(eltanin_test_map_io PRIVATE ${ELTANIN_WARNING_FLAGS})
target_compile_definitions(eltanin_test_map_io PRIVATE
  ELTANIN_TEST_TMP_DIR="${CMAKE_CURRENT_BINARY_DIR}"
  ELTANIN_TEST_MAP_DIR="${ELTANIN_TEST_MAP_DIR}")

gtest_discover_tests(eltanin_test_map_io
  PROPERTIES ENVIRONMENT "${ELTANIN_TEST_ENVIRONMENT}")
