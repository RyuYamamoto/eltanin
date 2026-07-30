add_executable(eltanin_test_sensor
  test_scan_projection.cpp
  test_scan_to_costmap.cpp)

target_link_libraries(eltanin_test_sensor PRIVATE
  eltanin::sensor eltanin::map GTest::gtest_main)
target_compile_options(eltanin_test_sensor PRIVATE ${ELTANIN_WARNING_FLAGS})

gtest_discover_tests(eltanin_test_sensor
  PROPERTIES ENVIRONMENT "${ELTANIN_TEST_ENVIRONMENT}")
