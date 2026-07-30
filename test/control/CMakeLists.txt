add_executable(eltanin_test_control
  test_pure_pursuit.cpp
  test_pure_pursuit_tracking.cpp)

target_link_libraries(eltanin_test_control PRIVATE eltanin::control GTest::gtest_main)
target_include_directories(eltanin_test_control PRIVATE ${PROJECT_SOURCE_DIR}/test)
target_compile_options(eltanin_test_control PRIVATE ${ELTANIN_WARNING_FLAGS})

gtest_discover_tests(eltanin_test_control
  PROPERTIES ENVIRONMENT "${ELTANIN_TEST_ENVIRONMENT}")
