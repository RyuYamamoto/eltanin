add_executable(eltanin_test_collision
  test_collision_checker.cpp
  test_velocity_limiter.cpp
  test_velocity_limiter_closed_loop.cpp
  test_velocity_governor.cpp)

target_link_libraries(eltanin_test_collision PRIVATE
  eltanin::collision eltanin::sim eltanin::map GTest::gtest_main)
target_include_directories(eltanin_test_collision PRIVATE ${PROJECT_SOURCE_DIR}/test)
target_compile_options(eltanin_test_collision PRIVATE ${ELTANIN_WARNING_FLAGS})

gtest_discover_tests(eltanin_test_collision
  PROPERTIES ENVIRONMENT "${ELTANIN_TEST_ENVIRONMENT}")
