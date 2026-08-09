add_executable(eltanin_test_control
  test_goal_approach.cpp
  test_path_follower.cpp
  test_pure_pursuit.cpp
  test_pure_pursuit_tracking.cpp
  test_velocity_profile.cpp
  test_velocity_profile_tracking.cpp)

if(ELTANIN_ENABLE_MPC)
  target_sources(eltanin_test_control PRIVATE test_qp_solver.cpp)
  # The QP abstraction stays out of the public headers, so the test reaches into src/.
  target_include_directories(eltanin_test_control PRIVATE ${PROJECT_SOURCE_DIR}/src/control)
endif()

target_link_libraries(eltanin_test_control PRIVATE eltanin::control GTest::gtest_main)
target_include_directories(eltanin_test_control PRIVATE ${PROJECT_SOURCE_DIR}/test)
target_compile_options(eltanin_test_control PRIVATE ${ELTANIN_WARNING_FLAGS})

gtest_discover_tests(eltanin_test_control
  PROPERTIES ENVIRONMENT "${ELTANIN_TEST_ENVIRONMENT}")
