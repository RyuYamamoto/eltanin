add_executable(eltanin_test_sim
  test_simple_simulator.cpp)

target_link_libraries(eltanin_test_sim PRIVATE eltanin::sim GTest::gtest_main)
target_compile_options(eltanin_test_sim PRIVATE ${ELTANIN_WARNING_FLAGS})

gtest_discover_tests(eltanin_test_sim
  PROPERTIES ENVIRONMENT "${ELTANIN_TEST_ENVIRONMENT}")
