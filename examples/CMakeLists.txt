add_executable(eltanin_load_map_summary load_map_summary.cpp)
target_link_libraries(eltanin_load_map_summary PRIVATE eltanin::map_io)
target_compile_options(eltanin_load_map_summary PRIVATE ${ELTANIN_WARNING_FLAGS})

add_executable(eltanin_plan_on_real_map plan_on_real_map.cpp)
target_link_libraries(eltanin_plan_on_real_map PRIVATE eltanin::map_io eltanin::planner)
target_include_directories(eltanin_plan_on_real_map PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(eltanin_plan_on_real_map PRIVATE ${ELTANIN_WARNING_FLAGS})

add_executable(eltanin_track_on_real_map track_on_real_map.cpp)
target_link_libraries(eltanin_track_on_real_map PRIVATE
  eltanin::map_io eltanin::planner eltanin::control)
target_include_directories(eltanin_track_on_real_map PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(eltanin_track_on_real_map PRIVATE ${ELTANIN_WARNING_FLAGS})

add_executable(eltanin_limiter_profile limiter_profile.cpp)
target_link_libraries(eltanin_limiter_profile PRIVATE
  eltanin::map_io eltanin::collision eltanin::sim)
target_compile_options(eltanin_limiter_profile PRIVATE ${ELTANIN_WARNING_FLAGS})

add_executable(eltanin_limit_on_real_map limit_on_real_map.cpp)
target_link_libraries(eltanin_limit_on_real_map PRIVATE
  eltanin::map_io eltanin::planner eltanin::control eltanin::collision eltanin::sim)
target_include_directories(eltanin_limit_on_real_map PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(eltanin_limit_on_real_map PRIVATE ${ELTANIN_WARNING_FLAGS})

add_executable(eltanin_navigate_on_real_map navigate_on_real_map.cpp)
target_link_libraries(eltanin_navigate_on_real_map PRIVATE
  eltanin::map_io eltanin::map eltanin::sensor eltanin::planner
  eltanin::control eltanin::collision eltanin::sim)
target_include_directories(eltanin_navigate_on_real_map PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(eltanin_navigate_on_real_map PRIVATE ${ELTANIN_WARNING_FLAGS})
target_compile_definitions(eltanin_navigate_on_real_map PRIVATE
  ELTANIN_TEST_MAP_DIR="${ELTANIN_TEST_MAP_DIR}")
