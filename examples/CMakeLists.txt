add_executable(eltanin_load_map_summary load_map_summary.cpp)
target_link_libraries(eltanin_load_map_summary PRIVATE eltanin::map_io)
target_compile_options(eltanin_load_map_summary PRIVATE ${ELTANIN_WARNING_FLAGS})

add_executable(eltanin_plan_on_real_map plan_on_real_map.cpp)
target_link_libraries(eltanin_plan_on_real_map PRIVATE eltanin::map_io eltanin::planner)
target_compile_options(eltanin_plan_on_real_map PRIVATE ${ELTANIN_WARNING_FLAGS})

add_executable(eltanin_track_on_real_map track_on_real_map.cpp)
target_link_libraries(eltanin_track_on_real_map PRIVATE
  eltanin::map_io eltanin::planner eltanin::control)
target_compile_options(eltanin_track_on_real_map PRIVATE ${ELTANIN_WARNING_FLAGS})
