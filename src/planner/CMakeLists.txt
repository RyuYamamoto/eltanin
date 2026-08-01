add_library(eltanin_planner STATIC
  astar_planner.cpp
  dubins_path.cpp
  hybrid_astar_planner.cpp
  path_smoother.cpp)
add_library(eltanin::planner ALIAS eltanin_planner)

target_compile_features(eltanin_planner PUBLIC cxx_std_20)
target_include_directories(eltanin_planner PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(eltanin_planner PUBLIC eltanin_core eltanin_map)
target_compile_options(eltanin_planner PRIVATE ${ELTANIN_WARNING_FLAGS})
set_target_properties(eltanin_planner PROPERTIES EXPORT_NAME planner)

install(TARGETS eltanin_planner EXPORT eltaninTargets)
