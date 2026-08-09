add_library(eltanin_control STATIC
  goal_approach.cpp
  path_follower.cpp
  pure_pursuit.cpp
  velocity_profile.cpp)
add_library(eltanin::control ALIAS eltanin_control)

target_compile_features(eltanin_control PUBLIC cxx_std_20)
target_include_directories(eltanin_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(eltanin_control PUBLIC eltanin_core)
target_compile_options(eltanin_control PRIVATE ${ELTANIN_WARNING_FLAGS})
set_target_properties(eltanin_control PROPERTIES EXPORT_NAME control)

if(ELTANIN_ENABLE_MPC)
  target_sources(eltanin_control PRIVATE
    mpc_problem.cpp
    qp_solver.cpp
    qp_solver_osqp.cpp)
  # PUBLIC so that a downstream translation unit compiles the headers exactly as we did.
  target_compile_definitions(eltanin_control PUBLIC ELTANIN_WITH_MPC)
  target_link_libraries(eltanin_control PRIVATE ${ELTANIN_OSQP_TARGET})
endif()

install(TARGETS eltanin_control EXPORT eltaninTargets)
