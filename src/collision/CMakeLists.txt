add_library(eltanin_collision STATIC
  collision_checker.cpp
  velocity_limiter.cpp
  velocity_governor.cpp)
add_library(eltanin::collision ALIAS eltanin_collision)

target_compile_features(eltanin_collision PUBLIC cxx_std_20)
target_include_directories(eltanin_collision PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(eltanin_collision PUBLIC eltanin_map)
target_compile_options(eltanin_collision PRIVATE ${ELTANIN_WARNING_FLAGS})
set_target_properties(eltanin_collision PROPERTIES EXPORT_NAME collision)

install(TARGETS eltanin_collision EXPORT eltaninTargets)
