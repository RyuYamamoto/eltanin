add_library(eltanin_core STATIC
  geometry.cpp
  polygon.cpp
  footprint.cpp
  path.cpp
  differential_drive.cpp)
add_library(eltanin::core ALIAS eltanin_core)

target_compile_features(eltanin_core PUBLIC cxx_std_20)
target_include_directories(eltanin_core PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(eltanin_core PUBLIC Eigen3::Eigen)
target_compile_options(eltanin_core PRIVATE ${ELTANIN_WARNING_FLAGS})
set_target_properties(eltanin_core PROPERTIES EXPORT_NAME core)

install(TARGETS eltanin_core EXPORT eltaninTargets)
