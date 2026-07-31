add_library(eltanin_sensor STATIC
  scan_projection.cpp
  scan_clearing.cpp)
add_library(eltanin::sensor ALIAS eltanin_sensor)

target_compile_features(eltanin_sensor PUBLIC cxx_std_20)
target_include_directories(eltanin_sensor PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(eltanin_sensor PUBLIC eltanin_core)
target_compile_options(eltanin_sensor PRIVATE ${ELTANIN_WARNING_FLAGS})
set_target_properties(eltanin_sensor PROPERTIES EXPORT_NAME sensor)

install(TARGETS eltanin_sensor EXPORT eltaninTargets)
