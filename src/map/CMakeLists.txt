add_library(eltanin_map STATIC
  cost_model.cpp
  distance_map.cpp
  layered_costmap.cpp
  layers/static_layer.cpp
  layers/obstacle_layer.cpp
  layers/raytrace_layer.cpp
  layers/inflation_layer.cpp)
add_library(eltanin::map ALIAS eltanin_map)

target_compile_features(eltanin_map PUBLIC cxx_std_20)
target_include_directories(eltanin_map PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(eltanin_map PUBLIC eltanin_core)
target_compile_options(eltanin_map PRIVATE ${ELTANIN_WARNING_FLAGS})
set_target_properties(eltanin_map PROPERTIES EXPORT_NAME map)

install(TARGETS eltanin_map EXPORT eltaninTargets)
