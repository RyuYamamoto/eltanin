find_package(yaml-cpp REQUIRED)

add_library(eltanin_map_io STATIC
  pgm.cpp
  map_loader.cpp)
add_library(eltanin::map_io ALIAS eltanin_map_io)

target_compile_features(eltanin_map_io PUBLIC cxx_std_20)
target_include_directories(eltanin_map_io PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(eltanin_map_io
  PUBLIC eltanin_map
  PRIVATE yaml-cpp::yaml-cpp)
target_compile_options(eltanin_map_io PRIVATE ${ELTANIN_WARNING_FLAGS})
set_target_properties(eltanin_map_io PROPERTIES EXPORT_NAME map_io)

install(TARGETS eltanin_map_io EXPORT eltaninTargets)
