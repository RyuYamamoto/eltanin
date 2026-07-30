add_library(eltanin_sim STATIC
  simple_simulator.cpp)
add_library(eltanin::sim ALIAS eltanin_sim)

target_compile_features(eltanin_sim PUBLIC cxx_std_20)
target_include_directories(eltanin_sim PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(eltanin_sim PUBLIC eltanin_core)
target_compile_options(eltanin_sim PRIVATE ${ELTANIN_WARNING_FLAGS})
set_target_properties(eltanin_sim PROPERTIES EXPORT_NAME sim)

install(TARGETS eltanin_sim EXPORT eltaninTargets)
