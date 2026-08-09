cmake_minimum_required(VERSION 3.20)
project(eltanin VERSION 0.1.0 LANGUAGES CXX)

# --------------------------------------------------------------------------
# Options
# --------------------------------------------------------------------------
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(ELTANIN_IS_TOP_LEVEL ON)
else()
  set(ELTANIN_IS_TOP_LEVEL OFF)
endif()

option(ELTANIN_BUILD_TESTS "Build eltanin unit tests" ${ELTANIN_IS_TOP_LEVEL})
option(ELTANIN_BUILD_EXAMPLES "Build eltanin examples" OFF)
option(ELTANIN_ENABLE_WERROR "Treat compiler warnings as errors" OFF)
# OFF keeps eltanin_control on the stdlib and Eigen only; ON adds the OSQP dependency the MPC needs.
option(ELTANIN_ENABLE_MPC "Build the MPC path follower and its QP solver backend" OFF)
set(ELTANIN_MPC_SOLVER_PROVIDER "fetch" CACHE STRING
  "Where OSQP comes from when ELTANIN_ENABLE_MPC is ON: fetch or package")
option(ELTANIN_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(ELTANIN_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)

# Defined outside ELTANIN_BUILD_TESTS so that examples/ can use it as its default map too.
set(ELTANIN_TEST_MAP_DIR
  "/home/pfr-ryu/workspace/eltanin_ws/src/navyu/navyu_navigation/map"
  CACHE PATH "Directory containing map.pgm / map.yaml used by the real-map tests and the demo")

# CMAKE_BUILD_TYPE is intentionally left empty by default so that NDEBUG is not
# defined and the assert()-based precondition checks stay active. Pass
# -DCMAKE_BUILD_TYPE=Release explicitly for optimized builds.

# Warning flags applied per target (PRIVATE) by each module CMakeLists.
set(ELTANIN_WARNING_FLAGS -Wall -Wextra -Wpedantic)
if(ELTANIN_ENABLE_WERROR)
  list(APPEND ELTANIN_WARNING_FLAGS -Werror)
endif()

# Sanitizers are applied with directory scope: they must not leak into the
# exported target interface (install(EXPORT) would then require exporting an
# extra INTERFACE library).
if(ELTANIN_ENABLE_ASAN)
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address)
endif()
if(ELTANIN_ENABLE_UBSAN)
  add_compile_options(-fsanitize=undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=undefined)
endif()

# --------------------------------------------------------------------------
# Dependencies
# --------------------------------------------------------------------------
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

include(GNUInstallDirs)

if(ELTANIN_ENABLE_MPC)
  if(ELTANIN_MPC_SOLVER_PROVIDER STREQUAL "package")
    find_package(osqp REQUIRED)
    set(ELTANIN_OSQP_TARGET osqp::osqpstatic)
  elseif(ELTANIN_MPC_SOLVER_PROVIDER STREQUAL "fetch")
    # Forced so that a stale cache from a previous configure cannot turn the demo or tests back on.
    set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
    set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL "" FORCE)
    set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
    set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
    set(OSQP_ALGEBRA_BACKEND "builtin" CACHE STRING "" FORCE)
    set(OSQP_ENABLE_INTERRUPT OFF CACHE BOOL "" FORCE)
    set(OSQP_ENABLE_PRINTING OFF CACHE BOOL "" FORCE)
    set(OSQP_ENABLE_PROFILING ON CACHE BOOL "" FORCE)
    include(FetchContent)
    FetchContent_Declare(osqp
      GIT_REPOSITORY https://github.com/osqp/osqp.git
      GIT_TAG v1.0.0
      GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(osqp)
    set(ELTANIN_OSQP_TARGET osqpstatic)
  else()
    message(FATAL_ERROR
      "ELTANIN_MPC_SOLVER_PROVIDER must be fetch or package, got ${ELTANIN_MPC_SOLVER_PROVIDER}")
  endif()
endif()

# --------------------------------------------------------------------------
# Modules
# --------------------------------------------------------------------------
add_subdirectory(src/core)
add_subdirectory(src/map)
add_subdirectory(src/map_io)
add_subdirectory(src/sensor)
add_subdirectory(src/planner)
add_subdirectory(src/control)
add_subdirectory(src/sim)
add_subdirectory(src/collision)

if(ELTANIN_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

if(ELTANIN_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# --------------------------------------------------------------------------
# Install / export
# --------------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
  EXPORT eltaninTargets
  FILE eltaninTargets.cmake
  NAMESPACE eltanin::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/eltanin)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${PROJECT_SOURCE_DIR}/cmake/eltaninConfig.cmake.in
  ${PROJECT_BINARY_DIR}/eltaninConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/eltanin)
write_basic_package_version_file(
  ${PROJECT_BINARY_DIR}/eltaninConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

install(
  FILES
    ${PROJECT_BINARY_DIR}/eltaninConfig.cmake
    ${PROJECT_BINARY_DIR}/eltaninConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/eltanin)
