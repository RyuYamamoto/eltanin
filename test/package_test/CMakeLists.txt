cmake_minimum_required(VERSION 3.20)
project(eltanin_package_test LANGUAGES CXX)

find_package(eltanin REQUIRED)

add_executable(eltanin_package_test main.cpp)
target_link_libraries(eltanin_package_test PRIVATE
  eltanin::core
  eltanin::map
  eltanin::map_io
  eltanin::sensor
  eltanin::planner)
