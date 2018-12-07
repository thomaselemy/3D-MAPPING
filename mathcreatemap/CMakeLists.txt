cmake_minimum_required(VERSION 3.10)
project(mathcreatemap)

set(CMAKE_CXX_STANDARD 14)

add_executable(mathcreatemap main.cpp new_georef.cpp math.hpp new_georef.hpp data.hpp)

target_link_libraries(mathcreatemap pthread)
