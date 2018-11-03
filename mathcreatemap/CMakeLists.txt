cmake_minimum_required(VERSION 3.10)
project(mathcreatemap)

set(CMAKE_CXX_STANDARD 14)

add_executable(mathcreatemap main.cpp georef.cpp math.hpp georef.hpp)

target_link_libraries(mathcreatemap pthread)
