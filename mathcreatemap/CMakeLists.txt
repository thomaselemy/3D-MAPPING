cmake_minimum_required(VERSION 3.10)
project(mathcreatemap)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")

if(COMPILER_SUPPORTS_MARCH_NATIVE)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif()

add_executable(mathcreatemap main.cpp new_georef.cpp math.hpp new_georef.hpp data.hpp)

target_link_libraries(mathcreatemap pthread)
