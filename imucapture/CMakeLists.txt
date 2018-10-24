cmake_minimum_required(VERSION 3.10)
project(imucapture)

set(CMAKE_CXX_STANDARD 11)

add_executable(imucapture main.cpp)

find_library(ncurses HINTS "/usr/lib")
include_directories(${ncurses_INCLUDE_DIRS})
set(LIBS ${LIBS} ${ncurses_LIBRARIES})

target_link_libraries(imucapture
        libncurses.a
        libncurses.so
        
        ${CMAKE_DL_LIBS}
        )