cmake_minimum_required(VERSION 3.10)
project(UAV_3D_Mapping)

set(CMAKE_CXX_STANDARD 11)

add_executable(UAV_3D_Mapping main.cpp)

find_library(pcap HINTS "/usr/lib")
include_directories(${pcap_INCLUDE_DIRS})
set(LIBS ${LIBS} ${pcap_LIBRARIES})

target_link_libraries(UAV_3D_Mapping
        libpcap.a
        libpcap.so
        ${CMAKE_DL_LIBS}
        )