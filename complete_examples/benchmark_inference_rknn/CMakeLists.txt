cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0015 NEW)
cmake_policy(SET CMP0081 NEW)
project(daheng)

include_directories("./include")
include_directories("./include/rga")
include_directories("./include/libyuv")
include_directories( ${CMAKE_SOURCE_DIR}/third-party/cxxopts/include)
find_package(HailoRT REQUIRED)

find_library(RGA_LIB rga HINTS ${CMAKE_SOURCE_DIR}/libs)
find_library(MPP_LIB rockchip_mpp HINTS ${CMAKE_SOURCE_DIR}/libs)
find_library(RKNN_LIB rknnrt HINTS ${CMAKE_SOURCE_DIR}/libs)

message(STATUS "RGA_LIB = ${RGA_LIB}")
message(STATUS "MPP_LIB = ${MPP_LIB}")
message(STATUS "RKNN_LIB = ${RKNN_LIB}")

file(GLOB SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_executable(main ${SRC_FILES})
add_compile_options(-g -O0)
target_link_libraries(main PUBLIC  ${RKNN_LIB} ${MPP_LIB} ${RGA_LIB}  HailoRT::libhailort -pthread -Wno-pedantic)