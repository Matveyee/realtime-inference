cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0015 NEW)
cmake_policy(SET CMP0081 NEW)
project(daheng)

include_directories("./include")
include_directories("./include/rga")
include_directories("./include/libyuv")
include_directories( /usr/include/libdrm)
# include_directories( ${CMAKE_SOURCE_DIR}/third-party/Modbus/include)
include_directories( ${CMAKE_SOURCE_DIR}/third-party/cxxopts/include)
find_package(HailoRT REQUIRED)

add_subdirectory(third-party/Modbus)


find_library(GX_LIB gxiapi HINTS ${CMAKE_SOURCE_DIR}/libs)
find_library(DRM_LIB drm HINTS /usr/lib /usr/local/lib)
find_library(RGA_LIB rga HINTS ${CMAKE_SOURCE_DIR}/libs)
find_library(RKNN_LIB rknnrt HINTS ${CMAKE_SOURCE_DIR}/libs)
find_library(YUV_LIB yuv HINTS ${CMAKE_SOURCE_DIR}/libs)
find_library(MPP_LIB rockchip_mpp HINTS ${CMAKE_SOURCE_DIR}/libs)

message(STATUS "GX_LIB = ${GX_LIB}")
message(STATUS "DRM_LIB = ${DRM_LIB}")
message(STATUS "RGA_LIB = ${RGA_LIB}")
message(STATUS "RKNN_LIB = ${RKNN_LIB}")
message(STATUS "YUV_LIB = ${YUV_LIB}")
message(STATUS "MPP_LIB = ${MPP_LIB}")


file(GLOB SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_executable(main ${SRC_FILES})
add_compile_options(-g -O0)
target_link_libraries(main PUBLIC ${YUV_LIB} ${MPP_LIB} ${GX_LIB} ${DRM_LIB} ${RGA_LIB} ${RKNN_LIB} HailoRT::libhailort -pthread -Wno-pedantic)
