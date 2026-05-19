cmake_minimum_required(VERSION 3.10)

project(camera_app)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE Release)

message("camera application...")

add_definitions(-O2)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-error=deprecated-declarations -Wno-deprecated-declarations")
set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# rknn api
set(RKNN_API_PATH ${CMAKE_SOURCE_DIR}/3rdparty/RK3588/librknn_api)
set(RKNN_RT_LIB ${RKNN_API_PATH}/aarch64/librknn_api.so)
include_directories(${RKNN_API_PATH}/include)
#set(RKNN_API_PATH ${CMAKE_SOURCE_DIR}/../../runtime/RK3588/${CMAKE_SYSTEM_NAME}/librknn_api)
#set(LIB_ARCH aarch64)
#set(RKNN_RT_LIB /usr/lib/librknnrt.so)
#include_directories(${RKNN_API_PATH}/include)
#include_directories(${CMAKE_SOURCE_DIR}/include/3rdparty)

#rga
set(RGA_PATH ${CMAKE_SOURCE_DIR}/3rdparty/librga-1.3.2_release)
set(RGA_LIB ${RGA_PATH}/libs/Linux/gcc-aarch64/librga.so)
include_directories(${RGA_PATH}/include)

#libconfig
set(LIBCONFIG_PATH ${CMAKE_SOURCE_DIR}/3rdparty/aarch64/libconfig)
include_directories(${LIBCONFIG_PATH}/include)
link_directories(${LIBCONFIG_PATH}/lib)

message("build camera_app lib on embed_platform.....")

set(CURRENT_LIB ${CMAKE_SOURCE_DIR}/output/lib)

#set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/3rdparty/opencv_armhf/install/share/OpenCV)
find_package(OpenCV)
include_directories(OpenCV_INCLUDE_DIRS)
message("INFO opencv include: '${OpenCV_INCLUDE_DIRS}'")

#依赖头文件路径
include_directories(
    #${CMAKE_CURRENT_SOURCE_DIR}/common/freetype/freetype2/
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/aarch64/eigen/include/eigen3/
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/proto
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${OpenCV_INCLUDE_DIRS}
    ${RGA_PATH}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/mpp/include 
    ${CMAKE_CURRENT_SOURCE_DIR}/RtspServer/ 
    ${CMAKE_CURRENT_SOURCE_DIR}/RtspServer/3rdpart 
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/mpp/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib/)

file(GLOB_RECURSE RTSPSRC ${CMAKE_CURRENT_SOURCE_DIR}/RtspServer/*.cpp)

#依赖库函数路径
link_directories(
    ${CURRENT_LIB}/
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/mpp/lib/
)


file(GLOB SRC ./app/*.cpp)
file(GLOB UTILS ./common/*.cpp)
file(GLOB_RECURSE DET_SRCS ./detector/*.cpp )
file(GLOB_RECURSE LOC_SRCS ./location/*.cpp)
file(GLOB_RECURSE TRACK_SRCS ./tracker/ByteTrack/*.cpp ./tracker/ByteTrack/*.hpp)


add_library(camera_app SHARED ${SRC} ${DET_SRCS} ${TRACK_SRCS} ${UTILS} ${LOC_SRCS})

target_link_libraries(camera_app ${RKNN_RT_LIB} ${RGA_LIB} ${OpenCV_LIBS})
target_link_libraries(camera_app config++ "stdc++fs")
target_link_libraries(camera_app ${OpenCV_LIBS})
target_link_libraries(camera_app pthread -lm -lrt) 
target_link_libraries(camera_app protobuf)
target_link_libraries(camera_app rockchip_mpp utils pthread)
target_link_libraries(camera_app avformat avcodec avutil)

# install target and libraries
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../output)
install(PROGRAMS ${RKNN_RT_LIB} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install(PROGRAMS ${RGA_LIB} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install(PROGRAMS ${RKNN_API_PATH}/armhf/librknnrt.so DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

# #测试程序
if(0)

    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/output)
    link_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/../lib
    )
    add_executable(camera_app_demo 
        ./test/test_camera_app.cpp 
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/nebulalink.perceptron3.0.5.pb.cc
        ./detector/MppEncoder.cpp 
        ./detector/MppDecoder.cpp 
        ${RTSPSRC}
    )
    target_link_libraries(camera_app_demo ${RKNN_RT_LIB} ${RGA_LIB} ${OpenCV_LIBS})
    target_link_libraries(camera_app_demo camera_app ${OpenCV_LIBS})
    target_link_libraries(camera_app_demo protobuf)
    target_link_libraries(camera_app_demo rockchip_mpp utils pthread)
    target_link_libraries(camera_app_demo avformat avcodec avutil)

endif()



