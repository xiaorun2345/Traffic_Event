cmake_minimum_required(VERSION 3.10)

project(videoDecoder)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE Release)

message("videoDecoder ...")

add_definitions(-std=c++11 -Wwrite-strings)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")


set(CUDA_NVCC_PLAGS ${CUDA_NVCC_PLAGS}; -std=c++11;-g;-G;-gencode;arch=compute_30;code=sm_30)

# 查找 FFmpeg 库
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
	message("embed platform--->opencv default path")
    # set(ENV{PKG_CONFIG_PATH} "../FFmpeg:$ENV{PKG_CONFIG_PATH}")
    #	find_package(PkgConfig REQUIRED)
    #	pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    # libavdevice
    # libavfilter
    # libavformat
    # libavcodec
    # libswresample
    # libswscale
    # libavutil
    #	)
else()
	message("x86_64 platform--->set opencv path")
	
endif()



# #设置opencv的搜索路径
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
	message("embed platform--->opencv default path")
    # set(OpenCV_DIR /usr/lib/aarch64-linux-gnu/cmake/opencv4/)
else()
	message("x86_64 platform--->set opencv path")
	# set(OpenCV_DIR /usr/local/opencv4.2.0/lib/cmake/opencv4)
endif()

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
message("INFO opencv include: '${OpenCV_INCLUDE_DIRS}'")

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/rk3588
    ${CMAKE_CURRENT_SOURCE_DIR}/rk3588/mpp/include
	)
link_directories(
    ${CMAKE_SOURCE_DIR}/lib	
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/mpp/lib
)
message("current lib: ${CMAKE_SOURCE_DIR}/lib")
file(GLOB RK3588SRC ./rk3588/*.cpp )

# 添加可执行文件
add_library(videoDecoder SHARED ${RK3588SRC})

# 链接 FFmpeg 和 OpenCV 库
target_link_libraries(videoDecoder  
	${OpenCV_LIBS}  
	libavcodec.so
	libavformat.so
	libavutil.so
	)
target_link_libraries(videoDecoder rockchip_mpp utils pthread)

