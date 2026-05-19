cmake_minimum_required(VERSION 3.0.0)

project(camera_person_warning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)


add_definitions(-std=c++11 -Wno-narrowing -O0 -pthread)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")


# 设置生成库的保存路径及依赖库路径
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
	message("embed platform")
	set(LIB_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
	set(CMAKE_PREFIX_PATH /usr/lib/aarch64-linux-gnu/)
else()
	message("x86_64 platform")
	set(LIB_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib_x86)
	# set(CONFIG_LIB_PATH /usr/local/libconfig-1.7.2)
	# set(PROTO_LIB_PATH /usr/local/protobuf-3.8.0)
	set(PROTO_LIB_PATH /home/user/tools/protobuf-3.8.0)
	set(OpenCV_DIR /usr/local/opencv4.2.0/lib/cmake/opencv4)
	include_directories( /usr/local/include )
    link_directories( /usr/local/lib )
	link_directories( /usr/lib )
	include_directories( /usr/include )
endif()


find_package(OpenCV  REQUIRED)
find_package(Threads)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)


# 设置可行性文件保存路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/output) 

###########################摄像机配置############################
set(CAMERA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/camera)
set(CAMERA_NAME "camera")
add_subdirectory(${CAMERA_NAME})
set_target_properties(
   camera_app 
   PROPERTIES LIBRARY_OUTPUT_DIRECTORY   ${LIB_OUTPUT_PATH}
)

##########################拉流配置################################
# set(DECODER_MANUFACTURE 1)
# if(DECODER_MANUFACTURE MATCHES 0)
set(DECODER_PATH ${CMAKE_CURRENT_SOURCE_DIR}/decoder)
set(DECODER_LIB videoDecoder)
# set(DECODER_NAME "videoDecoder")

add_subdirectory("decoder")
set_target_properties(
	${DECODER_LIB}
	PROPERTIES LIBRARY_OUTPUT_DIRECTORY  ${LIB_OUTPUT_PATH}
)


#######################视觉预警主进程配置############################
include_directories(  
	${CMAKE_CURRENT_SOURCE_DIR}/common/
	${CMAKE_CURRENT_SOURCE_DIR}/src/
	${CAMERA_PATH}/app/
	${CMAKE_CURRENT_SOURCE_DIR}/common/eigen/include/eigen3/
	${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/jsoncpp/include/
	/usr/local/include/jetson-utils/
	${CMAKE_CURRENT_SOURCE_DIR}/common/freetype/freetype2/
	${OpenCV_INCLUDE_DIRS}
	${GST_INCLUDE_DIRS}
	${CONFIG_LIB_PATH}/include
	${PROTO_LIB_PATH}/include

	${DECODER_PATH}/rk3588
	${DECODER_PATH}/rk3588/mpp/include
	${CMAKE_CURRENT_SOURCE_DIR}/ffmedia_release/
	${CMAKE_CURRENT_SOURCE_DIR}/ffmedia_release/include
	${CAMERA_PATH}
	${CAMERA_PATH}/RtspServer
	${CAMERA_PATH}/RtspServer/3rdpart
)

link_directories( 
	${LIB_OUTPUT_PATH}
	${GST_LIBRARIES}
	/usr/lib/aarch64-linux-gnu/gupnp-dlna/
	${CONFIG_LIB_PATH}/lib
	${PROTO_LIB_PATH}/lib
	${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/mpp/lib
	${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/jsoncpp/lib/arm64
	${CMAKE_CURRENT_SOURCE_DIR}/ffmedia_release/lib
	${CMAKE_CURRENT_SOURCE_DIR}/ffmedia_release/inference_examples/lib/RK3588
)


file(GLOB_RECURSE RTSP_SERVER_SRCS ${CAMERA_PATH}/RtspServer/*.cpp)

set(SOURCES_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/DealRCF.cpp
				  ${CMAKE_CURRENT_SOURCE_DIR}/src/LedScreen.cpp
				  #${CMAKE_CURRENT_SOURCE_DIR}/src/SerialPortCommunication.cpp
				  ${CMAKE_CURRENT_SOURCE_DIR}/common/nebulalink.perceptron3.0.5.pb.cc
				  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
				  ${RTSP_SERVER_SRCS})



add_executable(cw_DealRCF_nebulalink ${SOURCES_FILES})

target_link_libraries(cw_DealRCF_nebulalink 
					protobuf pthread 
					config++ 
					camera_app 
					${EXTRA_LIBS}
					${DECODER_LIB}
					${CMAKE_THREAD_LIBS_INIT}
					${OpenCV_LIBS}
					${GST_LIBRARIES}
					-lgstreamer-1.0 -lgobject-2.0 -lglib-2.0 -lgstapp-1.0
					ff_media 
					curl
					crypto
    				jsoncpp
					
					avformat avcodec avutil)



if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
	target_link_libraries(cw_DealRCF_nebulalink
		# jetson-utils 
		# tzmedia 
		# rtpdepay 
		# rtspc 
		# videodec
		#boost_system
	)
#target_compile_definitions(cw_DealRCF_nebulalink PRIVATE USE_TZ_SDK)
#target_compile_definitions(cw_DealRCF_nebulalink PRIVATE USE_DH_SDK)
endif()
