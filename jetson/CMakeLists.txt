cmake_minimum_required(VERSION 3.4)

project(jetson_example)
include_directories(BEFORE /usr/include/c  /usr/include/cpp)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

project(NanoPreview)
add_executable(nano_preview jetson_nano_preview.cpp )
target_link_libraries( nano_preview ArducamDepthCamera  ${OpenCV_LIBS} )

project(NxPreview)
add_executable(nx_preview jetson_nx_preview.cpp )
target_link_libraries( nx_preview ArducamDepthCamera  ${OpenCV_LIBS} )


