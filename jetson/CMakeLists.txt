cmake_minimum_required(VERSION 3.4)

project(jetson_example)
include_directories(BEFORE /usr/include/c  /usr/include/cpp)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

project(JetsonPreview)
add_executable(jetson_preview jetson_preview.cpp )
target_link_libraries( jetson_preview ArducamDepthCamera  ${OpenCV_LIBS} )

