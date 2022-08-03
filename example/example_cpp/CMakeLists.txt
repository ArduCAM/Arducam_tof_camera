cmake_minimum_required(VERSION 3.4)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

LINK_DIRECTORIES(./lib)
include_directories(./lib)

project(PreviewDeptah)
add_executable(preview_deptah preview_depth.cpp )
target_link_libraries( preview_deptah ArducamDepthCamera  ${OpenCV_LIBS} )

project(CaptureRaw)
add_executable(capture_raw capture_raw.cpp )
target_link_libraries( capture_raw ArducamDepthCamera  ${OpenCV_LIBS} )

add_subdirectory(lib)
