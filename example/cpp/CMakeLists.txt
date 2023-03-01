cmake_minimum_required(VERSION 3.4)
include_directories(/usr/include/cpp)

project(cpp_example)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

project(PreviewDeptah)
add_executable(preview_depth preview_depth.cpp )
target_link_libraries( preview_depth ArducamDepthCamera  ${OpenCV_LIBS} )

project(SelectVideoPreview)
add_executable(select_video_preview select_video_preview.cpp )
target_link_libraries( select_video_preview ArducamDepthCamera  ${OpenCV_LIBS} )

project(CaptureRaw)
add_executable(capture_raw capture_raw.cpp )
target_link_libraries( capture_raw ArducamDepthCamera  ${OpenCV_LIBS} )


project(PreviewUSB)
add_executable(preview_usb preview_by_usb.cpp )
target_link_libraries( preview_usb ArducamDepthCamera  ${OpenCV_LIBS} )