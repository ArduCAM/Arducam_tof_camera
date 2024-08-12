cmake_minimum_required(VERSION 3.4)

project(cpp_example)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(preview_depth preview_depth.cpp)
target_link_libraries(preview_depth ${CXXLIBS} ${OpenCV_LIBS})

add_executable(capture_raw capture_raw.cpp)
target_link_libraries(capture_raw ${CXXLIBS} ${OpenCV_LIBS})

add_executable(preview_usb preview_by_usb.cpp)
target_link_libraries(preview_usb ${CXXLIBS} ${OpenCV_LIBS})

add_executable(example test.cpp)
target_link_libraries(example ${CXXLIBS} ${OpenCV_LIBS})
