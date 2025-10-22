cmake_minimum_required(VERSION 3.4)

project(modern_preview)

find_package(ArducamDepthCamera REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(modern_preview
    source/main.cpp
    source/Camera.cpp
    source/Viewer.cpp
    source/TOFApplication.cpp)

target_link_libraries(modern_preview ArducamDepthCamera::ArducamDepthCamera ${OpenCV_LIBS})
