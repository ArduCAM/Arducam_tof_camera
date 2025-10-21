cmake_minimum_required(VERSION 3.4)

project(modern_preview)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(modern_preview
    modern_preview/modern_preview.cpp
    modern_preview/Camera.cpp
    modern_preview/Viewer.cpp
    modern_preview/TOFApplication.cpp)

target_link_libraries(modern_preview ${CXXLIBS} ${OpenCV_LIBS})
