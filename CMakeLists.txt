cmake_minimum_required(VERSION 3.4)

project(modern_preview)

find_package(ArducamDepthCamera REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(modern_preview
    apps/main.cpp
    src/Camera.cpp
    src/Viewer.cpp
    src/TOFApplication.cpp)

target_link_libraries(modern_preview ArducamDepthCamera::ArducamDepthCamera ${OpenCV_LIBS})
