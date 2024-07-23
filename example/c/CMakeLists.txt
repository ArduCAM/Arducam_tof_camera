cmake_minimum_required(VERSION 3.4)
include_directories(/usr/include/c)

project(c_example)

add_executable(preview_depth_c preview_depth.c)
target_link_libraries(preview_depth_c ArducamDepthCamera2c)
