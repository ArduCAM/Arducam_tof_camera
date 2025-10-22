cmake_minimum_required(VERSION 3.4)

project(c_example)

add_executable(preview_depth_c preview_depth.c)
target_link_libraries(preview_depth_c ${CLIBS})
