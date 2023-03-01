cmake_minimum_required(VERSION 3.4)
include_directories(/usr/include/c)

project(Test_C)
add_executable( test_c test.c )
target_link_libraries( test_c ArducamDepthCamera2c)
