cmake_minimum_required(VERSION 3.4)
LINK_DIRECTORIES(./lib)
include_directories(./lib)

project(Test_C)
add_executable( test_c test.c )
target_link_libraries( test_c ArducamDepthCamera_c pthread)

add_subdirectory(lib)
