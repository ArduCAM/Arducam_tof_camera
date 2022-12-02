cmake_minimum_required(VERSION 3.4)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb -Wno-psabi")  

project(TOFCameraExample)

add_subdirectory(example)
add_subdirectory(pcl_preview)
add_subdirectory(jetson)
