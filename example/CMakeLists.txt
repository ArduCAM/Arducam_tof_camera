cmake_minimum_required(VERSION 3.4)

SET(CMAKE_BUILD_TYPE "Release")  
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")  

project(example)

add_subdirectory(example_c)
add_subdirectory(example_cpp)
add_subdirectory(example_python)
