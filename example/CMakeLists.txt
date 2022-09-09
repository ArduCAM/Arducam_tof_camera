cmake_minimum_required(VERSION 3.4)

include_directories(BEFORE /usr/include/c  /usr/include/cpp)

project(example)

add_subdirectory(c)
add_subdirectory(cpp)
