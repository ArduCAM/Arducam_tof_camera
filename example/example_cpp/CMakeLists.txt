cmake_minimum_required(VERSION 3.4)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# LINK_DIRECTORIES(./lib)
# include_directories(./lib)

project(Test_CPP)
add_executable(test_cpp test.cpp )
target_link_libraries( test_cpp ArducamDepthCamera  ${OpenCV_LIBS} )

add_subdirectory(lib)
