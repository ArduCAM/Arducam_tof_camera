cmake_minimum_required(VERSION 3.4)

project(TOFCameraExample)

# /home/user/workspace/tof/arducam_tof_sdk/install/lib/cmake/ArducamDepthCamera/ArducamDepthCameraConfig.cmake
# set(ArducamDepthCamera_DIR "/home/user/workspace/tof/arducam_tof_sdk/install/lib/cmake/ArducamDepthCamera")
find_package(ArducamDepthCamera REQUIRED)

set(CLIBS ArducamDepthCamera::ArducamDepthCamera4C)
set(CXXLIBS ArducamDepthCamera::ArducamDepthCamera)

# include_directories(${INC_DIR})
# include_directories("/usr/local/include")
# link_directories("/usr/local/lib")

add_subdirectory(example)
add_subdirectory(pcl_preview)
