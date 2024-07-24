cmake_minimum_required(VERSION 3.4)

project(TOFCameraExample)

set(INC_DIR
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/install/include
)

set(LIBS
    ${OpenCV_LIBS}
)

set(C_LIBS
    ArducamDepthCamera2c
    ${LIBS}
)

set(CPP_LIBS
    ArducamDepthCamera
    ${LIBS}
)

# include_directories(${INC_DIR})
# include_directories("/usr/local/include")
# link_directories("/usr/local/lib")

add_subdirectory(example)
add_subdirectory(pcl_preview)
