cmake_minimum_required(VERSION 3.6)

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(pcl_visualization QUIET pcl)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

LINK_DIRECTORIES(./lib)
include_directories(./lib)

project(PreviewDeptah)
add_executable(preview_depth preview_depth.cpp )
target_link_libraries( preview_depth ArducamDepthCamera  ${OpenCV_LIBS} )

project(CaptureRaw)
add_executable(capture_raw capture_raw.cpp )
target_link_libraries( capture_raw ArducamDepthCamera  ${OpenCV_LIBS} )

find_package(PCL)
if (PCL_FOUND)
    message(STATUS "LIBPCL_LINK_LIBRARIES=${PCL_INCLUDE_DIRS}")
    message(STATUS "EPOXY_LINK_LIBRARIES=${PCL_LIBRARY_DIRS}")
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    add_executable (preview_pointcloud preview_pointcloud.cpp)
    target_link_libraries (preview_pointcloud ${PCL_LIBRARIES} -lboost_thread -lpthread ArducamDepthCamera  ${OpenCV_LIBS} )
else()
    message(STATUS "PCL point cloud preview skipped!")
endif()

add_subdirectory(lib)
