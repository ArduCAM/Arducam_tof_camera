cmake_minimum_required(VERSION 3.13)
project(PreviewPointcloud)
add_compile_options(-Wno-psabi)
find_package(PCL QUIET COMPONENTS common io visualization)
find_package(OpenCV REQUIRED)

set(CORE_LIBS ArducamDepthCamera ${PCL_LIBRARIES})

set(WITH_OPENCV_WORLD OFF CACHE BOOL "with opencv_world")
if(WITH_OPENCV_WORLD)
  set(CORE_LIBS ${CORE_LIBS} opencv_world)
else()
  set(CORE_LIBS ${CORE_LIBS} ${OpenCV_LIBS})
endif()

if(PCL_FOUND)
    message(STATUS "LIBPCL_LINK_LIBRARIES=${PCL_INCLUDE_DIRS}")
    message(STATUS "EPOXY_LINK_LIBRARIES=${PCL_LIBRARY_DIRS}")
    add_executable(preview_pointcloud preview_pointcloud.cpp)
    target_include_directories(preview_pointcloud PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
    )
    target_link_directories(preview_pointcloud PRIVATE
      ${PCL_LIBRARY_DIRS}
    )
    target_compile_definitions(preview_pointcloud PRIVATE
        ${PCL_DEFINITIONS}
    )
    #-lboost_thread -lpthread
    target_link_libraries(preview_pointcloud
        ${CORE_LIBS}
    )
else()
    message(WARNING "PCL point cloud preview need libpcl!")
endif()
