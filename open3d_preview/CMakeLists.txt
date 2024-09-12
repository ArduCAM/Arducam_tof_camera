cmake_minimum_required(VERSION 3.4)

project(PreviewPointcloud)

add_compile_options(-Wno-psabi)

find_package(Open3D QUIET)
find_package(OpenCV REQUIRED)

set(CORE_LIBS ${CXXLIBS} Open3D::Open3D)

set(WITH_OPENCV_WORLD OFF CACHE BOOL "with opencv_world")
if(WITH_OPENCV_WORLD)
  set(CORE_LIBS ${CORE_LIBS} opencv_world)
else()
  set(CORE_LIBS ${CORE_LIBS} ${OpenCV_LIBS})
endif()

if(Open3D_FOUND)
  add_executable(preview_pointcloud preview_pointcloud.cpp)
  target_include_directories(preview_pointcloud PRIVATE
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(preview_pointcloud PRIVATE
    ${CORE_LIBS}
  )
else()
  message(WARNING "Open3D point cloud preview need libopen3d!")
endif()
