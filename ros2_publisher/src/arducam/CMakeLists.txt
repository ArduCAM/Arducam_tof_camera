cmake_minimum_required(VERSION 3.8)
project(arducam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# set(ENV{ARDUCAM_LIB_PATH} /usr/lib)
set(ENV{ARDUCAM_HEARD_PATH} /usr/include/cpp)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  # /usr/include/cpp
  $ENV{ARDUCAM_HEARD_PATH}
  ${PCL_INCLUDE_DIRS}
  ${ament_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(tof_pointcloud src/tof_pointcloud.cpp)
ament_target_dependencies(tof_pointcloud rclcpp std_msgs sensor_msgs
)
target_link_libraries(tof_pointcloud  ArducamDepthCamera
)
install(TARGETS
tof_pointcloud
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
