cmake_minimum_required(VERSION 3.8)
project(kinetic_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Note: robot_localization is a runtime dependency (ROS node), not a build dependency
# It's listed in package.xml for rosdep installation but we don't find_package() it
# as it doesn't export CMake targets

add_executable(kinetic_estimator_node
  src/kinetic_estimator_node.cpp
)

target_include_directories(kinetic_estimator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(kinetic_estimator_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS kinetic_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

# No headers to install yet (line kept for future header additions)
# install(
#   DIRECTORY include/
#   DESTINATION include
# )

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
