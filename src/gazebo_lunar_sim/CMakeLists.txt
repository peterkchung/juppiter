cmake_minimum_required(VERSION 3.8)
project(gazebo_lunar_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ros_gz REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

# gz_ros2_control is optional - needed for hardware control bridge
find_package(gz_ros2_control QUIET)
if(gz_ros2_control_FOUND)
  message(STATUS "gz_ros2_control found - hardware control bridge enabled")
else()
  message(WARNING "gz_ros2_control not found - building without control bridge")
endif()

install(
  DIRECTORY 
    launch
    urdf
    worlds
    models
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
