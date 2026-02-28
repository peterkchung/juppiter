# About: Build config for common_msgs.
# Generates C++ and Python types from .msg and .srv definitions
# using the ROS 2 IDL pipeline (rosidl).
cmake_minimum_required(VERSION 3.8)
project(common_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection2D.msg"
  "msg/Detection2DArray.msg"
  "msg/ObjectResult.msg"
  "srv/QueryObjects.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
