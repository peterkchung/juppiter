# About: Build config for sensor_bridge — EuRoC replay node with stereo depth.

cmake_minimum_required(VERSION 3.8)
project(sensor_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(sensor_bridge_node
  src/main.cpp
  src/sensor_bridge_node.cpp
  src/euroc_reader.cpp
  src/stereo_depth.cpp
)

target_include_directories(sensor_bridge_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(sensor_bridge_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

target_link_libraries(sensor_bridge_node ${OpenCV_LIBS})

install(TARGETS sensor_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
