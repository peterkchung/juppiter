cmake_minimum_required(VERSION 3.8)
project(orb_slam3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 required for ORB-SLAM3
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vio_estimator_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)

# Pangolin (for ORB-SLAM3 visualization)
find_package(Pangolin REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/orb_slam3_estimator.cpp
  src/slam/visual_odometry.cpp
  src/slam/tracking.cpp
  src/slam/local_mapping.cpp
  src/slam/loop_closing.cpp
  src/slam/atlas.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  nav_msgs::nav_msgs
  sensor_msgs::sensor_msgs
  geometry_msgs::geometry_msgs
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  vio_estimator_interfaces::vio_estimator_interfaces
  cv_bridge::cv_bridge
  image_transport::image_transport
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
)

add_executable(orb_slam3_node
  src/orb_slam3_node.cpp
)

target_link_libraries(orb_slam3_node
  ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} orb_slam3_node
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp nav_msgs sensor_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs
  vio_estimator_interfaces cv_bridge image_transport
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
