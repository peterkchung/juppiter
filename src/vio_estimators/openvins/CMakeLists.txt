cmake_minimum_required(VERSION 3.8)
project(openvins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 for OpenVINS (better constexpr support)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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

# Optional: OpenVINS library (external system dependency)
# If not found, adapter will use stub implementation
find_package(ov_msckf QUIET PATHS /opt/openvins)
find_package(ov_core QUIET PATHS /opt/openvins)

if(ov_msckf_FOUND AND ov_core_FOUND)
  message(STATUS "OpenVINS library found - building with real MSCKF")
  add_definitions(-DOPENVINS_AVAILABLE=1)
  set(OPENVINS_LIBS ov_msckf ov_core)
else()
  message(WARNING "OpenVINS library not found - building stub implementation")
  message(WARNING "Run: sudo ./scripts/setup_openvins.sh")
  set(OPENVINS_LIBS)
endif()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Library: OpenVINS adapter
add_library(${PROJECT_NAME} SHARED
  src/openvins_adapter.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  vio_estimator_interfaces
  cv_bridge
  image_transport
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${OPENVINS_LIBS}
)

# Executable: OpenVINS node
add_executable(openvins_node
  src/openvins_node.cpp
)

target_link_libraries(openvins_node
  ${PROJECT_NAME}
)

ament_target_dependencies(openvins_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  vio_estimator_interfaces
  cv_bridge
)

# Install targets
install(TARGETS ${PROJECT_NAME} openvins_node
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
  rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs
  vio_estimator_interfaces cv_bridge image_transport
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
