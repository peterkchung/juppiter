cmake_minimum_required(VERSION 3.8)
project(fusion_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(common_msgs REQUIRED)

find_package(sensor_interfaces REQUIRED)
find_package(lio_estimator_interfaces REQUIRED)
find_package(vio_estimator_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/fusion_engine.cpp
  src/mode_manager.cpp
  src/weight_calculator.cpp
  src/health_client.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
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
  common_msgs
  sensor_interfaces
  lio_estimator_interfaces
  vio_estimator_interfaces
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

add_executable(fusion_node
  src/fusion_node.cpp
)

target_link_libraries(fusion_node
  ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} fusion_node
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# No config files to install yet
# install(
#   DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config
# )

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs yaml-cpp common_msgs
  sensor_interfaces lio_estimator_interfaces vio_estimator_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # TODO: Add tests when test files are created
  # find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(test_fusion_engine test/test_fusion_engine.cpp)
  # target_link_libraries(test_fusion_engine ${PROJECT_NAME})
endif()

ament_package()
