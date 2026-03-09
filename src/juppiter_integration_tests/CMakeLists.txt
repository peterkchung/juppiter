cmake_minimum_required(VERSION 3.8)
project(juppiter_integration_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Scenario runner node
add_executable(scenario_runner
  src/scenario_runner.cpp
  src/fault_injector.cpp
  src/metrics_collector.cpp
  src/scenario_loader.cpp
)

ament_target_dependencies(scenario_runner
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
)

target_link_libraries(scenario_runner yaml-cpp)

install(TARGETS scenario_runner
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Integration tests
  ament_add_gtest(test_fdiir_scenarios test/test_fdiir_scenarios.cpp)
  ament_target_dependencies(test_fdiir_scenarios
    rclcpp
    nav_msgs
    sensor_msgs
    std_msgs
  )
  target_link_libraries(test_fdiir_scenarios yaml-cpp)
endif()

ament_package()
