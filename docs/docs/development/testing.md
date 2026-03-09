# Testing

juppiter includes comprehensive testing at multiple levels. This guide covers how to run and write tests.

## Test Levels

```
┌─────────────────────────────────────────────────────┐
│  Test Pyramid                                       │
│                                                     │
│        ┌─────────┐                                  │
│        │  FDIIR  │  ← Scenario-based fault tests   │
│        │Scenarios│                                  │
│       ┌┴─────────┴┐                                 │
│       │Integration│  ← Component interactions     │
│       │   Tests   │                                  │
│      ┌┴───────────┴┐                                │
│      │  Unit Tests │  ← Isolated function tests    │
│      └─────────────┘                                │
└─────────────────────────────────────────────────────┘
```

## Running Tests

### Quick Test

```bash
# Run all tests
colcon test

# View results
colcon test-result --verbose
```

### Package-Specific Tests

```bash
# Test specific package
colcon test --packages-select sensor_core

# Test multiple packages
colcon test --packages-select sensor_core fusion_core gazebo_lunar_sim

# With detailed output
colcon test --packages-select sensor_core --event-handlers console_direct+
```

### Test Categories

```bash
# Run only unit tests
colcon test --packages-select sensor_core --ctest-args -L unit

# Run only integration tests
colcon test --packages-select sensor_core --ctest-args -L integration

# Run FDIIR scenarios
ros2 launch gazebo_lunar_sim fdiir_test_runner.launch.py
```

## Unit Tests

### Structure

Located in `test/` directory of each package:

```
sensor_core/
├── src/
├── include/
└── test/
    ├── test_health_monitor.cpp
    ├── test_time_synchronizer.cpp
    └── test_sensor_bridge.cpp
```

### Writing Unit Tests

```cpp
// test_health_monitor.cpp
#include <gtest/gtest.h>
#include "sensor_core/health_monitor.hpp"

using namespace sensor_core;

class HealthMonitorTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("test_node");
        monitor_ = std::make_unique<HealthMonitor>(node_);
    }
    
    void TearDown() override {
        rclcpp::shutdown();
    }
    
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<HealthMonitor> monitor_;
};

TEST_F(HealthMonitorTest, InitialHealthIsZero) {
    auto health = monitor_->getOverallHealth();
    EXPECT_DOUBLE_EQ(health, 0.0);
}

TEST_F(HealthMonitorTest, UpdateFromSensor) {
    sensor_interfaces::HealthStatus status;
    status.score = 0.95;
    status.flags = sensor_interfaces::FaultFlags::NONE;
    status.publication_rate_hz = 10.0;
    status.expected_rate_hz = 10.0;
    
    monitor_->updateSensorHealth("lidar", status);
    
    auto health = monitor_->getSensorHealth("lidar");
    EXPECT_DOUBLE_EQ(health, 0.95);
}

TEST_F(HealthMonitorTest, HealthDegradesWithStaleData) {
    sensor_interfaces::HealthStatus status;
    status.score = 1.0;
    status.flags = sensor_interfaces::FaultFlags::STALE;
    
    monitor_->updateSensorHealth("lidar", status);
    
    auto health = monitor_->getSensorHealth("lidar");
    EXPECT_LT(health, 1.0);  // Should be reduced
}
```

### CMake Configuration

```cmake
# CMakeLists.txt
find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_health_monitor test/test_health_monitor.cpp)
target_link_libraries(test_health_monitor ${PROJECT_NAME})
```

## Integration Tests

### Launch Files

Located in `test/` or `launch/` directories:

```python
# test_sensor_integration.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start sensor bridge
        Node(
            package='sensor_core',
            executable='sensor_bridge_node',
            name='sensor_bridge',
            parameters=[{'config_path': 'config/compute_profiles/dev.yaml'}]
        ),
        
        # Wait 2 seconds then run tests
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'sensor_core', 'integration_test_runner'],
                    output='screen'
                )
            ]
        )
    ])
```

### Writing Integration Tests

```cpp
// integration_test_runner.cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class IntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        node_ = std::make_shared<rclcpp::Node>("integration_test");
        
        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/perception/odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                odom_received_ = true;
                last_odom_ = msg;
            });
    }
    
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    bool odom_received_ = false;
    nav_msgs::msg::Odometry::SharedPtr last_odom_;
};

TEST_F(IntegrationTest, OdometryPublished) {
    // Spin for 5 seconds
    rclcpp::Rate rate(10);
    for (int i = 0; i < 50; ++i) {
        rclcpp::spin_some(node_);
        rate.sleep();
    }
    
    EXPECT_TRUE(odom_received_) << "No odometry message received";
}

TEST_F(IntegrationTest, OdometryHasValidCovariance) {
    // Wait for first message
    while (!odom_received_ && rclcpp::ok()) {
        rclcpp::spin_some(node_);
    }
    
    // Check covariance is positive definite
    auto& cov = last_odom_->pose.covariance;
    EXPECT_GT(cov[0], 0.0);   // x variance
    EXPECT_GT(cov[7], 0.0);   // y variance
    EXPECT_GT(cov[14], 0.0);  // z variance
}
```

## FDIIR Tests

See [FDIIR Testing Guide](../user-guides/fdiir-testing.md) for detailed information.

### Running All Scenarios

```bash
# Run all FDIIR test scenarios
ros2 launch gazebo_lunar_sim fdiir_test_runner.launch.py

# Run specific scenario
ros2 launch gazebo_lunar_sim simulation.launch.py scenario:=lidar_dropout

# Run with recording
ros2 launch gazebo_lunar_sim simulation.launch.py \
  scenario:=lidar_dropout \
  record:=true
```

### Scenario Validation

Tests validate:
1. **Fault Detection** - System detects injected fault
2. **Fault Isolation** - Correct component identified
3. **Mode Transition** - System enters correct mode
4. **Graceful Degradation** - Position continuity maintained
5. **Recovery** - System returns to nominal when fault clears

## Test Coverage

### Measuring Coverage

```bash
# Build with coverage
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="--coverage"

# Run tests
colcon test

# Generate coverage report
# (Install lcov first: apt-get install lcov)
lcov --capture --directory build --output-file coverage.info
lcov --remove coverage.info '/usr/*' --output-file coverage.info
lcov --list coverage.info

# Generate HTML report
genhtml coverage.info --output-directory coverage_html
```

### Coverage Requirements

| Component | Minimum Coverage |
|-----------|------------------|
| sensor_core | 80% |
| fusion_core | 80% |
| sensor_interfaces | N/A (headers only) |
| Providers | 60% |
| Estimators | 70% |

## Performance Testing

### Latency Measurement

```bash
# Monitor perception pipeline latency
ros2 topic echo /perception/odom --field header.stamp

# Compare to input timestamps
ros2 topic echo /lidar/points --field header.stamp
```

### CPU Profiling

```bash
# Run with perf
docker compose -f docker/docker-compose.yml exec dev bash
perf record -g ros2 run sensor_core sensor_bridge_node
perf report
```

### Memory Checking

```bash
# Run with valgrind
valgrind --tool=memcheck --leak-check=full \
  ros2 run sensor_core sensor_bridge_node
```

## Continuous Integration

Tests run automatically on:
- Every PR to `develop` or `main`
- Nightly builds
- Release preparation

See [CI/CD Guide](ci-cd.md) for pipeline details.

## Writing Good Tests

### Unit Test Best Practices

✓ **Do:**
- Test one thing per test
- Use descriptive names: `TEST_F(HealthMonitorTest, DegradesWhenStale)`
- Set up and tear down properly
- Mock external dependencies
- Test edge cases and error paths

✗ **Don't:**
- Test multiple scenarios in one test
- Depend on test order
- Leave test data files in repo root
- Use sleep() for synchronization
- Skip error handling paths

### Integration Test Best Practices

✓ **Do:**
- Test realistic scenarios
- Clean up resources after tests
- Use timeouts to prevent hangs
- Log detailed failure information
- Test with actual ROS topics

✗ **Don't:**
- Hardcode topic names
- Assume specific timing
- Leave nodes running after tests
- Test with hardcoded paths

## Debugging Tests

### Verbose Output

```bash
# GTest verbose
colcon test --packages-select sensor_core --ctest-args -V

# ROS 2 verbose logging
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
ros2 run sensor_core sensor_bridge_node --ros-args --log-level debug
```

### Debugging Failures

```bash
# Run specific test with GDB
gdb -ex run --args \
  ./build/sensor_core/test_health_monitor \
  --gtest_filter=HealthMonitorTest.InitialHealthIsZero

# Add breakpoints
(gdb) break HealthMonitor::getOverallHealth
(gdb) run
```

## Test Data

### Test Datasets

- **EuRoC**: Located in `test/euroc_provider/`
  - Download: `./scripts/download_euroc.sh`
  - Usage: `ros2 launch launch/replay.launch.py dataset_path:=/ws/data/MH_01_easy/mav0`

### Mock Data

Create mock sensor data for testing:

```cpp
// test/mock_data.hpp
sensor_msgs::msg::PointCloud2 createMockPointCloud() {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = rclcpp::Clock().now();
    msg.height = 1;
    msg.width = 100;
    // ... populate data
    return msg;
}
```

## Next Topics

- [FDIIR Testing](../user-guides/fdiir-testing.md) - Fault injection scenarios
- [CI/CD](ci-cd.md) - Automated testing pipeline
- [Contributing](contributing.md) - Code review process
