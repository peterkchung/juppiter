# Sensor Integration Guide

This guide walks you through adding a new sensor to juppiter using the plugin architecture.

## Overview

juppiter uses `pluginlib` to load sensors dynamically. To add a new sensor, you create a plugin package that implements the `SensorDriver` interface.

```
┌─────────────────────────────────────────────────────┐
│  Your Sensor Plugin                                  │
│  ┌─────────────────────────────────────────────┐   │
│  │  MySensorDriver                             │   │
│  │  ├─ initialize(node)                        │   │
│  │  ├─ start() → publish /my_sensor/data       │   │
│  │  ├─ stop()                                  │   │
│  │  └─ getHealthStatus()                       │   │
│  └─────────────────────────────────────────────┘   │
│           │                                         │
│           implements                                │
│           ▼                                         │
│  ┌─────────────────────────────────────────────┐   │
│  │  sensor_interfaces::SensorDriver (abstract) │   │
│  └─────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────┘
                     │
                     ▼
         ┌─────────────────────┐
         │   sensor_core        │
         │  loads dynamically   │
         └─────────────────────┘
```

## Prerequisites

Before starting:
- Complete [Installation](../getting-started/installation.md)
- Understand [Plugin Architecture](../architecture/interfaces.md)
- Have your sensor working standalone (outside juppiter)

## Step-by-Step Integration

### Step 1: Create Package

```bash
cd /home/peter/projects/juppiter/src
mkdir -p mysensor_driver/{src,include/mysensor_driver,config}
cd mysensor_driver
```

### Step 2: Create package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mysensor_driver</name>
  <version>0.1.0</version>
  <description>My custom sensor driver for juppiter</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>sensor_interfaces</depend>
  <depend>sensor_msgs</depend>
  <depend>pluginlib</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Step 3: Create Plugin Description

Create `mysensor_driver_plugin.xml`:

```xml
<library path="mysensor_driver">
  <class 
    name="mysensor_driver::MySensorDriver"
    type="mysensor_driver::MySensorDriver"
    base_class_type="sensor_interfaces::SensorDriver">
    <description>
      Driver for my custom sensor.
    </description>
  </class>
</library>
```

### Step 4: Implement the Driver

Create `include/mysensor_driver/my_sensor_driver.hpp`:

```cpp
#pragma once

#include "sensor_interfaces/sensor_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace mysensor_driver {

class MySensorDriver : public sensor_interfaces::SensorDriver {
public:
    MySensorDriver();
    ~MySensorDriver() override;
    
    // Required interface methods
    bool initialize(rclcpp::Node::SharedPtr node) override;
    bool start() override;
    bool stop() override;
    void shutdown() override;
    
    sensor_interfaces::HealthStatus getHealthStatus() override;
    sensor_interfaces::SensorCapability getCapabilities() override;
    
    bool loadCalibration(const std::string& path) override;
    sensor_interfaces::SensorCalibration getCalibration() override;
    
    void registerTimeSync(
        std::shared_ptr<sensor_interfaces::TimeSynchronizer> sync) override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::shared_ptr<sensor_interfaces::TimeSynchronizer> time_sync_;
    sensor_interfaces::SensorCalibration calibration_;
    
    // Your sensor-specific members
    void* sensor_handle_;  // Handle to your sensor library
    
    void publishData();
    void connectToSensor();
    void disconnectFromSensor();
};

} // namespace mysensor_driver
```

### Step 5: Implement Methods

Create `src/my_sensor_driver.cpp`:

```cpp
#include "mysensor_driver/my_sensor_driver.hpp"
#include <rclcpp/logging.hpp>

namespace mysensor_driver {

MySensorDriver::MySensorDriver() 
    : sensor_handle_(nullptr) {
}

MySensorDriver::~MySensorDriver() {
    shutdown();
}

bool MySensorDriver::initialize(rclcpp::Node::SharedPtr node) {
    node_ = node;
    
    // Create ROS publisher
    pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/mysensor/points", 10);
    
    // Load parameters
    node_->declare_parameter("port", "/dev/ttyUSB0");
    node_->declare_parameter("baudrate", 115200);
    node_->declare_parameter("frame_id", "mysensor_frame");
    
    RCLCPP_INFO(node_->get_logger(), 
                "MySensorDriver initialized for port: %s",
                node_->get_parameter("port").as_string().c_str());
    
    return true;
}

bool MySensorDriver::start() {
    // Connect to hardware
    connectToSensor();
    
    if (!sensor_handle_) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to connect to sensor");
        return false;
    }
    
    // Start publishing at 10Hz
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MySensorDriver::publishData, this));
    
    RCLCPP_INFO(node_->get_logger(), "MySensorDriver started");
    return true;
}

bool MySensorDriver::stop() {
    if (timer_) {
        timer_->cancel();
    }
    
    disconnectFromSensor();
    
    RCLCPP_INFO(node_->get_logger(), "MySensorDriver stopped");
    return true;
}

void MySensorDriver::shutdown() {
    stop();
    pub_.reset();
    node_.reset();
}

sensor_interfaces::HealthStatus MySensorDriver::getHealthStatus() {
    sensor_interfaces::HealthStatus health;
    
    if (!sensor_handle_) {
        health.score = 0.0;
        health.flags = sensor_interfaces::FaultFlags::STALE;
        health.details = "Sensor not connected";
    } else {
        // Check if data is flowing
        // This is simplified - real implementation would
        // track last publish time
        health.score = 1.0;
        health.flags = sensor_interfaces::FaultFlags::NONE;
        health.publication_rate_hz = 10.0;
        health.expected_rate_hz = 10.0;
        health.details = "Operating normally";
    }
    
    health.timestamp = node_->now();
    return health;
}

sensor_interfaces::SensorCapability MySensorDriver::getCapabilities() {
    return sensor_interfaces::SensorCapability::LIDAR_POINTS;
}

bool MySensorDriver::loadCalibration(const std::string& path) {
    // Load calibration from YAML or XML file
    // Implementation depends on your calibration format
    
    calibration_.sensor_name = "mysensor";
    calibration_.version = 1;
    calibration_.calibration_date = node_->now();
    
    // Set default extrinsics (identity transform)
    calibration_.extrinsics = {
        1, 0, 0, 0,  // Row 1
        0, 1, 0, 0,  // Row 2
        0, 0, 1, 0,  // Row 3
        0, 0, 0, 1   // Row 4
    };
    
    return true;
}

sensor_interfaces::SensorCalibration MySensorDriver::getCalibration() {
    return calibration_;
}

void MySensorDriver::registerTimeSync(
    std::shared_ptr<sensor_interfaces::TimeSynchronizer> sync) {
    time_sync_ = sync;
}

void MySensorDriver::publishData() {
    if (!sensor_handle_) {
        return;
    }
    
    // Read from your sensor
    // This is pseudo-code - replace with your actual sensor API
    auto points = readSensorData(sensor_handle_);
    
    // Convert to ROS message
    auto msg = sensor_msgs::msg::PointCloud2();
    msg.header.stamp = node_->now();
    msg.header.frame_id = node_->get_parameter("frame_id").as_string();
    
    // Fill in point cloud data
    // msg.height = ...
    // msg.width = ...
    // msg.fields = ...
    // msg.data = ...
    
    // Register timestamp for synchronization monitoring
    if (time_sync_) {
        time_sync_->registerTimestamp("mysensor", msg.header.stamp);
    }
    
    pub_->publish(msg);
}

void MySensorDriver::connectToSensor() {
    // Initialize your sensor hardware
    // Example: open serial port, USB, or network connection
    // sensor_handle_ = your_sensor_init(...);
    
    // For this example, we'll simulate a successful connection
    sensor_handle_ = reinterpret_cast<void*>(0x1);  // Non-null indicates connected
}

void MySensorDriver::disconnectFromSensor() {
    if (sensor_handle_) {
        // your_sensor_close(sensor_handle_);
        sensor_handle_ = nullptr;
    }
}

} // namespace mysensor_driver
```

### Step 6: Create CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(mysensor_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# Create the plugin library
add_library(${PROJECT_NAME} SHARED
  src/my_sensor_driver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_interfaces
  sensor_msgs
  pluginlib
)

# Export the plugin description
pluginlib_export_plugin_description_file(
  sensor_interfaces mysensor_driver_plugin.xml)

# Install targets
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES mysensor_driver_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### Step 7: Build and Test

```bash
# Build the package
colcon build --packages-select mysensor_driver

# Verify plugin is registered
ros2 pkg plugins --type sensor_interfaces::SensorDriver

# Should show: mysensor_driver::MySensorDriver
```

### Step 8: Add to Configuration

Edit `config/compute_profiles/dev.yaml`:

```yaml
providers:
  - mysensor_driver::MySensorDriver
  - gazebo_lunar_sim::GazeboLidarProvider
  # ... other providers
```

Create `config/sensors/mysensor.yaml`:

```yaml
mysensor:
  name: "mysensor"
  frame_id: "mysensor_frame"
  topic: "/mysensor/points"
  publish_rate_hz: 10.0
  
  # Hardware connection
  connection:
    type: "serial"  # or "ethernet", "usb"
    port: "/dev/ttyUSB0"
    baudrate: 115200
  
  # Time synchronization
  time_sync:
    reference_clock: "ros"
    offset_s: 0.0
```

### Step 9: Launch with New Sensor

```bash
# Source the workspace
source install/setup.bash

# Launch with updated config
ros2 launch gazebo_lunar_sim simulation.launch.py compute_profile:=dev

# Verify sensor is publishing
ros2 topic list | grep mysensor
ros2 topic hz /mysensor/points
ros2 topic echo /perception/health
```

## Best Practices

### Do:
- ✓ Return realistic health scores (not always 1.0)
- ✓ Handle connection failures gracefully
- ✓ Use parameters for hardware settings (ports, baudrate)
- ✓ Register timestamps with TimeSynchronizer
- ✓ Log initialization progress
- ✓ Implement proper shutdown cleanup
- ✓ Check for null pointers before dereferencing
- ✓ Use try-catch around hardware calls

### Don't:
- ✗ Block in start()/stop() (use async operations)
- ✗ Hardcode topic names
- ✗ Ignore calibration data
- ✗ Publish at unlimited rates
- ✗ Create publishers in constructors
- ✗ Assume node is always valid

## Testing Your Sensor

### Unit Tests

Create `test/test_mysensor_driver.cpp`:

```cpp
#include <gtest/gtest.h>
#include "mysensor_driver/my_sensor_driver.hpp"

TEST(MySensorDriverTest, Initialization) {
    auto driver = std::make_shared<mysensor_driver::MySensorDriver>();
    // Test initialization
}

TEST(MySensorDriverTest, HealthStatus) {
    auto driver = std::make_shared<mysensor_driver::MySensorDriver>();
    auto health = driver->getHealthStatus();
    EXPECT_GE(health.score, 0.0);
    EXPECT_LE(health.score, 1.0);
}
```

### Integration Tests

```bash
# Run with your sensor connected
ros2 launch mysensor_driver test.launch.py

# Verify data flow
ros2 topic hz /mysensor/points
```

## Troubleshooting

### Plugin Not Found

```bash
# Verify plugin is built
ls install/mysensor_driver/lib/libmysensor_driver.so

# Check plugin description
ros2 pkg plugins --type sensor_interfaces::SensorDriver

# Rebuild with verbose output
colcon build --packages-select mysensor_driver --event-handlers console_direct+
```

### Connection Issues

```bash
# Check hardware connection
ls -la /dev/ttyUSB0  # or your port

# Test with minimal code
# Write a standalone program to verify sensor works
```

### Health Always 0

- Check `sensor_handle_` is properly initialized
- Verify `getHealthStatus()` is implemented
- Ensure `time_sync_->registerTimestamp()` is called

## Example: Minimal Working Plugin

For a minimal example, see the `euroc_provider` package in `test/euroc_provider/`. It demonstrates:
- Basic SensorDriver implementation
- Health monitoring
- Time synchronization
- Parameter loading

## Next Steps

- [Configuration Guide](configuration.md) - Tune your sensor settings
- [Architecture/Interfaces](../architecture/interfaces.md) - Deep dive into interfaces
- [FDIIR Testing](fdiir-testing.md) - Test fault handling
