# Interfaces and Plugin Architecture

juppiter uses a plugin-based architecture via `pluginlib`, allowing sensors and estimators to be loaded dynamically at runtime without recompiling the core system.

## Plugin System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                 Plugin Architecture                         │
│                                                              │
│  ┌─────────────────────────────────────────────────────┐   │
│  │           Core System (sensor_core)                 │   │
│  │  • SensorBridgeNode                                  │   │
│  │  • PluginLoader                                      │   │
│  │  • No knowledge of specific sensors                  │   │
│  └────────────────────┬────────────────────────────────┘   │
│                       │                                      │
│           ┌───────────┴───────────┐                         │
│           │   pluginlib::ClassLoader│                        │
│           │   (dynamically loads)   │                        │
│           └───────────┬───────────┘                         │
│                       │                                      │
│  ┌────────────────────┼────────────────────────────────┐   │
│  │       Plugin Libraries (shared objects)                │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌────────────┐ │   │
│  │  │ euroc_provider│  │ lidar_provider│  │ your_sensor│ │   │
│  │  │   .so        │  │    .so       │  │   .so      │ │   │
│  │  └──────────────┘  └──────────────┘  └────────────┘ │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                              │
│  Interface: sensor_interfaces::SensorDriver (pure virtual) │
└─────────────────────────────────────────────────────────────┘
```

## Sensor Interfaces

### Base Interface: SensorDriver

All sensor providers must implement:

```cpp
// sensor_interfaces/include/sensor_interfaces/sensor_driver.hpp
namespace sensor_interfaces {

class SensorDriver {
public:
    virtual ~SensorDriver() = default;
    
    // Lifecycle methods
    virtual bool initialize(rclcpp::Node::SharedPtr node) = 0;
    virtual bool start() = 0;
    virtual bool stop() = 0;
    virtual void shutdown() = 0;
    
    // Health and capabilities
    virtual HealthStatus getHealthStatus() = 0;
    virtual SensorCapability getCapabilities() = 0;
    
    // Calibration
    virtual bool loadCalibration(const std::string& path) = 0;
    virtual SensorCalibration getCalibration() = 0;
    
    // Time synchronization
    virtual void registerTimeSync(
        std::shared_ptr<TimeSynchronizer> sync) = 0;
};

} // namespace sensor_interfaces
```

### Supporting Types

```cpp
// Capabilities that a sensor can provide
enum class SensorCapability : uint32_t {
    NONE = 0,
    COLOR_IMAGE = 1 << 0,
    MONO_IMAGE = 1 << 1,
    DEPTH_IMAGE = 1 << 2,
    LIDAR_POINTS = 1 << 3,
    IMU_DATA = 1 << 4,
    GNSS_DATA = 1 << 5,
    MAGNETOMETER = 1 << 6,
    BAROMETER = 1 << 7
};

// Health status reporting
struct HealthStatus {
    double score;                    // 0.0 to 1.0
    FaultFlags flags;                // Active faults
    rclcpp::Time timestamp;          // Measurement time
    std::string details;             // Human-readable status
    double publication_rate_hz;       // Measured rate
    double expected_rate_hz;          // Target rate
};

// Calibration data
struct SensorCalibration {
    std::string sensor_name;
    uint32_t version;
    rclcpp::Time calibration_date;
    std::vector<double> intrinsics;    // Camera: K matrix
    std::vector<double> distortion;     // Camera: D coeffs
    std::vector<double> extrinsics;   // Transform to base_link
    double timeshift_s;               // Time offset correction
};
```

## Estimator Interfaces

### LIO Estimator Interface

```cpp
// lio_estimators/lio_estimator_interfaces/include/lio_estimator_interfaces/lio_estimator.hpp

namespace lio_estimator_interfaces {

class LIOEstimator {
public:
    virtual ~LIOEstimator() = default;
    
    // Initialize with configuration
    virtual bool initialize(
        rclcpp::Node::SharedPtr node,
        const std::string& config_path) = 0;
    
    // Input: LiDAR point cloud
    virtual void processPointCloud(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg) = 0;
    
    // Input: IMU data
    virtual void processImu(
        const sensor_msgs::msg::Imu::SharedPtr msg) = 0;
    
    // Output: Current pose estimate
    virtual nav_msgs::msg::Odometry getOdometry() = 0;
    
    // Health metrics
    virtual EstimatorHealth getHealth() = 0;
    
    // Reset (e.g., after fault recovery)
    virtual bool reset() = 0;
};

} // namespace lio_estimator_interfaces
```

### VIO Estimator Interface

```cpp
// vio_estimators/vio_estimator_interfaces/include/vio_estimator_interfaces/vio_estimator.hpp

namespace vio_estimator_interfaces {

class VIOEstimator {
public:
    virtual ~VIOEstimator() = default;
    
    virtual bool initialize(
        rclcpp::Node::SharedPtr node,
        const std::string& config_path) = 0;
    
    // Input: Stereo images
    virtual void processStereoImages(
        const sensor_msgs::msg::Image::SharedPtr left,
        const sensor_msgs::msg::Image::SharedPtr right) = 0;
    
    // Input: IMU data
    virtual void processImu(
        const sensor_msgs::msg::Imu::SharedPtr msg) = 0;
    
    // Output: Current pose estimate
    virtual nav_msgs::msg::Odometry getOdometry() = 0;
    
    // Health metrics
    virtual EstimatorHealth getHealth() = 0;
    
    // Reset
    virtual bool reset() = 0;
};

} // namespace vio_estimator_interfaces
```

## Creating a Sensor Plugin

### Step 1: Create Package Structure

```bash
cd src
mkdir -p my_sensor_provider/{src,include/my_sensor_provider}
cd my_sensor_provider
```

### Step 2: Implement SensorDriver

```cpp
// include/my_sensor_provider/my_sensor_driver.hpp
#pragma once

#include "sensor_interfaces/sensor_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace my_sensor_provider {

class MySensorDriver : public sensor_interfaces::SensorDriver {
public:
    MySensorDriver();
    ~MySensorDriver() override;
    
    // SensorDriver interface
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
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    sensor_interfaces::SensorCalibration calibration_;
    std::shared_ptr<sensor_interfaces::TimeSynchronizer> time_sync_;
    
    void publishImuData();
};

} // namespace my_sensor_provider
```

### Step 3: Implementation

```cpp
// src/my_sensor_driver.cpp
#include "my_sensor_provider/my_sensor_driver.hpp"

namespace my_sensor_provider {

MySensorDriver::MySensorDriver() = default;
MySensorDriver::~MySensorDriver() = default;

bool MySensorDriver::initialize(rclcpp::Node::SharedPtr node) {
    node_ = node;
    
    // Create publisher
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
        "/my_sensor/imu", 10);
    
    RCLCPP_INFO(node_->get_logger(), "MySensorDriver initialized");
    return true;
}

bool MySensorDriver::start() {
    // Start publishing at 100Hz
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&MySensorDriver::publishImuData, this));
    
    RCLCPP_INFO(node_->get_logger(), "MySensorDriver started");
    return true;
}

bool MySensorDriver::stop() {
    timer_->cancel();
    RCLCPP_INFO(node_->get_logger(), "MySensorDriver stopped");
    return true;
}

void MySensorDriver::shutdown() {
    stop();
    imu_pub_.reset();
    node_.reset();
}

sensor_interfaces::HealthStatus MySensorDriver::getHealthStatus() {
    sensor_interfaces::HealthStatus health;
    health.score = 1.0;  // Perfect health
    health.flags = sensor_interfaces::FaultFlags::NONE;
    health.timestamp = node_->now();
    health.publication_rate_hz = 100.0;
    health.expected_rate_hz = 100.0;
    health.details = "Operating normally";
    return health;
}

sensor_interfaces::SensorCapability MySensorDriver::getCapabilities() {
    return sensor_interfaces::SensorCapability::IMU_DATA;
}

bool MySensorDriver::loadCalibration(const std::string& path) {
    // Load calibration from file
    // Implementation depends on your calibration format
    return true;
}

sensor_interfaces::SensorCalibration MySensorDriver::getCalibration() {
    return calibration_;
}

void MySensorDriver::registerTimeSync(
    std::shared_ptr<sensor_interfaces::TimeSynchronizer> sync) {
    time_sync_ = sync;
}

void MySensorDriver::publishImuData() {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = node_->now();
    msg.header.frame_id = "my_sensor_imu";
    
    // Fill with your sensor data
    msg.linear_acceleration.x = 0.0;
    msg.linear_acceleration.y = 0.0;
    msg.linear_acceleration.z = 9.81;
    
    // Register timestamp for sync monitoring
    if (time_sync_) {
        time_sync_->registerTimestamp("my_sensor", msg.header.stamp);
    }
    
    imu_pub_->publish(msg);
}

} // namespace my_sensor_provider
```

### Step 4: Plugin Registration

Create `my_sensor_provider_plugin.xml`:

```xml
<library path="my_sensor_provider">
  <class name="my_sensor_provider::MySensorDriver"
         type="my_sensor_provider::MySensorDriver"
         base_class_type="sensor_interfaces::SensorDriver">
    <description>
      My custom sensor driver for juppiter.
    </description>
  </class>
</library>
```

### Step 5: CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_sensor_provider)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)

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

pluginlib_export_plugin_description_file(
  sensor_interfaces my_sensor_provider_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
```

### Step 6: Package Configuration

Add to `config/compute_profiles/*.yaml`:

```yaml
providers:
  - my_sensor_provider::MySensorDriver
  - euroc_provider::EurocDriver
```

## Loading Plugins at Runtime

The Sensor Core loads plugins using `pluginlib`:

```cpp
// sensor_core/src/sensor_bridge_node.cpp

void SensorBridgeNode::loadProviders() {
    pluginlib::ClassLoader<sensor_interfaces::SensorDriver> 
        loader("sensor_interfaces", "sensor_interfaces::SensorDriver");
    
    for (const auto& class_name : config_.providers) {
        try {
            auto provider = loader.createSharedInstance(class_name);
            
            if (provider->initialize(shared_from_this())) {
                providers_.push_back(provider);
                RCLCPP_INFO(get_logger(), "Loaded provider: %s", 
                           class_name.c_str());
            }
        } catch (const pluginlib::PluginlibException& ex) {
            RCLCPP_ERROR(get_logger(), "Failed to load %s: %s",
                        class_name.c_str(), ex.what());
        }
    }
}
```

## Interface Versioning

Interfaces use semantic versioning:

```cpp
// In sensor_driver.hpp
static constexpr uint32_t INTERFACE_VERSION = 1;
static constexpr uint32_t INTERFACE_MIN_VERSION = 1;
```

When breaking changes are made:
1. Increment major version
2. Update all implementing plugins
3. Document migration path

## Best Practices

### Do:
- ✓ Implement all virtual methods
- ✓ Report realistic health scores
- ✓ Use consistent frame IDs
- ✓ Register timestamps with TimeSynchronizer
- ✓ Handle exceptions gracefully
- ✓ Log initialization progress

### Don't:
- ✗ Block in start()/stop()
- ✗ Assume ROS node is always valid
- ✗ Hardcode topic names (use parameters)
- ✗ Ignore calibration data
- ✗ Swallow exceptions without logging

## Next Topics

- [Sensor Integration Guide](../user-guides/sensor-integration.md) - Detailed plugin development
- [Configuration Guide](../user-guides/configuration.md) - YAML configuration
- [Architecture Overview](overview.md) - System architecture
