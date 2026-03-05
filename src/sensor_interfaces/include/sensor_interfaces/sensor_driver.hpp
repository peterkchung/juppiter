// Copyright 2026 Arconic Labs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// About: Abstract interface for sensor driver implementations.
// Provides a unified contract for all sensor providers (EuRoC, RealSense,
// lidar, etc.) to enable plugin-based loading and monitoring.

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace sensor_interfaces
{

/**
 * @brief Sensor capability flags for capability discovery.
 */
enum class SensorCapability {
  COLOR_IMAGE,
  DEPTH_IMAGE,
  IMU,
  LIDAR_POINTS,
  STEREO_LEFT,
  STEREO_RIGHT
};

/**
 * @brief Health status of a sensor driver.
 */
struct HealthStatus {
  bool is_healthy{false};
  std::string status_message;
  float health_score{0.0f};  // 0.0 to 1.0
  std::vector<std::string> active_faults;
  std::chrono::nanoseconds last_timestamp{0};
  std::chrono::nanoseconds staleness{0};
};

/**
 * @brief Abstract base class for all sensor drivers.
 * 
 * Implementations must provide lifecycle management, health monitoring,
 and timestamp reporting for time synchronization.
 */
class SensorDriver {
public:
  virtual ~SensorDriver() = default;

  /**
   * @brief Initialize the driver with configuration.
   * @param node Parent ROS node for topic creation
   * @param config_path Path to provider-specific configuration
   * @return true if initialization successful
   */
  virtual bool initialize(
    const rclcpp::Node::SharedPtr & node,
    const std::string & config_path) = 0;

  /**
   * @brief Start data acquisition and publishing.
   * @return true if started successfully
   */
  virtual bool start() = 0;

  /**
   * @brief Stop data acquisition.
   */
  virtual void stop() = 0;

  /**
   * @brief Shutdown and release resources.
   */
  virtual void shutdown() = 0;

  /**
   * @brief Get driver name for identification.
   */
  virtual std::string get_name() const = 0;

  /**
   * @brief Check if driver supports specific capability.
   */
  virtual bool has_capability(SensorCapability cap) const = 0;

  /**
   * @brief Get all supported capabilities.
   */
  virtual std::vector<SensorCapability> get_capabilities() const = 0;

  /**
   * @brief Get current health status.
   */
  virtual HealthStatus get_health() const = 0;

  /**
   * @brief Get timestamp of most recent data.
   * Required for time synchronization monitoring.
   */
  virtual std::chrono::nanoseconds get_last_timestamp() const = 0;

  /**
   * @brief Get topics published by this driver.
   */
  virtual std::vector<std::string> get_published_topics() const = 0;
};

}  // namespace sensor_interfaces
