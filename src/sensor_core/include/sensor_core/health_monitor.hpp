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
// About: Aggregates health status from all sensors and publishes perception health.
// Computes overall health score according to PRD formula.

#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "sensor_interfaces/sensor_driver.hpp"
#include "sensor_interfaces/time_sync_types.hpp"

namespace sensor_core
{

/**
 * @brief Perception health message structure (matches PRD specification).
 */
struct PerceptionHealth {
  rclcpp::Time stamp;
  float overall_score{0.0f};  // 0.0 to 1.0
  float lidar_score{0.0f};
  float vision_score{0.0f};
  float imu_score{0.0f};
  float covariance_norm{0.0f};
  uint32_t staleness_ms{0};
  std::string estimator_mode{"unknown"};
  std::vector<std::string> fault_flags;
};

/**
 * @brief Monitors and aggregates health from all sensor drivers.
 */
class HealthMonitor {
public:
  explicit HealthMonitor(
    rclcpp::Node::SharedPtr node,
    double nominal_threshold = 0.75,
    double degraded_threshold = 0.55);

  /**
   * @brief Register a sensor driver for health monitoring.
   */
  void register_driver(
    const std::string & name,
    std::shared_ptr<sensor_interfaces::SensorDriver> driver);

  /**
   * @brief Update health status (called periodically).
   */
  PerceptionHealth update();

  /**
   * @brief Get current health status.
   */
  PerceptionHealth get_health() const;

  /**
   * @brief Determine mode based on health scores.
   * @return "nominal", "degraded_lidar", "degraded_vision", or "safe_stop"
   */
  std::string determine_mode() const;

  /**
   * @brief Set time sync status from TimeSynchronizer.
   */
  void set_sync_status(const sensor_interfaces::TimeSyncStatus & status);

private:
  rclcpp::Node::SharedPtr node_;
  double nominal_threshold_;
  double degraded_threshold_;
  
  std::map<std::string, std::shared_ptr<sensor_interfaces::SensorDriver>> drivers_;
  sensor_interfaces::TimeSyncStatus sync_status_;
  PerceptionHealth current_health_;
  mutable std::mutex mutex_;

  float compute_overall_score(const std::map<std::string, sensor_interfaces::HealthStatus> & statuses);
};

}  // namespace sensor_core
