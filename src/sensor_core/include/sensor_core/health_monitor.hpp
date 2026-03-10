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
// About: Aggregates health status from all sensors for on-demand queries.
// Provides health data to fusion_core via service calls.

#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "common_msgs/msg/perception_health.hpp"
#include "sensor_interfaces/sensor_driver.hpp"
#include "sensor_interfaces/time_sync_types.hpp"

namespace sensor_core
{

/**
 * @brief Monitors and aggregates health from all sensor drivers.
 * Provides on-demand health aggregation via service calls.
 * Does NOT determine system mode - that is fusion_core's responsibility.
 */
class HealthMonitor {
public:
  explicit HealthMonitor(rclcpp::Node::SharedPtr node);

  /**
   * @brief Register a sensor driver for health monitoring.
   */
  void register_driver(
    const std::string & name,
    std::shared_ptr<sensor_interfaces::SensorDriver> driver);

  /**
   * @brief Aggregate current health from all registered drivers.
   * Called on-demand when fusion_core requests health via service.
   * @return Current health status with all sensor and estimator scores.
   */
  common_msgs::msg::PerceptionHealth aggregate_health();

  /**
   * @brief Get last aggregated health (cached).
   */
  common_msgs::msg::PerceptionHealth get_cached_health() const;

  /**
   * @brief Set time sync status from TimeSynchronizer.
   */
  void set_sync_status(const sensor_interfaces::TimeSyncStatus & status);

private:
  rclcpp::Node::SharedPtr node_;
  
  std::map<std::string, std::shared_ptr<sensor_interfaces::SensorDriver>> drivers_;
  sensor_interfaces::TimeSyncStatus sync_status_;
  common_msgs::msg::PerceptionHealth cached_health_;
  mutable std::mutex mutex_;

  /**
   * @brief Compute fault flags bitmask from current status.
   */
  uint16_t compute_fault_flags(
    const std::map<std::string, sensor_interfaces::HealthStatus> & statuses);
};

}  // namespace sensor_core
