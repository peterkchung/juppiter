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
// About: Implementation of health monitoring for on-demand service queries.
// Aggregates sensor and estimator health. Does NOT determine system mode.

#include "sensor_core/health_monitor.hpp"

#include <algorithm>

namespace sensor_core
{

HealthMonitor::HealthMonitor(rclcpp::Node::SharedPtr node)
: node_(node)
{
}

void HealthMonitor::register_driver(
  const std::string & name,
  std::shared_ptr<sensor_interfaces::SensorDriver> driver)
{
  std::lock_guard<std::mutex> lock(mutex_);
  drivers_[name] = driver;
}

common_msgs::msg::PerceptionHealth HealthMonitor::aggregate_health()
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  // Collect health from all registered drivers (sensors)
  std::map<std::string, sensor_interfaces::HealthStatus> statuses;
  for (const auto & [name, driver] : drivers_) {
    statuses[name] = driver->get_health_status();
  }
  
  // Build health message (ARM-optimized field ordering)
  common_msgs::msg::PerceptionHealth health;
  health.stamp = node_->now();
  
  // Sensor-level health scores
  for (const auto & [name, status] : statuses) {
    if (name.find("lidar") != std::string::npos) {
      health.lidar_health = status.score;
    } else if (name.find("camera") != std::string::npos || 
               name.find("stereo") != std::string::npos) {
      health.camera_health = status.score;
    } else if (name.find("imu") != std::string::npos) {
      health.imu_health = status.score;
    }
  }
  
  // Time sync status
  health.time_sync_skew_ms = sync_status_.skew_max_ms;
  health.time_sync_valid = sync_status_.is_valid;
  
  // Fault flags
  health.fault_flags = compute_fault_flags(statuses);
  
  // Note: Estimator health (lio, vio, kinematic) is monitored by fusion_core
  // sensor_core only monitors raw sensor health
  health.lio_health = 1.0f;  // Placeholder - fusion_core tracks this
  health.vio_health = 1.0f;
  health.kinematic_health = 1.0f;
  health.overall_health = 1.0f;  // Placeholder
  
  // Mode is determined by fusion_core, not sensor_core
  health.mode = common_msgs::msg::PerceptionHealth::MODE_NOMINAL;
  health.mode_string = "nominal";
  health.service_responsive = true;
  
  // Cache for quick retrieval
  cached_health_ = health;
  
  return health;
}

common_msgs::msg::PerceptionHealth HealthMonitor::get_cached_health() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return cached_health_;
}

void HealthMonitor::set_sync_status(const sensor_interfaces::TimeSyncStatus & status)
{
  std::lock_guard<std::mutex> lock(mutex_);
  sync_status_ = status;
}

uint16_t HealthMonitor::compute_fault_flags(
  const std::map<std::string, sensor_interfaces::HealthStatus> & statuses)
{
  uint16_t flags = common_msgs::msg::PerceptionHealth::FAULT_NONE;
  
  for (const auto & [name, status] : statuses) {
    // Check for stale data
    if (status.flags & sensor_interfaces::FaultFlags::STALE) {
      if (name.find("lidar") != std::string::npos) {
        flags |= common_msgs::msg::PerceptionHealth::FAULT_LIDAR_STALE;
      } else if (name.find("camera") != std::string::npos) {
        flags |= common_msgs::msg::PerceptionHealth::FAULT_CAMERA_STALE;
      } else if (name.find("imu") != std::string::npos) {
        flags |= common_msgs::msg::PerceptionHealth::FAULT_IMU_STALE;
      }
    }
  }
  
  // Time sync fault
  if (!sync_status_.is_valid) {
    flags |= common_msgs::msg::PerceptionHealth::FAULT_TIME_SYNC;
  }
  
  return flags;
}

}  // namespace sensor_core
