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
// About: Implementation of health monitoring and aggregation.

#include "sensor_core/health_monitor.hpp"

#include <algorithm>

namespace sensor_core
{

HealthMonitor::HealthMonitor(
  rclcpp::Node::SharedPtr node,
  double nominal_threshold,
  double degraded_threshold)
: node_(node),
  nominal_threshold_(nominal_threshold),
  degraded_threshold_(degraded_threshold)
{
}

void HealthMonitor::register_driver(
  const std::string & name,
  std::shared_ptr<sensor_interfaces::SensorDriver> driver)
{
  std::lock_guard<std::mutex> lock(mutex_);
  drivers_[name] = driver;
}

PerceptionHealth HealthMonitor::update()
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  // Collect health from all drivers
  std::map<std::string, sensor_interfaces::HealthStatus> statuses;
  for (const auto & [name, driver] : drivers_) {
    statuses[name] = driver->get_health();
  }
  
  // Compute scores
  current_health_.stamp = node_->now();
  current_health_.overall_score = compute_overall_score(statuses);
  
  // Determine mode
  current_health_.estimator_mode = determine_mode();
  
  // Check sync status
  if (!sync_status_.is_synchronized) {
    current_health_.fault_flags.push_back("time_sync_violation");
  }
  
  return current_health_;
}

PerceptionHealth HealthMonitor::get_health() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_health_;
}

std::string HealthMonitor::determine_mode() const
{
  if (current_health_.overall_score >= nominal_threshold_) {
    return "nominal";
  } else if (current_health_.overall_score < degraded_threshold_) {
    return "safe_stop";
  } else {
    // Determine which subsystem is failing
    if (current_health_.lidar_score < current_health_.vision_score) {
      return "degraded_lidar";
    } else {
      return "degraded_vision";
    }
  }
}

void HealthMonitor::set_sync_status(const sensor_interfaces::TimeSyncStatus & status)
{
  std::lock_guard<std::mutex> lock(mutex_);
  sync_status_ = status;
}

float HealthMonitor::compute_overall_score(
  const std::map<std::string, sensor_interfaces::HealthStatus> & statuses)
{
  if (statuses.empty()) {
    return 0.0f;
  }
  
  // Compute weighted raw score (PRD formula starting point)
  // Note: Full PRD formula requires more context (covariance, staleness, etc.)
  float total_score = 0.0f;
  float total_weight = 0.0f;
  
  for (const auto & [name, status] : statuses) {
    total_score += status.health_score * 0.33f;  // Equal weight for now
    total_weight += 0.33f;
    
    // Track subsystem scores
    if (name.find("lidar") != std::string::npos) {
      current_health_.lidar_score = status.health_score;
    } else if (name.find("camera") != std::string::npos || name.find("stereo") != std::string::npos) {
      current_health_.vision_score = status.health_score;
    } else if (name.find("imu") != std::string::npos) {
      current_health_.imu_score = status.health_score;
    }
  }
  
  return total_weight > 0 ? total_score / total_weight : 0.0f;
}

}  // namespace sensor_core
