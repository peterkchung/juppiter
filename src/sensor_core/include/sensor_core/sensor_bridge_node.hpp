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
// About: Main sensor orchestration node that loads providers via pluginlib,
// monitors time synchronization, and provides health data via on-demand service.

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common_msgs/srv/get_perception_health.hpp"
#include "sensor_core/health_monitor.hpp"
#include "sensor_core/time_synchronizer.hpp"
#include "sensor_interfaces/sensor_driver.hpp"

namespace sensor_core
{

class SensorBridgeNode : public rclcpp::Node {
public:
  SensorBridgeNode();
  ~SensorBridgeNode() override;

private:
  void load_providers();
  void start_providers();
  void stop_providers();
  void monitor_loop();
  
  /**
   * @brief Service callback for on-demand health queries.
   * Responds to fusion_core requests with current aggregated health.
   */
  void on_health_request(
    const std::shared_ptr<common_msgs::srv::GetPerceptionHealth::Request> request,
    std::shared_ptr<common_msgs::srv::GetPerceptionHealth::Response> response);

  // Plugin loader for sensor drivers
  std::unique_ptr<pluginlib::ClassLoader<sensor_interfaces::SensorDriver>> driver_loader_;

  // Loaded provider instances
  std::map<std::string, std::shared_ptr<sensor_interfaces::SensorDriver>> providers_;

  // Core monitoring components
  std::unique_ptr<TimeSynchronizer> time_sync_;
  std::unique_ptr<HealthMonitor> health_monitor_;

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  
  // Health service (on-demand, no publishing)
  rclcpp::Service<common_msgs::srv::GetPerceptionHealth>::SharedPtr health_service_;

  // Parameters
  std::vector<std::string> provider_classes_;
  std::string calibration_version_;
  double monitor_rate_hz_{10.0};
};

}  // namespace sensor_core
