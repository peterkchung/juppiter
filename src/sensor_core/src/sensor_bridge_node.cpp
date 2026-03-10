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
// About: Main sensor orchestration node. Loads providers via pluginlib,
// monitors time sync, and provides health data via on-demand service.
// Does NOT publish mode - mode authority belongs to fusion_core.

#include "sensor_core/sensor_bridge_node.hpp"

namespace sensor_core
{

SensorBridgeNode::SensorBridgeNode()
: Node("sensor_bridge")
{
  // Declare parameters
  declare_parameter<std::vector<std::string>>("providers", {});
  declare_parameter<std::string>("calibration_version", "unknown");
  declare_parameter<double>("monitor_rate_hz", 10.0);
  declare_parameter<double>("target_skew_ms", 5.0);
  declare_parameter<double>("max_skew_ms", 10.0);

  // Read parameters
  provider_classes_ = get_parameter("providers").as_string_array();
  calibration_version_ = get_parameter("calibration_version").as_string();
  monitor_rate_hz_ = get_parameter("monitor_rate_hz").as_double();

  RCLCPP_INFO(get_logger(), "Sensor Bridge starting...");
  RCLCPP_INFO(get_logger(), "Calibration version: %s", calibration_version_.c_str());
  RCLCPP_INFO(get_logger(), "Providers to load: %zu", provider_classes_.size());

  // Initialize monitoring components
  double target_skew = get_parameter("target_skew_ms").as_double();
  double max_skew = get_parameter("max_skew_ms").as_double();
  time_sync_ = std::make_unique<TimeSynchronizer>(target_skew, max_skew);

  // Initialize health monitor (no thresholds - fusion_core determines mode)
  health_monitor_ = std::make_unique<HealthMonitor>(shared_from_this());

  // Initialize plugin loader
  try {
    driver_loader_ = std::make_unique<pluginlib::ClassLoader<sensor_interfaces::SensorDriver>>(
      "sensor_interfaces", "sensor_interfaces::SensorDriver");
    RCLCPP_INFO(get_logger(), "Plugin loader initialized");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize plugin loader: %s", e.what());
    throw;
  }

  // Create health service (on-demand, no continuous publishing)
  health_service_ = create_service<common_msgs::srv::GetPerceptionHealth>(
    "/perception/get_health",
    std::bind(&SensorBridgeNode::on_health_request, this, 
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "Health service ready: /perception/get_health");

  // Load and start providers
  load_providers();
  start_providers();

  // Start monitoring loop (for time sync, not for publishing)
  auto monitor_period = std::chrono::duration<double>(1.0 / monitor_rate_hz_);
  monitor_timer_ = create_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(monitor_period),
    std::bind(&SensorBridgeNode::monitor_loop, this));

  RCLCPP_INFO(get_logger(), "Sensor Bridge ready");
}

SensorBridgeNode::~SensorBridgeNode()
{
  stop_providers();
}

void SensorBridgeNode::load_providers()
{
  for (const auto & class_name : provider_classes_) {
    try {
      RCLCPP_INFO(get_logger(), "Loading provider: %s", class_name.c_str());
      
      auto driver = driver_loader_->createSharedInstance(class_name);
      
      // Initialize the driver
      std::string config_path = "";  // Could be parameterized per-provider
      if (!driver->initialize(shared_from_this(), config_path)) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize provider: %s", class_name.c_str());
        continue;
      }

      // Register for monitoring
      std::string driver_name = driver->get_name();
      providers_[driver_name] = driver;
      time_sync_->register_sensor(driver_name);
      health_monitor_->register_driver(driver_name, driver);

      RCLCPP_INFO(get_logger(), "Provider loaded: %s", driver_name.c_str());
      
      // Log capabilities
      auto caps = driver->get_capabilities();
      for (const auto & cap : caps) {
        RCLCPP_DEBUG(get_logger(), "  - Capability: %d", static_cast<int>(cap));
      }

    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to load provider %s: %s", 
                   class_name.c_str(), e.what());
    }
  }

  if (providers_.empty()) {
    RCLCPP_WARN(get_logger(), "No providers loaded!");
  }
}

void SensorBridgeNode::start_providers()
{
  for (auto & [name, driver] : providers_) {
    try {
      if (driver->start()) {
        RCLCPP_INFO(get_logger(), "Started provider: %s", name.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to start provider: %s", name.c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Exception starting provider %s: %s", 
                   name.c_str(), e.what());
    }
  }
}

void SensorBridgeNode::stop_providers()
{
  for (auto & [name, driver] : providers_) {
    try {
      driver->stop();
      driver->shutdown();
      RCLCPP_INFO(get_logger(), "Stopped provider: %s", name.c_str());
    } catch (const std::exception & e) {
      RCPP_ERROR(get_logger(), "Exception stopping provider %s: %s", 
                   name.c_str(), e.what());
    }
  }
  providers_.clear();
}

void SensorBridgeNode::monitor_loop()
{
  // Update timestamps from all providers
  for (const auto & [name, driver] : providers_) {
    auto timestamp = driver->get_last_timestamp();
    time_sync_->update_timestamp(name, timestamp);
  }

  // Get sync status and update health monitor
  auto sync_status = time_sync_->get_status();
  health_monitor_->set_sync_status(sync_status);

  // Log warnings if out of sync
  if (!sync_status.is_valid) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Time sync violation: mean=%.2fms, p95=%.2fms",
      sync_status.mean_skew_ms, sync_status.p95_skew_ms);
  }
}

void SensorBridgeNode::on_health_request(
  const std::shared_ptr<common_msgs::srv::GetPerceptionHealth::Request> /*request*/,
  std::shared_ptr<common_msgs::srv::GetPerceptionHealth::Response> response)
{
  // Aggregate health on-demand
  response->health = health_monitor_->aggregate_health();
  
  RCLCPP_DEBUG(get_logger(), 
    "Health request: overall=%.2f, lidar=%.2f, camera=%.2f, imu=%.2f, sync=%s",
    response->health.overall_health,
    response->health.lidar_health,
    response->health.camera_health,
    response->health.imu_health,
    response->health.time_sync_valid ? "valid" : "violation");
}

}  // namespace sensor_core
