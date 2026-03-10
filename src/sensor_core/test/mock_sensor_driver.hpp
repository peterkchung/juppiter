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
// About: Manual mock for SensorDriver interface for unit testing

#pragma once

#include <string>
#include <vector>

#include "sensor_interfaces/sensor_driver.hpp"
#include "sensor_interfaces/time_sync_types.hpp"

namespace sensor_core
{
namespace testing
{

/**
 * @brief Manual mock implementation of SensorDriver for testing
 * 
 * This is a lightweight manual mock (no gmock dependency)
 * that allows setting return values and tracking method calls.
 */
class MockSensorDriver : public sensor_interfaces::SensorDriver
{
public:
  MockSensorDriver(const std::string & name)
  : name_(name)
  {
  }

  // Configuration
  void setHealthStatus(const sensor_interfaces::HealthStatus & status)
  {
    health_status_ = status;
  }

  void setCapabilities(sensor_interfaces::SensorCapability caps)
  {
    capabilities_ = caps;
  }

  void setLastTimestamp(const rclcpp::Time & timestamp)
  {
    last_timestamp_ = timestamp;
  }

  void setInitialized(bool initialized)
  {
    initialized_ = initialized;
  }

  // Tracking
  int getInitializeCallCount() const { return initialize_count_; }
  int getStartCallCount() const { return start_count_; }
  int getStopCallCount() const { return stop_count_; }
  int getShutdownCallCount() const { return shutdown_count_; }

  // SensorDriver interface implementation
  bool initialize(rclcpp::Node::SharedPtr node, const std::string & config_path) override
  {
    initialize_count_++;
    node_ = node;
    config_path_ = config_path;
    return initialized_;
  }

  bool start() override
  {
    start_count_++;
    return true;
  }

  bool stop() override
  {
    stop_count_++;
    return true;
  }

  void shutdown() override
  {
    shutdown_count_++;
  }

  sensor_interfaces::HealthStatus getHealthStatus() override
  {
    return health_status_;
  }

  sensor_interfaces::SensorCapability getCapabilities() override
  {
    return capabilities_;
  }

  bool loadCalibration(const std::string & path) override
  {
    calibration_path_ = path;
    return true;
  }

  sensor_interfaces::SensorCalibration getCalibration() override
  {
    return calibration_;
  }

  void registerTimeSync(std::shared_ptr<sensor_interfaces::TimeSynchronizer> sync) override
  {
    time_sync_ = sync;
  }

  std::string getName() const { return name_; }
  rclcpp::Time getLastTimestamp() const { return last_timestamp_; }

private:
  std::string name_;
  rclcpp::Node::SharedPtr node_;
  std::string config_path_;
  std::string calibration_path_;
  
  sensor_interfaces::HealthStatus health_status_;
  sensor_interfaces::SensorCapability capabilities_{sensor_interfaces::SensorCapability::NONE};
  sensor_interfaces::SensorCalibration calibration_;
  std::shared_ptr<sensor_interfaces::TimeSynchronizer> time_sync_;
  
  rclcpp::Time last_timestamp_{0, 0, RCL_ROS_TIME};
  bool initialized_{true};
  
  // Call tracking
  int initialize_count_{0};
  int start_count_{0};
  int stop_count_{0};
  int shutdown_count_{0};
};

}  // namespace testing
}  // namespace sensor_core
