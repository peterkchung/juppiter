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
// About: Monitors inter-sensor timestamp skew for time synchronization.
// Tracks time offsets between multiple sensors and reports sync status.

#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "sensor_interfaces/time_sync_types.hpp"

namespace sensor_core
{

using sensor_interfaces::TimeSyncStatus;

/**
 * @brief Monitors time synchronization across multiple sensors.
 */
class TimeSynchronizer {
public:
  explicit TimeSynchronizer(
    double target_skew_ms = 5.0,
    double max_skew_ms = 10.0);

  /**
   * @brief Register a sensor for monitoring.
   */
  void register_sensor(const std::string & sensor_name);

  /**
   * @brief Update timestamp for a sensor.
   */
  void update_timestamp(
    const std::string & sensor_name,
    std::chrono::nanoseconds timestamp);

  /**
   * @brief Get current synchronization status.
   */
  TimeSyncStatus get_status() const;

  /**
   * @brief Check if all sensors are within target skew.
   */
  bool is_synchronized() const;

  /**
   * @brief Get skew between two specific sensors.
   */
  double get_skew_ms(
    const std::string & sensor_a,
    const std::string & sensor_b) const;

  /**
   * @brief Get list of registered sensors.
   */
  std::vector<std::string> get_registered_sensors() const;

private:
  struct SensorTimeState {
    std::chrono::nanoseconds last_timestamp{0};
    std::chrono::steady_clock::time_point last_update;
  };

  double target_skew_ms_;
  double max_skew_ms_;
  std::map<std::string, SensorTimeState> sensors_;
  mutable std::mutex mutex_;
};

}  // namespace sensor_core
