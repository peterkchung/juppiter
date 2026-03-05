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
// About: Implementation of time synchronization monitoring.

#include "sensor_core/time_synchronizer.hpp"

#include <algorithm>
#include <numeric>

namespace sensor_core
{

TimeSynchronizer::TimeSynchronizer(double target_skew_ms, double max_skew_ms)
: target_skew_ms_(target_skew_ms), max_skew_ms_(max_skew_ms)
{
}

void TimeSynchronizer::register_sensor(const std::string & sensor_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  sensors_[sensor_name] = SensorTimeState{};
}

void TimeSynchronizer::update_timestamp(
  const std::string & sensor_name,
  std::chrono::nanoseconds timestamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = sensors_.find(sensor_name);
  if (it != sensors_.end()) {
    it->second.last_timestamp = timestamp;
    it->second.last_update = std::chrono::steady_clock::now();
  }
}

TimeSyncStatus TimeSynchronizer::get_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  TimeSyncStatus status;
  status.sync_policy = "software";
  status.last_sync_check = std::chrono::steady_clock::now();

  if (sensors_.size() < 2) {
    status.is_synchronized = true;  // Can't be out of sync with only one sensor
    return status;
  }

  // Collect all timestamps
  std::vector<std::chrono::nanoseconds> timestamps;
  for (const auto & [name, state] : sensors_) {
    if (state.last_timestamp.count() > 0) {
      timestamps.push_back(state.last_timestamp);
    }
  }

  if (timestamps.size() < 2) {
    status.is_synchronized = false;
    return status;
  }

  // Compute pairwise skews
  std::vector<double> skews_ms;
  for (size_t i = 0; i < timestamps.size(); ++i) {
    for (size_t j = i + 1; j < timestamps.size(); ++j) {
      double skew_ms = std::abs(
        std::chrono::duration<double, std::milli>(timestamps[i] - timestamps[j]).count());
      skews_ms.push_back(skew_ms);
    }
  }

  if (!skews_ms.empty()) {
    status.mean_skew_ms = std::accumulate(skews_ms.begin(), skews_ms.end(), 0.0) / skews_ms.size();
    
    std::sort(skews_ms.begin(), skews_ms.end());
    size_t p95_idx = static_cast<size_t>(skews_ms.size() * 0.95);
    status.p95_skew_ms = skews_ms[std::min(p95_idx, skews_ms.size() - 1)];
    status.max_skew_ms = skews_ms.back();
    
    status.is_synchronized = status.mean_skew_ms <= target_skew_ms_ &&
                            status.p95_skew_ms <= max_skew_ms_;
  }

  return status;
}

bool TimeSynchronizer::is_synchronized() const
{
  return get_status().is_synchronized;
}

double TimeSynchronizer::get_skew_ms(
  const std::string & sensor_a,
  const std::string & sensor_b) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it_a = sensors_.find(sensor_a);
  auto it_b = sensors_.find(sensor_b);
  
  if (it_a == sensors_.end() || it_b == sensors_.end()) {
    return -1.0;  // Invalid
  }
  
  return std::abs(
    std::chrono::duration<double, std::milli>(
      it_a->second.last_timestamp - it_b->second.last_timestamp).count());
}

std::vector<std::string> TimeSynchronizer::get_registered_sensors() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> names;
  for (const auto & [name, state] : sensors_) {
    names.push_back(name);
  }
  return names;
}

}  // namespace sensor_core
