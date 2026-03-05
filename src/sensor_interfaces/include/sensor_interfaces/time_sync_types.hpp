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
// About: Data structures for time synchronization and calibration management.

#pragma once

#include <array>
#include <chrono>
#include <string>
#include <vector>

namespace sensor_interfaces
{

/**
 * @brief Time synchronization policy.
 */
enum class TimePolicy {
  SOFTWARE,   // ROS time only, monitor skew
  HARDWARE_PPS  // Hardware PPS signal for tight sync
};

/**
 * @brief Status of time synchronization between sensors.
 */
struct TimeSyncStatus {
  bool is_synchronized{false};
  double mean_skew_ms{0.0};
  double p95_skew_ms{0.0};
  double max_skew_ms{0.0};
  std::string sync_policy{"software"};
  std::chrono::steady_clock::time_point last_sync_check;
};

/**
 * @brief 3D transformation (translation + quaternion rotation).
 */
struct Transform3D {
  std::array<double, 3> translation{0.0, 0.0, 0.0};
  std::array<double, 4> rotation{0.0, 0.0, 0.0, 1.0};  // x, y, z, w quaternion
};

/**
 * @brief Camera intrinsic parameters.
 */
struct CameraIntrinsics {
  int width{0};
  int height{0};
  double fx{0.0}, fy{0.0};  // Focal lengths
  double cx{0.0}, cy{0.0};  // Principal point
  std::array<double, 5> distortion{0.0, 0.0, 0.0, 0.0, 0.0};  // k1, k2, p1, p2, k3
  std::string distortion_model{"plumb_bob"};
};

/**
 * @brief Calibration data for a multi-sensor rig.
 */
struct SensorCalibration {
  std::string version;  // Semantic version string
  std::string timestamp;  // ISO 8601 timestamp
  std::string description;
  
  struct SensorConfig {
    std::string name;
    std::string type;  // "camera", "lidar", "imu"
    std::string parent_frame;  // Frame this sensor is attached to
    Transform3D extrinsics;
    // Type-specific parameters
    CameraIntrinsics camera_intrinsics;  // Valid if type == "camera"
  };
  
  std::vector<SensorConfig> sensors;
};

}  // namespace sensor_interfaces
