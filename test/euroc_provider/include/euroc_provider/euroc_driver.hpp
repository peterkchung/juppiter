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
// About: EuRoC dataset driver implementing SensorDriver interface.
// Replays EuRoC MAV datasets with wall-clock timing, publishing stereo
// images, depth, and IMU data.

#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "sensor_interfaces/sensor_driver.hpp"

namespace euroc_provider
{

// Forward declarations
class EurocReader;
class StereoDepth;

/**
 * @brief SensorDriver implementation for EuRoC MAV dataset replay.
 */
class EurocDriver : public sensor_interfaces::SensorDriver {
public:
  EurocDriver();
  ~EurocDriver() override;

  // SensorDriver interface implementation
  bool initialize(
    const rclcpp::Node::SharedPtr & node,
    const std::string & config_path) override;
  bool start() override;
  void stop() override;
  void shutdown() override;

  std::string get_name() const override { return "euroc_driver"; }
  bool has_capability(sensor_interfaces::SensorCapability cap) const override;
  std::vector<sensor_interfaces::SensorCapability> get_capabilities() const override;
  sensor_interfaces::HealthStatus get_health() const override;
  std::chrono::nanoseconds get_last_timestamp() const override;
  std::vector<std::string> get_published_topics() const override;

private:
  void timer_callback();
  void publish_imu_up_to(int64_t target_ns);
  void publish_frame(size_t frame_idx);
  rclcpp::Time to_ros_time(int64_t timestamp_ns) const;
  void reset_playback();

  // ROS node reference (provided during initialize)
  rclcpp::Node::SharedPtr node_;

  // Dataset components
  std::unique_ptr<EurocReader> reader_;
  std::unique_ptr<StereoDepth> depth_engine_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_cam_info_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  std::string dataset_path_;
  std::string frame_id_;
  double playback_rate_{1.0};
  bool loop_{false};
  bool publish_depth_{true};

  // Playback state
  rclcpp::Time wall_start_;
  int64_t dataset_start_ns_{0};
  size_t next_frame_idx_{0};
  size_t next_imu_idx_{0};
  bool is_running_{false};

  // Camera info (built once from params)
  sensor_msgs::msg::CameraInfo cam_info_msg_;

  // Last timestamp for health monitoring
  mutable std::mutex timestamp_mutex_;
  std::chrono::nanoseconds last_timestamp_{0};
};

}  // namespace euroc_provider
