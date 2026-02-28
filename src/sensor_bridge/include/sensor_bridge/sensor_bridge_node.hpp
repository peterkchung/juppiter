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
// About: ROS 2 node that replays EuRoC MAV datasets as sensor topics.
// Publishes color images, depth maps, IMU data, and camera info.

#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace sensor_bridge
{

// Forward declarations to keep ROS header clean of OpenCV internals
class EurocReader;
class StereoDepth;

class SensorBridgeNode : public rclcpp::Node {
public:
  SensorBridgeNode();
  ~SensorBridgeNode() override;

private:
  void timer_callback();
  void publish_imu_up_to(int64_t target_ns);
  void publish_frame(size_t frame_idx);
  rclcpp::Time to_ros_time(int64_t timestamp_ns) const;
  void reset_playback();

  // Dataset
  std::unique_ptr<EurocReader> reader_;
  std::unique_ptr<StereoDepth> depth_engine_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_cam_info_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  std::string frame_id_;
  double playback_rate_;
  bool loop_;
  bool publish_depth_;

  // Playback state
  rclcpp::Time wall_start_;
  int64_t dataset_start_ns_ = 0;
  size_t next_frame_idx_ = 0;
  size_t next_imu_idx_ = 0;

  // Camera info (built once from params)
  sensor_msgs::msg::CameraInfo cam_info_msg_;
};

}  // namespace sensor_bridge
