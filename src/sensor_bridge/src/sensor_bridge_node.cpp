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
// About: ROS 2 sensor_bridge node implementation.
// Replays EuRoC MAV datasets with wall-clock timing, interleaving IMU before
// camera frames to preserve causal ordering for VIO.

#include "sensor_bridge/sensor_bridge_node.hpp"

#include <chrono>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>

#include "euroc_reader.hpp"
#include "stereo_depth.hpp"

using namespace std::chrono_literals;

namespace sensor_bridge
{

SensorBridgeNode::~SensorBridgeNode() = default;

SensorBridgeNode::SensorBridgeNode()
: Node("sensor_bridge_node")
{
  // Declare parameters
  declare_parameter<std::string>("dataset_path", "");
  declare_parameter<double>("playback_rate", 1.0);
  declare_parameter<bool>("loop", false);
  declare_parameter<bool>("publish_depth", true);
  declare_parameter<std::string>("frame_id", "camera_link");
  declare_parameter<int>("image_width", 752);
  declare_parameter<int>("image_height", 480);
  declare_parameter<double>("cam0_fx", 458.654);
  declare_parameter<double>("cam0_fy", 457.296);
  declare_parameter<double>("cam0_cx", 367.215);
  declare_parameter<double>("cam0_cy", 248.375);
  declare_parameter<std::vector<double>>("cam0_d",
    {-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0});
  declare_parameter<double>("cam1_fx", 457.587);
  declare_parameter<double>("cam1_fy", 456.134);
  declare_parameter<double>("cam1_cx", 379.999);
  declare_parameter<double>("cam1_cy", 255.238);
  declare_parameter<std::vector<double>>("cam1_d",
    {-0.28368365, 0.07451284, -0.00010473, -3.55590700e-05, 0.0});
  declare_parameter<double>("baseline_m", 0.110074);
  declare_parameter<std::vector<double>>("R_cam0_cam1",
    {0.9999, 0.0098, -0.0074, -0.0099, 0.9999, -0.0044, 0.0074, 0.0045, 0.9999});
  declare_parameter<std::vector<double>>("T_cam0_cam1",
    {-0.110074, 0.000399, 0.000853});
  declare_parameter<int>("num_disparities", 96);
  declare_parameter<int>("block_size", 11);

  // Read parameters
  auto dataset_path = get_parameter("dataset_path").as_string();
  playback_rate_ = get_parameter("playback_rate").as_double();
  loop_ = get_parameter("loop").as_bool();
  publish_depth_ = get_parameter("publish_depth").as_bool();
  frame_id_ = get_parameter("frame_id").as_string();

  if (dataset_path.empty()) {
    RCLCPP_ERROR(get_logger(), "dataset_path parameter is required");
    throw std::runtime_error("dataset_path not set");
  }

  // Load dataset
  RCLCPP_INFO(get_logger(), "Loading EuRoC dataset from: %s", dataset_path.c_str());
  reader_ = std::make_unique<EurocReader>(dataset_path);
  RCLCPP_INFO(get_logger(), "Loaded %zu frames, %zu IMU samples",
              reader_->num_frames(), reader_->num_imu_samples());

  // Initialize stereo depth if enabled
  if (publish_depth_) {
    StereoParams sp;
    sp.width = get_parameter("image_width").as_int();
    sp.height = get_parameter("image_height").as_int();
    sp.cam0_fx = get_parameter("cam0_fx").as_double();
    sp.cam0_fy = get_parameter("cam0_fy").as_double();
    sp.cam0_cx = get_parameter("cam0_cx").as_double();
    sp.cam0_cy = get_parameter("cam0_cy").as_double();
    auto d0 = get_parameter("cam0_d").as_double_array();
    for (size_t i = 0; i < 5 && i < d0.size(); ++i) {
      sp.cam0_d[i] = d0[i];
    }
    sp.cam1_fx = get_parameter("cam1_fx").as_double();
    sp.cam1_fy = get_parameter("cam1_fy").as_double();
    sp.cam1_cx = get_parameter("cam1_cx").as_double();
    sp.cam1_cy = get_parameter("cam1_cy").as_double();
    auto d1 = get_parameter("cam1_d").as_double_array();
    for (size_t i = 0; i < 5 && i < d1.size(); ++i) {
      sp.cam1_d[i] = d1[i];
    }
    sp.baseline_m = get_parameter("baseline_m").as_double();
    auto R = get_parameter("R_cam0_cam1").as_double_array();
    for (size_t i = 0; i < 9 && i < R.size(); ++i) {
      sp.R_cam0_cam1[i] = R[i];
    }
    auto T = get_parameter("T_cam0_cam1").as_double_array();
    for (size_t i = 0; i < 3 && i < T.size(); ++i) {
      sp.T_cam0_cam1[i] = T[i];
    }
    sp.num_disparities = get_parameter("num_disparities").as_int();
    sp.block_size = get_parameter("block_size").as_int();

    depth_engine_ = std::make_unique<StereoDepth>(sp);
  }

  // Build CameraInfo message (constant across all frames)
  cam_info_msg_.header.frame_id = frame_id_;
  cam_info_msg_.width = get_parameter("image_width").as_int();
  cam_info_msg_.height = get_parameter("image_height").as_int();
  cam_info_msg_.distortion_model = "plumb_bob";
  auto d0 = get_parameter("cam0_d").as_double_array();
  cam_info_msg_.d = d0;
  double fx = get_parameter("cam0_fx").as_double();
  double fy = get_parameter("cam0_fy").as_double();
  double cx = get_parameter("cam0_cx").as_double();
  double cy = get_parameter("cam0_cy").as_double();
  cam_info_msg_.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  cam_info_msg_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  cam_info_msg_.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};

  // Create publishers
  pub_color_ = create_publisher<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", 10);
  pub_depth_ = create_publisher<sensor_msgs::msg::Image>(
      "/camera/depth/image_rect_raw", 10);
  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>(
      "/camera/imu", 100);
  pub_cam_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/color/camera_info", 10);

  // Initialize playback state
  reset_playback();

  // 200Hz wall timer — fast enough for IMU interleaving
  timer_ = create_wall_timer(5ms, std::bind(&SensorBridgeNode::timer_callback, this));
  RCLCPP_INFO(get_logger(), "Replay started (rate=%.1f, loop=%s, depth=%s)",
              playback_rate_, loop_ ? "true" : "false",
              publish_depth_ ? "true" : "false");
}

void SensorBridgeNode::timer_callback()
{
  if (next_frame_idx_ >= reader_->num_frames()) {
    if (loop_) {
      RCLCPP_INFO(get_logger(), "Looping dataset");
      reset_playback();
    } else {
      timer_->cancel();
      RCLCPP_INFO(get_logger(), "Dataset replay complete");
      return;
    }
  }

  // Compute elapsed dataset time from wall clock
  auto wall_now = now();
  double wall_elapsed_s = (wall_now - wall_start_).seconds();

  int64_t dataset_elapsed_ns;
  if (playback_rate_ <= 0.0) {
    // Fast-as-possible: jump to the next frame's timestamp
    dataset_elapsed_ns = reader_->image_timestamp_ns(
        std::min(next_frame_idx_, reader_->num_frames() - 1)) - dataset_start_ns_;
  } else {
    dataset_elapsed_ns = static_cast<int64_t>(wall_elapsed_s * playback_rate_ * 1e9);
  }

  int64_t target_ns = dataset_start_ns_ + dataset_elapsed_ns;

  // Publish all IMU samples up to target time (before camera for causal ordering)
  publish_imu_up_to(target_ns);

  // Publish camera frame if its timestamp has been reached
  while (next_frame_idx_ < reader_->num_frames() &&
    reader_->image_timestamp_ns(next_frame_idx_) <= target_ns)
  {
    publish_frame(next_frame_idx_);
    next_frame_idx_++;
  }
}

void SensorBridgeNode::publish_imu_up_to(int64_t target_ns)
{
  const auto & imu = reader_->imu_samples();
  while (next_imu_idx_ < imu.size() && imu[next_imu_idx_].timestamp_ns <= target_ns) {
    const auto & s = imu[next_imu_idx_];

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = to_ros_time(s.timestamp_ns);
    msg.header.frame_id = frame_id_;
    msg.angular_velocity.x = s.gx;
    msg.angular_velocity.y = s.gy;
    msg.angular_velocity.z = s.gz;
    msg.linear_acceleration.x = s.ax;
    msg.linear_acceleration.y = s.ay;
    msg.linear_acceleration.z = s.az;

    pub_imu_->publish(msg);
    next_imu_idx_++;
  }
}

void SensorBridgeNode::publish_frame(size_t frame_idx)
{
  auto ts_ns = reader_->image_timestamp_ns(frame_idx);
  auto stamp = to_ros_time(ts_ns);

  // Load and publish grayscale image
  cv::Mat gray = reader_->load_image(0, frame_idx);
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = frame_id_;

  auto color_msg = cv_bridge::CvImage(header, "mono8", gray).toImageMsg();
  pub_color_->publish(*color_msg);

  // Publish CameraInfo with matching timestamp
  cam_info_msg_.header.stamp = stamp;
  pub_cam_info_->publish(cam_info_msg_);

  // Compute and publish depth if enabled
  if (publish_depth_ && depth_engine_) {
    cv::Mat right = reader_->load_image(1, frame_idx);
    cv::Mat depth = depth_engine_->compute(gray, right);

    auto depth_msg = cv_bridge::CvImage(header, "32FC1", depth).toImageMsg();
    pub_depth_->publish(*depth_msg);
  }
}

rclcpp::Time SensorBridgeNode::to_ros_time(int64_t timestamp_ns) const
{
  return rclcpp::Time(timestamp_ns, RCL_ROS_TIME);
}

void SensorBridgeNode::reset_playback()
{
  wall_start_ = now();
  dataset_start_ns_ = reader_->image_timestamp_ns(0);
  next_frame_idx_ = 0;
  next_imu_idx_ = 0;
}

}  // namespace sensor_bridge
