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
// About: EuRoC driver implementation.

#include "euroc_provider/euroc_driver.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>

#include "euroc_provider/euroc_reader.hpp"
#include "euroc_provider/stereo_depth.hpp"

using namespace std::chrono_literals;

namespace euroc_provider
{

EurocDriver::EurocDriver() = default;

EurocDriver::~EurocDriver() {
  if (is_running_) {
    stop();
  }
  shutdown();
}

bool EurocDriver::initialize(
  const rclcpp::Node::SharedPtr & node,
  const std::string & /*config_path*/)
{
  node_ = node;
  
  // Declare parameters
  node_->declare_parameter<std::string>("dataset_path", "");
  node_->declare_parameter<double>("playback_rate", 1.0);
  node_->declare_parameter<bool>("loop", false);
  node_->declare_parameter<bool>("publish_depth", true);
  node_->declare_parameter<std::string>("frame_id", "camera_link");
  
  // Read parameters
  dataset_path_ = node_->get_parameter("dataset_path").as_string();
  playback_rate_ = node_->get_parameter("playback_rate").as_double();
  loop_ = node_->get_parameter("loop").as_bool();
  publish_depth_ = node_->get_parameter("publish_depth").as_bool();
  frame_id_ = node_->get_parameter("frame_id").as_string();

  if (dataset_path_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "dataset_path parameter is required");
    return false;
  }

  // Load dataset
  RCLCPP_INFO(node_->get_logger(), "Loading EuRoC dataset from: %s", dataset_path_.c_str());
  try {
    reader_ = std::make_unique<EurocReader>(dataset_path_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load dataset: %s", e.what());
    return false;
  }
  
  RCLCPP_INFO(node_->get_logger(), "Loaded %zu frames, %zu IMU samples",
              reader_->num_frames(), reader_->num_imu_samples());

  // Initialize stereo depth if enabled
  if (publish_depth_) {
    // Use default EuRoC parameters
    StereoParams sp;
    sp.width = 752;
    sp.height = 480;
    sp.cam0_fx = 458.654;
    sp.cam0_fy = 457.296;
    sp.cam0_cx = 367.215;
    sp.cam0_cy = 248.375;
    sp.cam0_d = {-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0};
    sp.cam1_fx = 457.587;
    sp.cam1_fy = 456.134;
    sp.cam1_cx = 379.999;
    sp.cam1_cy = 255.238;
    sp.cam1_d = {-0.28368365, 0.07451284, -0.00010473, -3.55590700e-05, 0.0};
    sp.baseline_m = 0.110074;
    sp.R_cam0_cam1 = {0.9999, 0.0098, -0.0074, -0.0099, 0.9999, -0.0044, 0.0074, 0.0045, 0.9999};
    sp.T_cam0_cam1 = {-0.110074, 0.000399, 0.000853};
    sp.num_disparities = 96;
    sp.block_size = 11;

    try {
      depth_engine_ = std::make_unique<StereoDepth>(sp);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to initialize stereo depth: %s", e.what());
      return false;
    }
  }

  // Build CameraInfo message
  cam_info_msg_.header.frame_id = frame_id_;
  cam_info_msg_.width = 752;
  cam_info_msg_.height = 480;
  cam_info_msg_.distortion_model = "plumb_bob";
  cam_info_msg_.d = {-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0};
  cam_info_msg_.k = {458.654, 0.0, 367.215, 0.0, 457.296, 248.375, 0.0, 0.0, 1.0};
  cam_info_msg_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  cam_info_msg_.p = {458.654, 0.0, 367.215, 0.0, 0.0, 457.296, 248.375, 0.0, 0.0, 0.0, 1.0, 0.0};

  // Create publishers
  pub_left_ = node_->create_publisher<sensor_msgs::msg::Image>("/stereo/left/image_raw", 10);
  pub_right_ = node_->create_publisher<sensor_msgs::msg::Image>("/stereo/right/image_raw", 10);
  pub_depth_ = node_->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_rect_raw", 10);
  pub_imu_ = node_->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 100);
  pub_cam_info_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info", 10);

  return true;
}

bool EurocDriver::start()
{
  if (is_running_) {
    return true;
  }

  reset_playback();

  // 200Hz wall timer for IMU interleaving
  timer_ = node_->create_wall_timer(5ms, std::bind(&EurocDriver::timer_callback, this));
  
  is_running_ = true;
  RCLCPP_INFO(node_->get_logger(), "EuRoC driver started (rate=%.1f, loop=%s, depth=%s)",
              playback_rate_, loop_ ? "true" : "false",
              publish_depth_ ? "true" : "false");
  return true;
}

void EurocDriver::stop()
{
  if (!is_running_) {
    return;
  }

  if (timer_) {
    timer_->cancel();
  }
  is_running_ = false;
  RCLCPP_INFO(node_->get_logger(), "EuRoC driver stopped");
}

void EurocDriver::shutdown()
{
  stop();
  reader_.reset();
  depth_engine_.reset();
  node_.reset();
}

bool EurocDriver::has_capability(sensor_interfaces::SensorCapability cap) const
{
  switch (cap) {
    case sensor_interfaces::SensorCapability::STEREO_LEFT:
    case sensor_interfaces::SensorCapability::STEREO_RIGHT:
    case sensor_interfaces::SensorCapability::IMU:
      return true;
    case sensor_interfaces::SensorCapability::DEPTH_IMAGE:
      return publish_depth_;
    case sensor_interfaces::SensorCapability::COLOR_IMAGE:
    case sensor_interfaces::SensorCapability::LIDAR_POINTS:
      return false;
    default:
      return false;
  }
}

std::vector<sensor_interfaces::SensorCapability> EurocDriver::get_capabilities() const
{
  std::vector<sensor_interfaces::SensorCapability> caps = {
    sensor_interfaces::SensorCapability::STEREO_LEFT,
    sensor_interfaces::SensorCapability::STEREO_RIGHT,
    sensor_interfaces::SensorCapability::IMU
  };
  
  if (publish_depth_) {
    caps.push_back(sensor_interfaces::SensorCapability::DEPTH_IMAGE);
  }
  
  return caps;
}

sensor_interfaces::HealthStatus EurocDriver::get_health() const
{
  sensor_interfaces::HealthStatus health;
  health.is_healthy = is_running_ && reader_ != nullptr;
  health.health_score = health.is_healthy ? 1.0f : 0.0f;
  health.status_message = is_running_ ? "running" : "stopped";
  
  std::lock_guard<std::mutex> lock(timestamp_mutex_);
  health.last_timestamp = last_timestamp_;
  
  return health;
}

std::chrono::nanoseconds EurocDriver::get_last_timestamp() const
{
  std::lock_guard<std::mutex> lock(timestamp_mutex_);
  return last_timestamp_;
}

std::vector<std::string> EurocDriver::get_published_topics() const
{
  return {
    "/stereo/left/image_raw",
    "/stereo/right/image_raw",
    "/imu/data",
    "/camera/color/camera_info"
  };
}

void EurocDriver::timer_callback()
{
  if (next_frame_idx_ >= reader_->num_frames()) {
    if (loop_) {
      RCLCPP_INFO(node_->get_logger(), "Looping dataset");
      reset_playback();
    } else {
      timer_->cancel();
      RCLCPP_INFO(node_->get_logger(), "Dataset replay complete");
      is_running_ = false;
      return;
    }
  }

  // Compute elapsed dataset time from wall clock
  auto wall_now = node_->now();
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

  // Publish all IMU samples up to target time
  publish_imu_up_to(target_ns);

  // Publish camera frame if its timestamp has been reached
  while (next_frame_idx_ < reader_->num_frames() &&
    reader_->image_timestamp_ns(next_frame_idx_) <= target_ns)
  {
    publish_frame(next_frame_idx_);
    next_frame_idx_++;
  }
}

void EurocDriver::publish_imu_up_to(int64_t target_ns)
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

void EurocDriver::publish_frame(size_t frame_idx)
{
  auto ts_ns = reader_->image_timestamp_ns(frame_idx);
  auto stamp = to_ros_time(ts_ns);

  // Update last timestamp
  {
    std::lock_guard<std::mutex> lock(timestamp_mutex_);
    last_timestamp_ = std::chrono::nanoseconds(ts_ns);
  }

  // Load and publish left image
  cv::Mat left = reader_->load_image(0, frame_idx);
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = frame_id_;

  auto left_msg = cv_bridge::CvImage(header, "mono8", left).toImageMsg();
  pub_left_->publish(*left_msg);

  // Load and publish right image
  cv::Mat right = reader_->load_image(1, frame_idx);
  auto right_msg = cv_bridge::CvImage(header, "mono8", right).toImageMsg();
  pub_right_->publish(*right_msg);

  // Publish CameraInfo with matching timestamp
  cam_info_msg_.header.stamp = stamp;
  pub_cam_info_->publish(cam_info_msg_);

  // Compute and publish depth if enabled
  if (publish_depth_ && depth_engine_) {
    cv::Mat depth = depth_engine_->compute(left, right);
    auto depth_msg = cv_bridge::CvImage(header, "32FC1", depth).toImageMsg();
    pub_depth_->publish(*depth_msg);
  }
}

rclcpp::Time EurocDriver::to_ros_time(int64_t timestamp_ns) const
{
  return rclcpp::Time(timestamp_ns, RCL_ROS_TIME);
}

void EurocDriver::reset_playback()
{
  wall_start_ = node_->now();
  dataset_start_ns_ = reader_->image_timestamp_ns(0);
  next_frame_idx_ = 0;
  next_imu_idx_ = 0;
}

}  // namespace euroc_provider

// Required for pluginlib
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(euroc_provider::EurocDriver, sensor_interfaces::SensorDriver)
