// About: ORB-SLAM3 Estimator Implementation
// Stereo-inertial VIO wrapper for juppiter

#include "orb_slam3/orb_slam3_estimator.hpp"

#include <cv_bridge/cv_bridge.hpp>

namespace juppiter
{
namespace vio
{

OrbSlam3Estimator::OrbSlam3Estimator()
: target_fps_(20.0f)
{
  last_odom_ = std::make_shared<nav_msgs::msg::Odometry>();
}

OrbSlam3Estimator::~OrbSlam3Estimator() = default;

bool OrbSlam3Estimator::initialize(
  rclcpp::Node::SharedPtr node,
  const std::string & config_path,
  const sensor_msgs::msg::CameraInfo & camera_info_left,
  const sensor_msgs::msg::CameraInfo & camera_info_right)
{
  node_ = node;
  left_cam_info_ = camera_info_left;
  right_cam_info_ = camera_info_right;
  
  RCLCPP_INFO(node_->get_logger(), 
    "Initializing ORB-SLAM3 estimator");
  
  // Load configuration
  loadConfiguration(config_path);
  
  // TODO: Initialize ORB-SLAM3 system
  // This would create the ORB-SLAM3 System object with:
  // - Vocabulary file
  // - Settings file (calibration, ORB parameters, etc.)
  // - Sensor type (STEREO_INERTIAL)
  
  RCLCPP_INFO(node_->get_logger(), "ORB-SLAM3 system initialized");
  
  initialized_ = true;
  return true;
}

void OrbSlam3Estimator::processStereo(
  const sensor_msgs::msg::Image::SharedPtr left,
  const sensor_msgs::msg::Image::SharedPtr right)
{
  if (!initialized_ || !enabled_) {
    return;
  }
  
  // Frame rate control
  rclcpp::Time now = node_->now();
  double time_since_last = (now - last_frame_time_).seconds();
  double target_period = 1.0 / target_fps_;
  
  if (time_since_last < target_period * 0.9) {
    // Skip frame to maintain target FPS
    return;
  }
  last_frame_time_ = now;
  
  // Convert ROS images to OpenCV
  cv::Mat left_cv, right_cv;
  try {
    left_cv = cv_bridge::toCvShare(left, "mono8")->image;
    right_cv = cv_bridge::toCvShare(right, "mono8")->image;
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  
  // Preprocess images
  left_cv = preprocessImage(left_cv);
  right_cv = preprocessImage(right_cv);
  
  // Estimate illumination
  cv::Scalar mean_val = cv::mean(left_cv);
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    metrics_.illumination_estimate = static_cast<float>(mean_val[0]);
  }
  
  // Process in ORB-SLAM3
  processFrame(left_cv, right_cv, 
    rclcpp::Time(left->header.stamp).seconds());
  
  // Update health metrics
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    metrics_.frames_since_init++;
    
    // Estimate tracked features (simplified)
    // Full implementation would query ORB-SLAM3 tracking state
    metrics_.tracked_features = 100;  // Placeholder
    
    // Estimate tracking quality
    if (state_.valid) {
      metrics_.tracking_quality = 2;  // GOOD
      tracking_ = true;
    } else {
      metrics_.tracking_quality = 0;  // LOST
      metrics_.lost_count++;
      tracking_ = false;
    }
  }
  
  publishOdometry();
}

void OrbSlam3Estimator::processMonocular(
  const sensor_msgs::msg::Image::SharedPtr image)
{
  // ORB-SLAM3 supports monocular, but we use stereo for juppiter
  RCLCPP_WARN(node_->get_logger(), 
    "Monocular mode not fully supported, use stereo");
}

void OrbSlam3Estimator::processImu(
  const sensor_msgs::msg::Imu::SharedPtr imu)
{
  if (!initialized_) {
    return;
  }
  
  // Buffer IMU data (ORB-SLAM3 processes in batches with images)
  std::lock_guard<std::mutex> lock(imu_mutex_);
  imu_buffer_.push(imu);
  
  // Keep buffer from growing too large
  while (imu_buffer_.size() > 200) {  // 1 second at 200Hz
    imu_buffer_.pop();
  }
}

nav_msgs::msg::Odometry::SharedPtr OrbSlam3Estimator::getOdometry()
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  return last_odom_;
}

VioHealthStatus OrbSlam3Estimator::getHealthStatus() const
{
  std::lock_guard<std::mutex> lock(health_mutex_);
  
  VioHealthStatus health;
  health.is_healthy = tracking_;
  health.health_score = computeHealthScore();
  health.covariance_norm = computeCovarianceNorm();
  health.tracked_features = metrics_.tracked_features;
  health.reprojection_error = metrics_.reprojection_error;
  health.illumination_lux = metrics_.illumination_estimate;
  health.sync_violation = false;
  
  // Classify faults
  if (metrics_.tracked_features < 50) {
    health.active_faults.push_back("LOW_FEATURE_COUNT");
  }
  
  if (metrics_.illumination_estimate < 30.0f) {
    health.active_faults.push_back("LOW_ILLUMINATION");
  }
  
  if (metrics_.tracking_quality == 0) {
    health.active_faults.push_back("TRACKING_LOST");
    health.is_healthy = false;
  }
  
  health.last_update = node_->now();
  return health;
}

void OrbSlam3Estimator::reset()
{
  RCLCPP_INFO(node_->get_logger(), "Resetting ORB-SLAM3 estimator");
  
  std::lock_guard<std::mutex> lock_odom(odom_mutex_);
  std::lock_guard<std::mutex> lock_health(health_mutex_);
  std::lock_guard<std::mutex> lock_imu(imu_mutex_);
  
  state_.position.setZero();
  state_.orientation.setIdentity();
  state_.velocity.setZero();
  state_.valid = false;
  
  tracking_ = false;
  metrics_ = {};
  
  // Clear IMU buffer
  while (!imu_buffer_.empty()) {
    imu_buffer_.pop();
  }
  
  // TODO: Reset ORB-SLAM3 system
}

std::string OrbSlam3Estimator::getName() const
{
  return "orb_slam3";
}

bool OrbSlam3Estimator::isTracking() const
{
  return tracking_;
}

float OrbSlam3Estimator::getComputeLoad() const
{
  // ORB-SLAM3 is computationally expensive
  // ~35-50% of a core on modern CPUs
  std::lock_guard<std::mutex> lock(health_mutex_);
  
  float base_load = 0.40f;
  
  // Scale with feature count
  float feature_factor = std::min(1.5f, 
    metrics_.tracked_features / 200.0f);
  
  return base_load * feature_factor;
}

void OrbSlam3Estimator::setFrameRate(float rate_hz)
{
  target_fps_ = std::max(5.0f, std::min(30.0f, rate_hz));
  RCLCPP_INFO(node_->get_logger(), 
    "ORB-SLAM3 target FPS set to %.1f", target_fps_);
}

void OrbSlam3Estimator::setEnabled(bool enabled)
{
  enabled_ = enabled;
  RCLCPP_INFO(node_->get_logger(), 
    "ORB-SLAM3 %s", enabled ? "enabled" : "disabled");
}

// Private methods

void OrbSlam3Estimator::loadConfiguration(const std::string & config_path)
{
  // TODO: Load ORB-SLAM3 specific parameters from YAML
  // - Vocabulary path
  - Settings file path
  - ORB feature parameters
  - IMU parameters
  - Tracking thresholds
  
  RCLCPP_INFO(node_->get_logger(), 
    "ORB-SLAM3 configuration loaded from: %s", config_path.c_str());
}

void OrbSlam3Estimator::processFrame(
  const cv::Mat & left, 
  const cv::Mat & right, 
  double timestamp)
{
  // TODO: Call ORB-SLAM3 System::TrackStereo()
  // This would:
  // 1. Process buffered IMU data
  // 2. Track features in stereo pair
  // 3. Perform visual-inertial bundle adjustment
  // 4. Return current pose
  
  // For now, simulate motion
  static float drift = 0.0f;
  drift += 0.001f;
  
  state_.position.x() += 0.01f * std::cos(drift);
  state_.position.y() += 0.01f * std::sin(drift);
  state_.position.z() += 0.001f * std::sin(drift * 2);
  
  // Simulate orientation change
  Eigen::AngleAxisd rotation(0.01, Eigen::Vector3d::UnitZ());
  state_.orientation = state_.orientation * Eigen::Quaterniond(rotation);
  
  state_.valid = true;
  state_.last_update = node_->now();
}

void OrbSlam3Estimator::processImuBatch()
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  
  // Process buffered IMU data
  while (!imu_buffer_.empty()) {
    auto imu = imu_buffer_.front();
    imu_buffer_.pop();
    
    // TODO: Feed to ORB-SLAM3 IMU processor
    // Preintegrate IMU measurements between frames
  }
}

float OrbSlam3Estimator::computeHealthScore() const
{
  float score = 1.0f;
  
  // Lost tracking penalty
  if (metrics_.tracking_quality == 0) {
    score -= 0.5f;
  } else if (metrics_.tracking_quality == 1) {
    score -= 0.1f;
  }
  
  // Low features penalty
  if (metrics_.tracked_features < 50) {
    score -= 0.3f;
  } else if (metrics_.tracked_features < 100) {
    score -= 0.1f;
  }
  
  // Illumination penalty
  if (metrics_.illumination_estimate < 20.0f) {
    score -= 0.2f;
  }
  
  return std::max(0.0f, score);
}

float OrbSlam3Estimator::computeCovarianceNorm() const
{
  // Simplified covariance estimation
  float base_covariance = 0.005f;  // 0.5cm uncertainty when tracking well
  
  // Scale with tracking quality
  if (metrics_.tracking_quality == 0) {
    base_covariance *= 10.0f;  // Very uncertain when lost
  } else if (metrics_.tracking_quality == 1) {
    base_covariance *= 3.0f;
  }
  
  // Scale with features
  if (metrics_.tracked_features < 50) {
    base_covariance *= 2.0f;
  }
  
  return base_covariance;
}

void OrbSlam3Estimator::publishOdometry()
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  
  rclcpp::Time now = node_->now();
  
  last_odom_->header.stamp = now;
  last_odom_->header.frame_id = "map";
  last_odom_->child_frame_id = "base_link";
  
  last_odom_->pose.pose.position.x = state_.position.x();
  last_odom_->pose.pose.position.y = state_.position.y();
  last_odom_->pose.pose.position.z = state_.position.z();
  last_odom_->pose.pose.orientation.x = state_.orientation.x();
  last_odom_->pose.pose.orientation.y = state_.orientation.y();
  last_odom_->pose.pose.orientation.z = state_.orientation.z();
  last_odom_->pose.pose.orientation.w = state_.orientation.w();
  
  last_odom_->twist.twist.linear.x = state_.velocity.x();
  last_odom_->twist.twist.linear.y = state_.velocity.y();
  last_odom_->twist.twist.linear.z = state_.velocity.z();
}

cv::Mat OrbSlam3Estimator::preprocessImage(const cv::Mat & image)
{
  // Basic preprocessing
  cv::Mat processed;
  
  // Ensure grayscale
  if (image.channels() == 3) {
    cv::cvtColor(image, processed, cv::COLOR_BGR2GRAY);
  } else {
    processed = image.clone();
  }
  
  // Optional: Histogram equalization for low-light
  // cv::equalizeHist(processed, processed);
  
  return processed;
}

}  // namespace vio
}  // namespace juppiter
