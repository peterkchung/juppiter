// About: OpenVINS Estimator Implementation
// MSCKF-based VIO for juppiter edge tier

#include "openvins/openvins_estimator.hpp"

#include <cv_bridge/cv_bridge.hpp>

namespace juppiter
{
namespace vio
{

OpenVinsEstimator::OpenVinsEstimator()
: target_fps_(20.0f)
{
  last_odom_ = std::make_shared<nav_msgs::msg::Odometry>();
}

OpenVinsEstimator::~OpenVinsEstimator() = default;

bool OpenVinsEstimator::initialize(
  rclcpp::Node::SharedPtr node,
  const std::string & config_path,
  const sensor_msgs::msg::CameraInfo & camera_info_left,
  const sensor_msgs::msg::CameraInfo & camera_info_right)
{
  node_ = node;
  left_cam_info_ = camera_info_left;
  right_cam_info_ = camera_info_right;
  
  RCLCPP_INFO(node_->get_logger(), 
    "Initializing OpenVINS estimator (edge tier)");
  
  // Load configuration
  loadConfiguration(config_path);
  
  // TODO: Initialize OpenVINS state server, updater, and tracker
  // - Load calibration parameters
  // - Initialize IMU noise parameters
  // - Setup feature tracker (KLT)
  // - Initialize MSCKF state
  
  RCLCPP_INFO(node_->get_logger(), 
    "OpenVINS initialized with %d max features", config_.max_features);
  
  initialized_ = true;
  return true;
}

void OpenVinsEstimator::processStereo(
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
    return;
  }
  last_frame_time_ = now;
  
  // Convert to OpenCV
  cv::Mat left_cv, right_cv;
  try {
    left_cv = cv_bridge::toCvShare(left, "mono8")->image;
    right_cv = cv_bridge::toCvShare(right, "mono8")->image;
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  
  // Preprocess
  left_cv = preprocessImage(left_cv);
  right_cv = preprocessImage(right_cv);
  
  // Estimate illumination
  cv::Scalar mean_val = cv::mean(left_cv);
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    metrics_.illumination_estimate = static_cast<float>(mean_val[0]);
  }
  
  // Process in OpenVINS
  processFrame(left_cv, right_cv, 
    rclcpp::Time(left->header.stamp).seconds());
  
  // Update health
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    state_.frame_count++;
    
    if (state_.valid) {
      metrics_.frames_tracked++;
      tracking_ = true;
    } else {
      metrics_.frames_lost++;
      tracking_ = false;
    }
  }
  
  publishOdometry();
}

void OpenVinsEstimator::processMonocular(
  const sensor_msgs::msg::Image::SharedPtr image)
{
  // OpenVINS supports stereo for juppiter
  RCLCPP_WARN(node_->get_logger(), 
    "OpenVINS configured for stereo, monocular not fully supported");
}

void OpenVinsEstimator::processImu(
  const sensor_msgs::msg::Imu::SharedPtr imu)
{
  if (!initialized_ || !enabled_) {
    return;
  }
  
  // Predict state with IMU (MSCKF propagation step)
  predictWithImu(imu);
}

nav_msgs::msg::Odometry::SharedPtr OpenVinsEstimator::getOdometry()
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  return last_odom_;
}

VioHealthStatus OpenVinsEstimator::getHealthStatus() const
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
  if (metrics_.tracked_features < config_.min_features) {
    health.active_faults.push_back("LOW_FEATURE_COUNT");
    health.is_healthy = false;
  }
  
  if (metrics_.illumination_estimate < 25.0f) {
    health.active_faults.push_back("LOW_ILLUMINATION");
  }
  
  if (!tracking_ && metrics_.frames_lost > 10) {
    health.active_faults.push_back("TRACKING_LOST");
    health.is_healthy = false;
  }
  
  health.last_update = node_->now();
  return health;
}

void OpenVinsEstimator::reset()
{
  RCLCPP_INFO(node_->get_logger(), "Resetting OpenVINS estimator");
  
  std::lock_guard<std::mutex> lock_odom(odom_mutex_);
  std::lock_guard<std::mutex> lock_health(health_mutex_);
  
  state_.position.setZero();
  state_.orientation.setIdentity();
  state_.velocity.setZero();
  state_.valid = false;
  state_.frame_count = 0;
  
  tracking_ = false;
  metrics_ = {};
  
  // TODO: Reset OpenVINS state
}

std::string OpenVinsEstimator::getName() const
{
  return "openvins";
}

bool OpenVinsEstimator::isTracking() const
{
  return tracking_;
}

float OpenVinsEstimator::getComputeLoad() const
{
  // OpenVINS is lighter than ORB-SLAM3
  // ~20-30% of a core (vs 35-50% for ORB-SLAM3)
  std::lock_guard<std::mutex> lock(health_mutex_);
  
  float base_load = 0.25f;
  float feature_factor = std::min(1.3f, 
    metrics_.tracked_features / 150.0f);
  
  return base_load * feature_factor;
}

void OpenVinsEstimator::setFrameRate(float rate_hz)
{
  target_fps_ = std::max(5.0f, std::min(30.0f, rate_hz));
  RCLCPP_INFO(node_->get_logger(), 
    "OpenVINS target FPS set to %.1f", target_fps_);
}

void OpenVinsEstimator::setEnabled(bool enabled)
{
  enabled_ = enabled;
  RCLCPP_INFO(node_->get_logger(), 
    "OpenVINS %s", enabled ? "enabled" : "disabled");
}

// Private methods

void OpenVinsEstimator::loadConfiguration(const std::string & config_path)
{
  // TODO: Load OpenVINS parameters from YAML
  // For now, use edge-optimized defaults
  
  config_.max_features = 150;      // Less than ORB-SLAM3's 1200
  config_.min_features = 50;      // Minimum for tracking
  config_.track_frequency = 20.0; // 20Hz
  config_.use_klt = true;          // Fast KLT tracking
  config_.window_size = 10;       // MSCKF window
  
  RCLCPP_INFO(node_->get_logger(), 
    "OpenVINS config loaded: %d max features, %.1f Hz", 
    config_.max_features, config_.track_frequency);
}

void OpenVinsEstimator::processFrame(
  const cv::Mat & left, 
  const cv::Mat & right, 
  double timestamp)
{
  // TODO: Implement OpenVINS MSCKF processing
  // 1. Track features with KLT
  // 2. Triangulate new features
  // 3. MSCKF update with feature constraints
  // 4. Marginalize old states
  
  // For now, simulate motion
  static float drift = 0.0f;
  drift += 0.001f;
  
  state_.position.x() += 0.01f * std::cos(drift);
  state_.position.y() += 0.01f * std::sin(drift);
  
  // Update orientation
  Eigen::AngleAxisd rotation(0.005, Eigen::Vector3d::UnitZ());
  state_.orientation = state_.orientation * Eigen::Quaterniond(rotation);
  
  state_.valid = true;
  state_.last_update = node_->now();
  
  // Simulate tracked features
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    metrics_.tracked_features = 100 + static_cast<int>(50 * std::sin(drift));
  }
}

void OpenVinsEstimator::predictWithImu(
  const sensor_msgs::msg::Imu::SharedPtr & imu)
{
  // TODO: Implement IMU propagation for MSCKF
  // This predicts the state between camera frames
  // OpenVINS uses continuous-time integration
  
  // Simplified for now
  state_.velocity.x() += imu->linear_acceleration.x * 0.005;
  state_.velocity.y() += imu->linear_acceleration.y * 0.005;
  state_.velocity.z() += imu->linear_acceleration.z * 0.005;
}

float OpenVinsEstimator::computeHealthScore() const
{
  float score = 1.0f;
  
  // Lost tracking penalty
  if (!tracking_) {
    score -= 0.4f;
  }
  
  // Low features penalty
  if (metrics_.tracked_features < config_.min_features) {
    score -= 0.35f;
  } else if (metrics_.tracked_features < config_.max_features * 0.5) {
    score -= 0.15f;
  }
  
  // Illumination penalty
  if (metrics_.illumination_estimate < 20.0f) {
    score -= 0.15f;
  }
  
  return std::max(0.0f, score);
}

float OpenVinsEstimator::computeCovarianceNorm() const
{
  // MSCKF covariance
  float base_covariance = 0.008f;  // 0.8cm (slightly worse than ORB-SLAM3)
  
  // Scale with features
  if (metrics_.tracked_features < config_.min_features) {
    base_covariance *= 5.0f;
  } else if (metrics_.tracked_features < config_.max_features * 0.5) {
    base_covariance *= 2.0f;
  }
  
  // Scale with tracking status
  if (!tracking_) {
    base_covariance *= 3.0f;
  }
  
  return base_covariance;
}

void OpenVinsEstimator::publishOdometry()
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

cv::Mat OpenVinsEstimator::preprocessImage(const cv::Mat & image)
{
  cv::Mat processed;
  
  // Ensure grayscale
  if (image.channels() == 3) {
    cv::cvtColor(image, processed, cv::COLOR_BGR2GRAY);
  } else {
    processed = image.clone();
  }
  
  // Optional CLAHE for low-light
  // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
  // clahe->apply(processed, processed);
  
  return processed;
}

}  // namespace vio
}  // namespace juppiter
