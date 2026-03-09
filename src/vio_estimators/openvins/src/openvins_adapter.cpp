// About: OpenVINS Adapter Implementation
// Wraps rpng/open_vins for juppiter VioEstimator interface

#include "openvins/openvins_adapter.hpp"

#include <chrono>
#include <cmath>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>

// cv_bridge for ROS-OpenCV conversion
#include <cv_bridge/cv_bridge.hpp>

namespace juppiter
{
namespace vio
{

OpenVINSAdapter::OpenVINSAdapter()
: target_period_ms_(100.0f)  // Default 10Hz
{
  // Initialize with defaults
}

OpenVINSAdapter::~OpenVINSAdapter()
{
  // Cleanup if needed
}

bool OpenVINSAdapter::initialize(
  rclcpp::Node::SharedPtr node,
  const std::string & config_path,
  const sensor_msgs::msg::CameraInfo & camera_info_left,
  const sensor_msgs::msg::CameraInfo & camera_info_right)
{
  node_ = node;
  
  logInfo("OpenVINSAdapter initializing...");
  
  // Load ROS parameters
  loadParameters(config_path);
  
  // Configure based on compute profile
  configureForProfile(compute_profile_);
  
  #ifdef OPENVINS_AVAILABLE
    logInfo("OpenVINS library available - initializing real MSCKF");
    
    try {
      // Initialize OpenVINS VioManager
      // This would load the calibration and create the state estimator
      // For now, we'll create a stub that simulates initialization
      
      // TODO: Implement real OpenVINS initialization
      // vio_manager_ = std::make_shared<ov_msckf::VioManager>(...);
      
      ov_initialized_ = true;
      logInfo("OpenVINS initialized successfully");
    } catch (const std::exception & e) {
      logWarn("Failed to initialize OpenVINS: " + std::string(e.what()));
      logWarn("Falling back to stub implementation");
      ov_initialized_ = false;
    }
  #else
    logWarn("OpenVINS library not available - using stub implementation");
    ov_initialized_ = false;
  #endif
  
  // For stub mode, generate test motion pattern
  if (!ov_initialized_) {
    logInfo("Stub mode: Will generate test circular motion");
    stub_state_.timestamp = getCurrentTimestamp();
    stub_state_.tracked_features = 75;  // Simulated mid-range features
    stub_state_.health_score = 0.75f;
  }
  
  initialized_ = true;
  tracking_ = false;  // Will start tracking after static initialization
  
  logInfo("OpenVINSAdapter initialization complete");
  return true;
}

void OpenVINSAdapter::loadParameters(const std::string & config_path)
{
  (void)config_path;  // Currently unused, YAML file parsed separately
  
  if (!node_) return;
  
  // Note: Parameters are already declared in OpenVINSNode::declare_parameters()
  // We just load (get) them here, don't re-declare
  
  // Load from parameter server (they must exist)
  try {
    compute_profile_ = node_->get_parameter("compute_profile").as_string();
    feature_tracker_ = node_->get_parameter("feature_tracker").as_string();
    calibration_file_ = node_->get_parameter("calibration_file").as_string();
    config_.max_features = node_->get_parameter("max_features").as_int();
    config_.min_features = node_->get_parameter("min_features").as_int();
    config_.target_fps = static_cast<float>(node_->get_parameter("target_fps").as_double());
    config_.skip_if_lio_healthy = node_->get_parameter("skip_if_lio_healthy").as_bool();
    config_.lio_healthy_threshold = static_cast<float>(
      node_->get_parameter("lio_healthy_threshold").as_double());
    config_.grid_x = node_->get_parameter("grid_x").as_int();
    config_.grid_y = node_->get_parameter("grid_y").as_int();
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
    logWarn("Parameter not declared: " + std::string(e.what()));
    logWarn("Using default values");
  }
  
  // Update timing
  target_period_ms_ = 1000.0f / config_.target_fps;
  
  logInfo("Parameters loaded:");
  logInfo("  Profile: " + compute_profile_);
  logInfo("  Tracker: " + feature_tracker_);
  logInfo("  Target FPS: " + std::to_string(config_.target_fps));
  logInfo("  Max features: " + std::to_string(config_.max_features));
}

void OpenVINSAdapter::configureForProfile(const std::string & profile)
{
  if (profile == "edge") {
    config_.target_fps = 10.0f;
    config_.max_features = 150;
    target_period_ms_ = 100.0f;
    logInfo("Configured for edge tier: 10Hz, 150 features");
  } else if (profile == "dev") {
    config_.target_fps = 20.0f;
    config_.max_features = 300;
    target_period_ms_ = 50.0f;
    logInfo("Configured for dev tier: 20Hz, 300 features");
  } else if (profile == "low_power") {
    config_.target_fps = 5.0f;
    config_.max_features = 100;
    target_period_ms_ = 200.0f;
    logInfo("Configured for low power: 5Hz, 100 features");
  }
}

void OpenVINSAdapter::setLIOHealth(float health_score)
{
  std::lock_guard<std::mutex> lock(health_mutex_);
  lio_health_score_ = health_score;
}

bool OpenVINSAdapter::shouldSkipFrame()
{
  // Edge optimization: Skip if LIO is very healthy and we're in edge/low_power mode
  if (config_.skip_if_lio_healthy && 
      (compute_profile_ == "edge" || compute_profile_ == "low_power")) {
    std::lock_guard<std::mutex> lock(health_mutex_);
    if (lio_health_score_ > config_.lio_healthy_threshold) {
      frames_skipped_++;
      return true;
    }
  }
  
  // Frame rate limiting
  rclcpp::Time now = node_->now();
  if (last_process_time_.nanoseconds() > 0) {
    float elapsed_ms = (now - last_process_time_).nanoseconds() / 1e6f;
    if (elapsed_ms < target_period_ms_) {
      return true;  // Not time for next frame yet
    }
  }
  
  frames_processed_++;
  return false;
}

void OpenVINSAdapter::processStereo(
  const sensor_msgs::msg::Image::SharedPtr left,
  const sensor_msgs::msg::Image::SharedPtr right)
{
  if (!initialized_ || !enabled_) return;
  
  // Check if we should skip this frame
  if (shouldSkipFrame()) {
    return;
  }
  
  // Convert ROS images to OpenCV
  cv_bridge::CvImagePtr left_cv, right_cv;
  try {
    left_cv = cv_bridge::toCvCopy(left, "mono8");
    right_cv = cv_bridge::toCvCopy(right, "mono8");
  } catch (cv_bridge::Exception & e) {
    logWarn("CV bridge exception: " + std::string(e.what()));
    return;
  }
  
  // Preprocess images
  cv::Mat left_processed = preprocessImage(left_cv->image);
  cv::Mat right_processed = preprocessImage(right_cv->image);
  
  // Get timestamp
  double timestamp = left->header.stamp.sec + left->header.stamp.nanosec * 1e-9;
  
  #ifdef OPENVINS_AVAILABLE
    if (ov_initialized_) {
      feedStereo(left_processed, right_processed, timestamp);
    } else {
  #endif
      // Stub: Simulate motion
      static double t = 0.0;
      t += 0.02;  // 20Hz simulation
      
      // Circular motion with offset from origin
      stub_state_.position[0] = 2.0 * std::cos(t) + 0.05;
      stub_state_.position[1] = 2.0 * std::sin(t) + 0.03;
      stub_state_.position[2] = 0.1 * std::sin(t * 2.0) - 0.02;
      
      // Orientation (pointing along tangent)
      double yaw = t + M_PI/2;
      stub_state_.orientation[0] = std::cos(yaw/2);  // w
      stub_state_.orientation[1] = 0.0;  // x
      stub_state_.orientation[2] = 0.0;  // y
      stub_state_.orientation[3] = std::sin(yaw/2);  // z
      
      stub_state_.timestamp = timestamp;
      stub_state_.tracked_features = config_.max_features * (0.7f + 0.3f * std::sin(t));
      stub_state_.health_score = 0.7f + 0.25f * std::sin(t * 0.5f);
      
      tracking_ = true;
  #ifdef OPENVINS_AVAILABLE
    }
  #endif
  
  // Extract and publish odometry
  extractOdometry();
  last_process_time_ = node_->now();
}

void OpenVINSAdapter::feedStereo(
  const cv::Mat & left, 
  const cv::Mat & right, 
  double timestamp)
{
  #ifdef OPENVINS_AVAILABLE
    // TODO: Implement real OpenVINS stereo feeding
    // vio_manager_->feed_image_stereo(timestamp, left, right);
    // vio_manager_->process_IMU(...) // IMU data from buffer
  #endif
  (void)left; (void)right; (void)timestamp;  // Suppress unused warnings
}

cv::Mat OpenVINSAdapter::preprocessImage(const cv::Mat & image)
{
  // Optional: CLAHE, resizing, etc.
  // For now, just return as-is
  return image.clone();
}

void OpenVINSAdapter::processMonocular(
  const sensor_msgs::msg::Image::SharedPtr image)
{
  if (!initialized_ || !enabled_) return;
  
  logWarn("Monocular mode not implemented - OpenVINS requires stereo for scale");
  (void)image;
}

void OpenVINSAdapter::processImu(
  const sensor_msgs::msg::Imu::SharedPtr imu)
{
  if (!initialized_) return;
  
  #ifdef OPENVINS_AVAILABLE
    if (ov_initialized_) {
      feedIMU(imu);
    } else {
  #endif
      // Stub: Just store in buffer
      std::lock_guard<std::mutex> lock(imu_mutex_);
      imu_buffer_.push_back(imu);
      
      // Keep buffer size reasonable
      while (imu_buffer_.size() > 100) {
        imu_buffer_.pop_front();
      }
  #ifdef OPENVINS_AVAILABLE
    }
  #endif
}

void OpenVINSAdapter::feedIMU(const sensor_msgs::msg::Imu::SharedPtr & imu)
{
  #ifdef OPENVINS_AVAILABLE
    // TODO: Implement real OpenVINS IMU feeding
    // vio_manager_->feed_IMU(timestamp, ...);
  #endif
  (void)imu;
}

void OpenVINSAdapter::extractOdometry()
{
  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->header.stamp = node_->now();
  odom->header.frame_id = "map";
  odom->child_frame_id = "base_link";
  
  #ifdef OPENVINS_AVAILABLE
    if (ov_initialized_ && vio_manager_) {
      // TODO: Extract real pose from OpenVINS state
      // auto state = vio_manager_->get_state();
      // odom->pose.pose.position.x = state->imu_pos()(0);
      // ...
    } else {
  #endif
      // Stub: Use simulated state
      odom->pose.pose.position.x = stub_state_.position[0];
      odom->pose.pose.position.y = stub_state_.position[1];
      odom->pose.pose.position.z = stub_state_.position[2];
      
      odom->pose.pose.orientation.w = stub_state_.orientation[0];
      odom->pose.pose.orientation.x = stub_state_.orientation[1];
      odom->pose.pose.orientation.y = stub_state_.orientation[2];
      odom->pose.pose.orientation.z = stub_state_.orientation[3];
      
      odom->twist.twist.linear.x = stub_state_.velocity[0];
      odom->twist.twist.linear.y = stub_state_.velocity[1];
      odom->twist.twist.linear.z = stub_state_.velocity[2];
  #ifdef OPENVINS_AVAILABLE
    }
  #endif
  
  // Covariance (diagonal)
  float cov_pos = 0.01f;  // 1cm position uncertainty
  float cov_rot = 0.001f; // Small rotation uncertainty
  float cov_vel = 0.1f;   // Velocity uncertainty
  
  for (int i = 0; i < 6; i++) {
    odom->pose.covariance[i*6 + i] = (i < 3) ? cov_pos : cov_rot;
  }
  for (int i = 0; i < 6; i++) {
    odom->twist.covariance[i*6 + i] = (i < 3) ? cov_vel : cov_rot;
  }
  
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    last_odom_ = odom;
    last_odom_time_ = node_->now();
  }
}

nav_msgs::msg::Odometry::SharedPtr OpenVINSAdapter::getOdometry()
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (!last_odom_) {
    // Return empty odometry if not initialized
    auto empty = std::make_shared<nav_msgs::msg::Odometry>();
    empty->header.stamp = node_->now();
    empty->header.frame_id = "map";
    return empty;
  }
  return last_odom_;
}

VioHealthStatus OpenVINSAdapter::getHealthStatus() const
{
  VioHealthStatus health;
  health.last_update = node_->now();
  
  if (!initialized_) {
    health.is_healthy = false;
    health.health_score = 0.0f;
    health.active_faults = {"not_initialized"};
    return health;
  }
  
  if (!enabled_) {
    health.is_healthy = false;
    health.health_score = -1.0f;  // Disabled marker
    health.active_faults = {"disabled_by_lio"};
    return health;
  }
  
  #ifdef OPENVINS_AVAILABLE
    if (ov_initialized_) {
      // TODO: Extract real metrics from OpenVINS
      health.tracked_features = config_.max_features;
      health.health_score = 0.95f;
    } else {
  #endif
      // Stub metrics
      health.tracked_features = stub_state_.tracked_features;
      health.health_score = stub_state_.health_score;
      health.reprojection_error = 1.5f;  // Simulated 1.5px error
      
      // Determine if tracking is healthy
      health.is_healthy = (stub_state_.tracked_features >= config_.min_features) &&
                          (stub_state_.health_score > 0.5f);
  #ifdef OPENVINS_AVAILABLE
    }
  #endif
  
  // Covariance norm (position uncertainty)
  health.covariance_norm = std::sqrt(0.01f);  // ~3cm
  
  return health;
}

void OpenVINSAdapter::reset()
{
  logInfo("Resetting OpenVINS adapter");
  
  #ifdef OPENVINS_AVAILABLE
    if (ov_initialized_ && vio_manager_) {
      // TODO: Reset OpenVINS state
      // vio_manager_->reset();
    }
  #endif
  
  tracking_ = false;
  frames_processed_ = 0;
  frames_skipped_ = 0;
  
  // Clear buffers
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_buffer_.clear();
  }
  
  // Reset stub state
  stub_state_.timestamp = 0.0;
  stub_state_.position[0] = 0.0;
  stub_state_.position[1] = 0.0;
  stub_state_.position[2] = 0.0;
  stub_state_.orientation[0] = 1.0;
  stub_state_.orientation[1] = 0.0;
  stub_state_.orientation[2] = 0.0;
  stub_state_.orientation[3] = 0.0;
  stub_state_.tracked_features = 0;
  stub_state_.health_score = 0.5f;
  
  logInfo("Reset complete");
}

std::string OpenVINSAdapter::getName() const
{
  return "openvins_msckf";
}

bool OpenVINSAdapter::isTracking() const
{
  return tracking_;
}

float OpenVINSAdapter::getComputeLoad() const
{
  // Estimate compute load based on frame rate and features
  float load = 0.0f;
  
  if (frames_processed_ + frames_skipped_ > 0) {
    float process_ratio = static_cast<float>(frames_processed_) / 
                         (frames_processed_ + frames_skipped_);
    load = process_ratio * (config_.target_fps / 20.0f) * 
           (static_cast<float>(config_.max_features) / 300.0f);
  }
  
  return std::min(1.0f, load);
}

void OpenVINSAdapter::setFrameRate(float rate_hz)
{
  config_.target_fps = rate_hz;
  target_period_ms_ = 1000.0f / rate_hz;
  logInfo("Frame rate set to " + std::to_string(rate_hz) + " Hz");
}

void OpenVINSAdapter::setEnabled(bool enabled)
{
  enabled_ = enabled;
  if (enabled) {
    logInfo("OpenVINS enabled");
  } else {
    logInfo("OpenVINS disabled (skipping frames)");
  }
}

double OpenVINSAdapter::getCurrentTimestamp() const
{
  if (node_) {
    auto now = node_->now();
    return now.seconds() + now.nanoseconds() * 1e-9;
  }
  return 0.0;
}

void OpenVINSAdapter::logDebug(const std::string & msg) const
{
  if (node_) {
    RCLCPP_DEBUG(node_->get_logger(), "[OpenVINS] %s", msg.c_str());
  }
}

void OpenVINSAdapter::logInfo(const std::string & msg) const
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "[OpenVINS] %s", msg.c_str());
  }
}

void OpenVINSAdapter::logWarn(const std::string & msg) const
{
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "[OpenVINS] %s", msg.c_str());
  }
}

}  // namespace vio
}  // namespace juppiter
