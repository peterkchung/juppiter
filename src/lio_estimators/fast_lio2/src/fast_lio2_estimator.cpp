// About: FAST-LIO2 Estimator Implementation
// Wraps FAST-LIO2 algorithm for juppiter integration

#include "fast_lio2/fast_lio2_estimator.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

namespace juppiter
{
namespace lio
{

FastLio2Estimator::FastLio2Estimator()
: downsample_factor_(1.0f)
{
  last_odom_ = std::make_shared<nav_msgs::msg::Odometry>();
}

FastLio2Estimator::~FastLio2Estimator() = default;

bool FastLio2Estimator::initialize(
  rclcpp::Node::SharedPtr node,
  const std::string & config_path)
{
  node_ = node;
  config_path_ = config_path;
  
  RCLCPP_INFO(node_->get_logger(), 
    "Initializing FAST-LIO2 estimator with config: %s", 
    config_path.c_str());
  
  // TODO: Load FAST-LIO2 configuration from YAML
  // For now, use sensible defaults
  
  // Initialize internal FAST-LIO2 state
  initializeFastLio2();
  
  initialized_ = true;
  RCLCPP_INFO(node_->get_logger(), "FAST-LIO2 estimator initialized successfully");
  
  return true;
}

void FastLio2Estimator::processPointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud)
{
  if (!initialized_) {
    RCLCPP_WARN(node_->get_logger(), 
      "FAST-LIO2 not initialized, dropping point cloud");
    return;
  }
  
  // Convert ROS message to PCL
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*point_cloud, *cloud);
  
  // Apply downsampling if configured
  if (downsample_factor_ > 1.0f) {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    float leaf_size = 0.1f * downsample_factor_;  // Default 10cm, scaled by factor
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*cloud);
  }
  
  // Update metrics
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    metrics_.points_last_frame = static_cast<int>(cloud->size());
    metrics_.frame_count++;
  }
  
  // Process in FAST-LIO2
  processPointCloudInternal(cloud);
  
  // Publish odometry
  publishOdometry();
}

void FastLio2Estimator::processImu(
  const sensor_msgs::msg::Imu::SharedPtr imu)
{
  if (!initialized_) {
    return;
  }
  
  processImuInternal(imu);
}

nav_msgs::msg::Odometry::SharedPtr FastLio2Estimator::getOdometry()
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  return last_odom_;
}

LioHealthStatus FastLio2Estimator::getHealthStatus() const
{
  std::lock_guard<std::mutex> lock(health_mutex_);
  
  LioHealthStatus health;
  health.is_healthy = true;
  health.health_score = computeHealthScore();
  health.covariance_norm = computeCovarianceNorm();
  health.point_cloud_density = metrics_.points_last_frame > 0 ? 
    static_cast<float>(metrics_.points_last_frame) / 1000.0f : 0.0f;
  health.registration_residual = metrics_.registration_residual;
  health.sync_violation = false;  // TODO: Implement sync checking
  
  // Classify faults
  if (metrics_.points_last_frame < 1000) {
    health.active_faults.push_back("LOW_POINT_DENSITY");
    health.is_healthy = false;
  }
  
  health.last_update = node_->now();
  return health;
}

void FastLio2Estimator::reset()
{
  RCLCPP_INFO(node_->get_logger(), "Resetting FAST-LIO2 estimator");
  
  std::lock_guard<std::mutex> lock_odom(odom_mutex_);
  std::lock_guard<std::mutex> lock_health(health_mutex_);
  
  state_.position.setZero();
  state_.orientation.setIdentity();
  state_.velocity.setZero();
  state_.valid = false;
  
  metrics_ = {};
  
  // Re-initialize FAST-LIO2
  initializeFastLio2();
}

std::string FastLio2Estimator::getName() const
{
  return "fast_lio2";
}

bool FastLio2Estimator::isReady() const
{
  return initialized_ && state_.valid;
}

float FastLio2Estimator::getComputeLoad() const
{
  // Estimate compute load based on recent processing times
  std::lock_guard<std::mutex> lock(health_mutex_);
  
  // FAST-LIO2 typically uses 30-40% of a core on modern CPUs
  // Scale based on point cloud size
  float base_load = 0.35f;
  float scale_factor = std::min(2.0f, metrics_.points_last_frame / 50000.0f);
  
  return base_load * scale_factor;
}

void FastLio2Estimator::setDownsampleFactor(float factor)
{
  downsample_factor_ = std::max(1.0f, factor);
  RCLCPP_INFO(node_->get_logger(), 
    "FAST-LIO2 downsample factor set to %.2f", downsample_factor_);
}

// Private methods

void FastLio2Estimator::initializeFastLio2()
{
  // TODO: Initialize FAST-LIO2 internal state
  // This would set up:
  // - ikd-Tree with specified parameters
  // - IMU initialization (bias estimation)
  // - State covariance
  // - Filter parameters
  
  RCLCPP_INFO(node_->get_logger(), "FAST-LIO2 internal state initialized");
}

void FastLio2Estimator::processPointCloudInternal(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud)
{
  // TODO: Implement actual FAST-LIO2 point cloud processing
  // This is the core algorithm that would:
  // 1. Undistort point cloud using IMU data
  // 2. Perform ICP/registration against ikd-Tree map
  // 3. Update state with iterated Kalman filter
  // 4. Add points to ikd-Tree map
  
  // For now, just store a placeholder state
  rclcpp::Time now = node_->now();
  
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    
    state_.last_update = now;
    state_.valid = true;
    
    // Update odometry message
    last_odom_->header.stamp = now;
    last_odom_->header.frame_id = "map";
    last_odom_->child_frame_id = "base_link";
    
    // Set pose (placeholder - would come from FAST-LIO2 state)
    last_odom_->pose.pose.position.x = state_.position.x();
    last_odom_->pose.pose.position.y = state_.position.y();
    last_odom_->pose.pose.position.z = state_.position.z();
    last_odom_->pose.pose.orientation.x = state_.orientation.x();
    last_odom_->pose.pose.orientation.y = state_.orientation.y();
    last_odom_->pose.pose.orientation.z = state_.orientation.z();
    last_odom_->pose.pose.orientation.w = state_.orientation.w();
    
    // Set twist
    last_odom_->twist.twist.linear.x = state_.velocity.x();
    last_odom_->twist.twist.linear.y = state_.velocity.y();
    last_odom_->twist.twist.linear.z = state_.velocity.z();
    last_odom_->twist.twist.angular.x = state_.angular_velocity.x();
    last_odom_->twist.twist.angular.y = state_.angular_velocity.y();
    last_odom_->twist.twist.angular.z = state_.angular_velocity.z();
  }
  
  // Simulate some drift for testing
  static float drift = 0.0f;
  drift += 0.001f;
  state_.position.x() += 0.01f;
  state_.position.y() += 0.001f * std::sin(drift);
}

void FastLio2Estimator::processImuInternal(
  const sensor_msgs::msg::Imu::SharedPtr & imu)
{
  // TODO: Implement IMU integration
  // This would:
  // 1. Compensate for IMU bias
  // 2. Integrate to get motion prediction
  // 3. Use for point cloud undistortion
  
  state_.angular_velocity.x() = imu->angular_velocity.x;
  state_.angular_velocity.y() = imu->angular_velocity.y;
  state_.angular_velocity.z() = imu->angular_velocity.z;
}

float FastLio2Estimator::computeHealthScore() const
{
  // Health score based on:
  // - Point cloud density (need minimum for good registration)
  // - Registration quality (residuals)
  // - State covariance magnitude
  
  float score = 1.0f;
  
  // Penalty for low point density
  if (metrics_.points_last_frame < 5000) {
    score -= 0.3f;
  } else if (metrics_.points_last_frame < 10000) {
    score -= 0.1f;
  }
  
  // Penalty for high residuals
  if (metrics_.registration_residual > 0.1f) {
    score -= 0.2f;
  }
  
  return std::max(0.0f, score);
}

float FastLio2Estimator::computeCovarianceNorm() const
{
  // Simplified covariance calculation
  // In real implementation, would extract from Kalman filter covariance matrix
  
  float base_covariance = 0.01f;  // 1cm position uncertainty
  
  // Scale with point density
  if (metrics_.points_last_frame < 5000) {
    base_covariance *= 3.0f;
  }
  
  // Scale with residuals
  base_covariance *= (1.0f + metrics_.registration_residual * 10.0f);
  
  return base_covariance;
}

void FastLio2Estimator::publishOdometry()
{
  // TODO: Create ROS publisher and publish odometry
  // For now, just store in last_odom_
}

}  // namespace lio
}  // namespace juppiter
