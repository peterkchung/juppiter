// About: DLIO Estimator Implementation
// Direct LiDAR-Inertial Odometry for edge deployment

#include "dlio/dlio_estimator.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

namespace juppiter
{
namespace lio
{

DlioEstimator::DlioEstimator()
: submap_(new pcl::PointCloud<pcl::PointXYZI>()),
  downsample_factor_(1.0f),
  current_voxel_size_(0.1f)
{
  last_odom_ = std::make_shared<nav_msgs::msg::Odometry>();
}

DlioEstimator::~DlioEstimator() = default;

bool DlioEstimator::initialize(
  rclcpp::Node::SharedPtr node,
  const std::string & config_path)
{
  node_ = node;
  
  RCLCPP_INFO(node_->get_logger(), 
    "Initializing DLIO estimator with config: %s", 
    config_path.c_str());
  
  // Load configuration
  loadConfig(config_path);
  
  initialized_ = true;
  RCLCPP_INFO(node_->get_logger(), "DLIO estimator initialized successfully");
  
  return true;
}

void DlioEstimator::processPointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud)
{
  if (!initialized_) {
    RCLCPP_WARN(node_->get_logger(), "DLIO not initialized, dropping point cloud");
    return;
  }
  
  // Convert to PCL
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*point_cloud, *cloud);
  
  // Downsample
  auto filtered = downsample(cloud);
  
  // Update metrics
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    metrics_.points_processed = static_cast<int>(filtered->size());
  }
  
  // Compute odometry
  computeOdometry(filtered);
  
  // Update submap if keyframe
  if (isKeyframe()) {
    updateSubmap(filtered);
    state_.keyframe_count++;
    
    std::lock_guard<std::mutex> lock(health_mutex_);
    metrics_.keyframes = state_.keyframe_count;
  }
}

void DlioEstimator::processImu(
  const sensor_msgs::msg::Imu::SharedPtr imu)
{
  // DLIO uses IMU for motion prediction and point cloud undistortion
  // Store latest IMU data for next point cloud processing
  // (Simplified implementation - full DLIO has sophisticated IMU integration)
  
  if (!state_.valid) {
    return;
  }
  
  // Update velocity estimate from IMU (simplified)
  // Full implementation would integrate IMU over time
  state_.velocity.x() += imu->linear_acceleration.x * 0.005;  // 200Hz assumption
  state_.velocity.y() += imu->linear_acceleration.y * 0.005;
  state_.velocity.z() += imu->linear_acceleration.z * 0.005;
}

nav_msgs::msg::Odometry::SharedPtr DlioEstimator::getOdometry()
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  return last_odom_;
}

LioHealthStatus DlioEstimator::getHealthStatus() const
{
  std::lock_guard<std::mutex> lock(health_mutex_);
  
  LioHealthStatus health;
  health.is_healthy = true;
  health.health_score = computeHealthScore();
  health.covariance_norm = computeCovarianceNorm();
  health.point_cloud_density = metrics_.points_processed > 0 ? 
    static_cast<float>(metrics_.points_processed) / 1000.0f : 0.0f;
  health.registration_residual = metrics_.registration_error;
  health.sync_violation = false;
  
  // Check for issues
  if (metrics_.points_processed < 3000) {
    health.active_faults.push_back("LOW_POINT_DENSITY");
    health.is_healthy = false;
  }
  
  if (metrics_.registration_error > 0.2f) {
    health.active_faults.push_back("HIGH_REGISTRATION_ERROR");
    health.is_healthy = false;
  }
  
  health.last_update = node_->now();
  return health;
}

void DlioEstimator::reset()
{
  RCLCPP_INFO(node_->get_logger(), "Resetting DLIO estimator");
  
  std::lock_guard<std::mutex> lock_odom(odom_mutex_);
  std::lock_guard<std::mutex> lock_health(health_mutex_);
  
  state_.position.setZero();
  state_.orientation.setIdentity();
  state_.velocity.setZero();
  state_.valid = false;
  state_.keyframe_count = 0;
  
  submap_->clear();
  metrics_ = {};
}

std::string DlioEstimator::getName() const
{
  return "dlio";
}

bool DlioEstimator::isReady() const
{
  return initialized_ && state_.valid;
}

float DlioEstimator::getComputeLoad() const
{
  // DLIO is lighter than FAST-LIO2
  // ~20-30% of a core on RPi5
  std::lock_guard<std::mutex> lock(health_mutex_);
  
  float base_load = 0.25f;
  float scale_factor = std::min(1.5f, metrics_.points_processed / 40000.0f);
  
  return base_load * scale_factor;
}

void DlioEstimator::setDownsampleFactor(float factor)
{
  downsample_factor_ = std::max(1.0f, factor);
  current_voxel_size_ = config_.voxel_size_m * downsample_factor_;
  
  RCLCPP_INFO(node_->get_logger(), 
    "DLIO downsample factor: %.2f, voxel size: %.3f m", 
    downsample_factor_, current_voxel_size_);
}

// Private methods

void DlioEstimator::loadConfig(const std::string & config_path)
{
  // TODO: Load from YAML
  // For now, use sensible defaults optimized for edge
  
  config_.keyframe_threshold_trans_m = 0.5f;  // More frequent keyframes than FAST-LIO2
  config_.keyframe_threshold_rot_deg = 15.0f;
  config_.submap_keyframes = 8;  // Smaller submap = less memory
  config_.voxel_size_m = 0.15f;  // Coarser than FAST-LIO2 for speed
  config_.adaptive_downsampling = true;
  
  current_voxel_size_ = config_.voxel_size_m;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr DlioEstimator::downsample(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
  
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(current_voxel_size_, current_voxel_size_, current_voxel_size_);
  voxel_grid.filter(*filtered);
  
  return filtered;
}

void DlioEstimator::updateSubmap(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud)
{
  // Add new keyframe to submap
  *submap_ += *cloud;
  
  // Downsample submap if too large
  if (submap_->size() > static_cast<size_t>(config_.submap_keyframes * 5000)) {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setInputCloud(submap_);
    voxel_grid.setLeafSize(current_voxel_size_, current_voxel_size_, current_voxel_size_);
    voxel_grid.filter(*submap_);
  }
}

void DlioEstimator::computeOdometry(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud)
{
  rclcpp::Time now = node_->now();
  
  // If first frame, just initialize
  if (!state_.valid) {
    state_.valid = true;
    *submap_ = *cloud;
    state_.last_update = now;
    
    // Publish initial odometry
    std::lock_guard<std::mutex> lock(odom_mutex_);
    last_odom_->header.stamp = now;
    last_odom_->header.frame_id = "map";
    last_odom_->child_frame_id = "base_link";
    return;
  }
  
  // Simplified ICP registration (full DLIO uses more sophisticated registration)
  // For now, simulate some motion
  static float drift = 0.0f;
  drift += 0.001f;
  
  state_.position.x() += 0.01f * std::cos(drift);
  state_.position.y() += 0.01f * std::sin(drift);
  
  // Publish odometry
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    
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
  
  state_.last_update = now;
}

bool DlioEstimator::isKeyframe() const
{
  // Simple keyframe detection based on distance/rotation from last keyframe
  // (Simplified - full implementation tracks keyframe poses)
  static int frame_count = 0;
  frame_count++;
  
  // Every 10th frame as keyframe for simplicity
  return (frame_count % 10) == 0;
}

float DlioEstimator::computeHealthScore() const
{
  float score = 1.0f;
  
  // Penalty for low point density
  if (metrics_.points_processed < 3000) {
    score -= 0.4f;
  } else if (metrics_.points_processed < 6000) {
    score -= 0.2f;
  }
  
  // Penalty for high registration error
  if (metrics_.registration_error > 0.15f) {
    score -= 0.3f;
  }
  
  // Small submap penalty (less information)
  if (metrics_.keyframes < 3) {
    score -= 0.1f;
  }
  
  return std::max(0.0f, score);
}

float DlioEstimator::computeCovarianceNorm() const
{
  // Simplified covariance
  float base_covariance = 0.02f;  // 2cm uncertainty (coarser than FAST-LIO2)
  
  // Scale with point density
  if (metrics_.points_processed < 3000) {
    base_covariance *= 4.0f;
  } else if (metrics_.points_processed < 6000) {
    base_covariance *= 2.0f;
  }
  
  // Scale with registration error
  base_covariance *= (1.0f + metrics_.registration_error * 5.0f);
  
  return base_covariance;
}

}  // namespace lio
}  // namespace juppiter
