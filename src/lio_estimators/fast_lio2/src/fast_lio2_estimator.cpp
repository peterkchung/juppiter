// About: FAST-LIO2 Estimator Stub
// Placeholder implementation for CI testing

#include "fast_lio2/fast_lio2_estimator.hpp"

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
  RCLCPP_INFO(node_->get_logger(), "FAST-LIO2 initialized (stub)");
  initialized_ = true;
  return true;
}

void FastLio2Estimator::processPointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud)
{
  if (!initialized_) return;
  // Stub: just update timestamp
  last_odom_->header = point_cloud->header;
}

void FastLio2Estimator::processImu(
  const sensor_msgs::msg::Imu::SharedPtr imu)
{
  if (!initialized_) return;
  // Stub implementation
}

nav_msgs::msg::Odometry::SharedPtr FastLio2Estimator::getOdometry()
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  return last_odom_;
}

LioHealthStatus FastLio2Estimator::getHealthStatus() const
{
  LioHealthStatus health;
  health.is_healthy = true;
  health.health_score = 0.9f;
  health.covariance_norm = 0.01f;
  health.point_cloud_density = 1000.0f;
  health.registration_residual = 0.05f;
  health.sync_violation = false;
  health.last_update = node_ ? node_->now() : rclcpp::Time(0);
  return health;
}

void FastLio2Estimator::reset()
{
  state_ = {};
  metrics_ = {};
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
  return 0.35f;
}

void FastLio2Estimator::setDownsampleFactor(float factor)
{
  downsample_factor_ = std::max(1.0f, factor);
}

// Stub implementations for helper classes
void FastLio2Estimator::initializeFastLio2() {}
float FastLio2Estimator::computeHealthScore() const { return 0.9f; }
float FastLio2Estimator::computeCovarianceNorm() const { return 0.01f; }
void FastLio2Estimator::publishOdometry() {}

}  // namespace lio
}  // namespace juppiter
