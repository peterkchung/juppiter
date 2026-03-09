// About: DLIO Estimator Stub
// Placeholder implementation for CI testing

#include "dlio/dlio_estimator.hpp"

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
  loadConfig(config_path);
  RCLCPP_INFO(node_->get_logger(), "DLIO initialized (stub)");
  initialized_ = true;
  return true;
}

void DlioEstimator::processPointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud)
{
  if (!initialized_) return;
  last_odom_->header = point_cloud->header;
  state_.valid = true;
}

void DlioEstimator::processImu(
  const sensor_msgs::msg::Imu::SharedPtr imu)
{
  if (!initialized_) return;
}

nav_msgs::msg::Odometry::SharedPtr DlioEstimator::getOdometry()
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  return last_odom_;
}

LioHealthStatus DlioEstimator::getHealthStatus() const
{
  LioHealthStatus health;
  health.is_healthy = true;
  health.health_score = 0.9f;
  health.covariance_norm = 0.02f;
  health.point_cloud_density = 800.0f;
  health.registration_residual = 0.1f;
  health.sync_violation = false;
  health.last_update = node_ ? node_->now() : rclcpp::Time(0);
  return health;
}

void DlioEstimator::reset()
{
  state_ = {};
  metrics_ = {};
  submap_->clear();
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
  return 0.25f;
}

void DlioEstimator::setDownsampleFactor(float factor)
{
  downsample_factor_ = std::max(1.0f, factor);
}

// Stub implementations
void DlioEstimator::loadConfig(const std::string & config_path) {}
pcl::PointCloud<pcl::PointXYZI>::Ptr DlioEstimator::downsample(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud) { return cloud; }
void DlioEstimator::updateSubmap(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud) {}
void DlioEstimator::computeOdometry(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud) {}
bool DlioEstimator::isKeyframe() const { return false; }
float DlioEstimator::computeHealthScore() const { return 0.9f; }
float DlioEstimator::computeCovarianceNorm() const { return 0.02f; }

}  // namespace lio
}  // namespace juppiter
