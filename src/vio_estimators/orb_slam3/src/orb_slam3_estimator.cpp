// About: ORB-SLAM3 Estimator Stub
// Placeholder implementation for CI testing

#include "orb_slam3/orb_slam3_estimator.hpp"

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
  
  RCLCPP_INFO(node_->get_logger(), "ORB-SLAM3 initialized (stub)");
  initialized_ = true;
  return true;
}

void OrbSlam3Estimator::processStereo(
  const sensor_msgs::msg::Image::SharedPtr left,
  const sensor_msgs::msg::Image::SharedPtr right)
{
  if (!initialized_) return;
  // Stub: just update timestamp
  last_odom_->header = left->header;
  tracking_ = true;
}

void OrbSlam3Estimator::processMonocular(
  const sensor_msgs::msg::Image::SharedPtr image)
{
  if (!initialized_) return;
}

void OrbSlam3Estimator::processImu(
  const sensor_msgs::msg::Imu::SharedPtr imu)
{
  if (!initialized_) return;
}

nav_msgs::msg::Odometry::SharedPtr OrbSlam3Estimator::getOdometry()
{
  return last_odom_;
}

VioHealthStatus OrbSlam3Estimator::getHealthStatus() const
{
  VioHealthStatus health;
  health.is_healthy = tracking_;
  health.health_score = 0.9f;
  health.covariance_norm = 0.01f;
  health.tracked_features = 100;
  health.reprojection_error = 1.0f;
  health.illumination_lux = 100.0f;
  health.sync_violation = false;
  health.last_update = node_ ? node_->now() : rclcpp::Time(0);
  return health;
}

void OrbSlam3Estimator::reset()
{
  tracking_ = false;
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
  return 0.4f;
}

void OrbSlam3Estimator::setFrameRate(float rate_hz)
{
  target_fps_ = std::max(5.0f, std::min(30.0f, rate_hz));
}

void OrbSlam3Estimator::setEnabled(bool enabled)
{
  enabled_ = enabled;
}

}  // namespace vio
}  // namespace juppiter
