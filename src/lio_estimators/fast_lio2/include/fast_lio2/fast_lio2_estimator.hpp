// About: FAST-LIO2 Estimator Implementation
// Wraps FAST-LIO2 algorithm for juppiter integration

#ifndef FAST_LIO2__FAST_LIO2_ESTIMATOR_HPP_
#define FAST_LIO2__FAST_LIO2_ESTIMATOR_HPP_

#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "lio_estimator_interfaces/lio_estimator.hpp"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace juppiter
{
namespace lio
{

// Forward declarations for FAST-LIO2 internals
struct IkdTreeManager;
struct ImuProcessor;
struct LidarProcessor;
struct State;

/**
 * @brief FAST-LIO2 estimator implementation for juppiter
 * 
 * Implements the LioEstimator interface wrapping FAST-LIO2 algorithm.
 * FAST-LIO2 uses ikd-Tree for incremental mapping and tightly-coupled
 * iterated Kalman filter for LiDAR-inertial fusion.
 * 
 * Features:
 * - Direct point cloud odometry (no feature extraction)
 * - ikd-Tree for fast nearest neighbor search
 * - IMU integration for motion prediction
 * - Supports multiple LiDAR types (Velodyne, Ouster, Livox)
 * 
 * Reference: https://github.com/hku-mars/FAST_LIO
 */
class FastLio2Estimator : public LioEstimator
{
public:
  FastLio2Estimator();
  ~FastLio2Estimator() override;

  // LioEstimator interface implementation
  bool initialize(
    rclcpp::Node::SharedPtr node,
    const std::string & config_path) override;

  void processPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) override;

  void processImu(
    const sensor_msgs::msg::Imu::SharedPtr imu) override;

  nav_msgs::msg::Odometry::SharedPtr getOdometry() override;

  LioHealthStatus getHealthStatus() const override;

  void reset() override;

  std::string getName() const override;

  bool isReady() const override;

  float getComputeLoad() const override;

  void setDownsampleFactor(float factor) override;

private:
  // ROS node
  rclcpp::Node::SharedPtr node_;
  bool initialized_{false};

  // Configuration
  std::string config_path_;
  
  // State
  struct {
    Eigen::Vector3d position{0.0, 0.0, 0.0};
    Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};
    Eigen::Vector3d velocity{0.0, 0.0, 0.0};
    Eigen::Vector3d angular_velocity{0.0, 0.0, 0.0};
    rclcpp::Time last_update;
    bool valid{false};
  } state_;

  // Health monitoring
  mutable std::mutex health_mutex_;
  struct {
    float point_cloud_density{0.0f};
    float registration_residual{0.0f};
    int points_last_frame{0};
    float avg_processing_time_ms{0.0f};
    int frame_count{0};
  } metrics_;

  // Downsample factor for resource adaptation
  float downsample_factor_{1.0f};

  // Last odometry output
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  mutable std::mutex odom_mutex_;

  // Private methods
  void initializeFastLio2();
  void processPointCloudInternal(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud);
  void processImuInternal(const sensor_msgs::msg::Imu::SharedPtr & imu);
  float computeHealthScore() const;
  float computeCovarianceNorm() const;
  void publishOdometry();
};

}  // namespace lio
}  // namespace juppiter

#endif  // FAST_LIO2__FAST_LIO2_ESTIMATOR_HPP_
