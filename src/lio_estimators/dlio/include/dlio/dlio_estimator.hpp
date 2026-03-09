# About: DLIO Estimator Implementation
# Lighter LIO alternative for edge deployment

#ifndef DLIO__DLIO_ESTIMATOR_HPP_
#define DLIO__DLIO_ESTIMATOR_HPP_

#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "lio_estimator_interfaces/lio_estimator.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace juppiter
{
namespace lio
{

/**
 * @brief DLIO (Direct LiDAR-Inertial Odometry) estimator
 * 
 * Lightweight alternative to FAST-LIO2 optimized for edge deployment.
 * Features:
 * - Continuous-time trajectory representation
 * - Direct point-to-map registration (no feature extraction)
 * - Lower memory footprint (no global map storage)
 * - Suitable for RPi5 8GB and Jetson Orin Nano
 * 
 * Reference: https://github.com/vectr-ucla/direct_lidar_inertial_odometry
 */
class DlioEstimator : public LioEstimator
{
public:
  DlioEstimator();
  ~DlioEstimator() override;

  // LioEstimator interface
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
  rclcpp::Node::SharedPtr node_;
  bool initialized_{false};

  // Configuration
  struct {
    float keyframe_threshold_trans_m{1.0f};
    float keyframe_threshold_rot_deg{10.0f};
    int submap_keyframes{10};
    float voxel_size_m{0.1f};
    bool adaptive_downsampling{true};
  } config_;

  // State
  struct {
    Eigen::Vector3d position{0.0, 0.0, 0.0};
    Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};
    Eigen::Vector3d velocity{0.0, 0.0, 0.0};
    rclcpp::Time last_update;
    bool valid{false};
    int keyframe_count{0};
  } state_;

  // Health metrics
  mutable std::mutex health_mutex_;
  struct {
    float registration_error{0.0f};
    int points_processed{0};
    int keyframes{0};
    float processing_time_ms{0.0f};
  } metrics_;

  // Local submap (no global map - lighter than FAST-LIO2)
  pcl::PointCloud<pcl::PointXYZI>::Ptr submap_;

  // Downsample factor for resource adaptation
  float downsample_factor_{1.0f};
  float current_voxel_size_{0.1f};

  // Last odometry
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  mutable std::mutex odom_mutex_;

  // Private methods
  void loadConfig(const std::string & config_path);
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsample(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud);
  void updateSubmap(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud);
  void computeOdometry(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud);
  bool isKeyframe() const;
  float computeHealthScore() const;
  float computeCovarianceNorm() const;
};

}  // namespace lio
}  // namespace juppiter

#endif  // DLIO__DLIO_ESTIMATOR_HPP_
