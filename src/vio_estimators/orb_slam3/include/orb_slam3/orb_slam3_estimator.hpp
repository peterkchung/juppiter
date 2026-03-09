// About: ORB-SLAM3 Estimator Implementation
// Stereo-inertial VIO for juppiter dev tier

#ifndef ORB_SLAM3__ORB_SLAM3_ESTIMATOR_HPP_
#define ORB_SLAM3__ORB_SLAM3_ESTIMATOR_HPP_

#include <memory>
#include <string>
#include <mutex>
#include <queue>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "vio_estimator_interfaces/vio_estimator.hpp"

// Eigen
#include <Eigen/Dense>

// OpenCV
#include <opencv2/opencv.hpp>

namespace juppiter
{
namespace vio
{

// Forward declarations - stubbed for now
// class System;
// class Atlas;
// class Tracking;
class LocalMapping;
class LoopClosing;

/**
 * @brief ORB-SLAM3 estimator for juppiter dev tier
 * 
 * Implements stereo-inertial visual odometry with:
 * - Multi-map system (Atlas) for robust tracking
 * - IMU integration for accurate scale and pose
 * - Loop closing for long-term consistency
 * - Visual-inertial bundle adjustment
 * 
 * Reference: https://github.com/UZ-SLAMLab/ORB_SLAM3
 */
class OrbSlam3Estimator : public VioEstimator
{
public:
  OrbSlam3Estimator();
  ~OrbSlam3Estimator() override;

  // VioEstimator interface
  bool initialize(
    rclcpp::Node::SharedPtr node,
    const std::string & config_path,
    const sensor_msgs::msg::CameraInfo & camera_info_left,
    const sensor_msgs::msg::CameraInfo & camera_info_right) override;

  void processStereo(
    const sensor_msgs::msg::Image::SharedPtr left,
    const sensor_msgs::msg::Image::SharedPtr right) override;

  void processMonocular(
    const sensor_msgs::msg::Image::SharedPtr image) override;

  void processImu(
    const sensor_msgs::msg::Imu::SharedPtr imu) override;

  nav_msgs::msg::Odometry::SharedPtr getOdometry() override;

  VioHealthStatus getHealthStatus() const override;

  void reset() override;

  std::string getName() const override;

  bool isTracking() const override;

  float getComputeLoad() const override;

  void setFrameRate(float rate_hz) override;

  void setEnabled(bool enabled) override;

private:
  rclcpp::Node::SharedPtr node_;
  bool initialized_{false};
  bool enabled_{true};
  bool tracking_{false};

  // Camera configuration
  sensor_msgs::msg::CameraInfo left_cam_info_;
  sensor_msgs::msg::CameraInfo right_cam_info_;
  
  // State
  struct {
    Eigen::Vector3d position{0.0, 0.0, 0.0};
    Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};
    Eigen::Vector3d velocity{0.0, 0.0, 0.0};
    rclcpp::Time last_update;
    bool valid{false};
  } state_;

  // Health metrics
  mutable std::mutex health_mutex_;
  struct {
    int tracked_features{0};
    float reprojection_error{0.0f};
    float illumination_estimate{0.0f};
    int tracking_quality{0};  // 0: LOST, 1: RECENT, 2: GOOD
    int lost_count{0};
    int frames_since_init{0};
  } metrics_;

  // ORB-SLAM3 system - stubbed out for now
  // std::unique_ptr<System> slam_system_;
  
  // IMU buffer (ORB-SLAM3 processes IMU in batches)
  std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;
  std::mutex imu_mutex_;

  // Frame rate control
  float target_fps_{20.0f};
  rclcpp::Time last_frame_time_;

  // Last odometry
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  mutable std::mutex odom_mutex_;

  // Private methods
  void loadConfiguration(const std::string & config_path);
  void processFrame(const cv::Mat & left, const cv::Mat & right, double timestamp);
  void processImuBatch();
  float computeHealthScore() const;
  float computeCovarianceNorm() const;
  void publishOdometry();
  
  // Image preprocessing
  cv::Mat preprocessImage(const cv::Mat & image);
};

}  // namespace vio
}  // namespace juppiter

#endif  // ORB_SLAM3__ORB_SLAM3_ESTIMATOR_HPP_
