// About: OpenVINS Estimator Implementation
// MSCKF-based VIO for juppiter edge tier

#ifndef OPENVINS__OPENVINS_ESTIMATOR_HPP_
#define OPENVINS__OPENVINS_ESTIMATOR_HPP_

#include <memory>
#include <string>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "vio_estimator_interfaces/vio_estimator.hpp"

#include <opencv2/opencv.hpp>

namespace juppiter
{
namespace vio
{

// Forward declarations for OpenVINS internals
struct State;
struct Updater;
struct Tracker;

/**
 * @brief OpenVINS estimator for juppiter edge tier
 * 
 * Implements Multi-State Constraint Kalman Filter (MSCKF) VIO:
 * - Lightweight compared to ORB-SLAM3
 * - Native ROS 2 support
 * - No loop closure (pure odometry)
 * - Lower memory footprint
 * 
 * Reference: https://github.com/rpng/open_vins
 */
class OpenVinsEstimator : public VioEstimator
{
public:
  OpenVinsEstimator();
  ~OpenVinsEstimator() override;

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
  
  // Configuration
  struct {
    int max_features{150};        // Fewer than ORB-SLAM3
    int min_features{50};         // Minimum to maintain tracking
    float track_frequency{20.0};  // Target FPS
    bool use_klt{true};           // Use KLT tracking (fast)
    int window_size{10};          // MSCKF window size
    float max_aruco_distance{5.0}; // Disable for outdoor
  } config_;

  // State
  struct {
    Eigen::Vector3d position{0.0, 0.0, 0.0};
    Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};
    Eigen::Vector3d velocity{0.0, 0.0, 0.0};
    rclcpp::Time last_update;
    bool valid{false};
    int frame_count{0};
  } state_;

  // Health metrics
  mutable std::mutex health_mutex_;
  struct {
    int tracked_features{0};
    float reprojection_error{0.0f};
    float illumination_estimate{0.0f};
    int frames_lost{0};
    int frames_tracked{0};
  } metrics_;

  // OpenVINS components
  std::unique_ptr<State> state_server_;
  std::unique_ptr<Updater> updater_;
  std::unique_ptr<Tracker> tracker_;

  // Frame rate control
  float target_fps_{20.0f};
  rclcpp::Time last_frame_time_;

  // Last odometry
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  mutable std::mutex odom_mutex_;

  // Private methods
  void loadConfiguration(const std::string & config_path);
  void processFrame(const cv::Mat & left, const cv::Mat & right, double timestamp);
  void predictWithImu(const sensor_msgs::msg::Imu::SharedPtr & imu);
  float computeHealthScore() const;
  float computeCovarianceNorm() const;
  void publishOdometry();
  cv::Mat preprocessImage(const cv::Mat & image);
};

}  // namespace vio
}  // namespace juppiter

#endif  // OPENVINS__OPENVINS_ESTIMATOR_HPP_
