# About: VIO Estimator Interface
# Defines the contract for all Visual-Inertial Odometry implementations
# Supports: ORB-SLAM3, OpenVINS, VINS-Fusion, LEVIO, UL-VIO

#ifndef VIO_ESTIMATOR_INTERFACES__VIO_ESTIMATOR_HPP_
#define VIO_ESTIMATOR_INTERFACES__VIO_ESTIMATOR_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace juppiter
{
namespace vio
{

/**
 * @brief Health status for VIO estimator
 * Used by FDIIR framework for mode switching decisions
 */
struct VioHealthStatus
{
  bool is_healthy{true};
  float health_score{1.0f};           // 0.0 to 1.0
  float covariance_norm{0.0f};         // Position uncertainty metric
  int tracked_features{0};             // Number of active visual features
  float reprojection_error{0.0f};    // Mean reprojection error (pixels)
  float illumination_lux{0.0f};        // Estimated scene illumination
  bool sync_violation{false};          // Time sync issues
  std::vector<std::string> active_faults;
  rclcpp::Time last_update;
};

/**
 * @brief Abstract interface for Visual-Inertial Odometry estimators
 * 
 * All VIO implementations (ORB-SLAM3, OpenVINS, VINS-Fusion, etc.) must implement
 * this interface to be compatible with the juppiter fusion core.
 */
class VioEstimator
{
public:
  virtual ~VioEstimator() = default;

  /**
   * @brief Initialize the estimator with ROS node and configuration
   * @param node Shared pointer to ROS node for logging and parameter access
   * @param config_path Path to YAML configuration file
   * @param camera_info_left Left camera calibration
   * @param camera_info_right Right camera calibration (for stereo)
   * @return true if initialization successful
   */
  virtual bool initialize(
    rclcpp::Node::SharedPtr node,
    const std::string & config_path,
    const sensor_msgs::msg::CameraInfo & camera_info_left,
    const sensor_msgs::msg::CameraInfo & camera_info_right) = 0;

  /**
   * @brief Process stereo image pair
   * @param left Left camera image
   * @param right Right camera image
   */
  virtual void processStereo(
    const sensor_msgs::msg::Image::SharedPtr left,
    const sensor_msgs::msg::Image::SharedPtr right) = 0;

  /**
   * @brief Process monocular image (for monocular VIO modes)
   * @param image Camera image
   */
  virtual void processMonocular(
    const sensor_msgs::msg::Image::SharedPtr image) = 0;

  /**
   * @brief Process incoming IMU data
   * @param imu IMU measurement
   */
  virtual void processImu(
    const sensor_msgs::msg::Imu::SharedPtr imu) = 0;

  /**
   * @brief Get current odometry estimate
   * @return Odometry message with pose and twist
   */
  virtual nav_msgs::msg::Odometry::SharedPtr getOdometry() = 0;

  /**
   * @brief Get health status for FDIIR monitoring
   * @return Health status structure
   */
  virtual VioHealthStatus getHealthStatus() const = 0;

  /**
   * @brief Reset estimator to initial state
   * Useful for recovery after fault clearance
   */
  virtual void reset() = 0;

  /**
   * @brief Get implementation name
   * @return String identifier (e.g., "orb_slam3", "openvins")
   */
  virtual std::string getName() const = 0;

  /**
   * @brief Check if estimator has valid tracking
   * @return true if tracking established and ready to contribute to fusion
   */
  virtual bool isTracking() const = 0;

  /**
   * @brief Get computational load metric (0.0 to 1.0)
   * For dynamic resource management in edge deployment
   * @return CPU load estimate
   */
  virtual float getComputeLoad() const { return 0.0f; }

  /**
   * @brief Set frame processing rate for resource adaptation
   * @param rate_hz Target frame rate (may reduce accuracy)
   */
  virtual void setFrameRate(float rate_hz) {}

  /**
   * @brief Enable/disable tracking (for selective updates)
   * @param enabled Whether to process frames
   */
  virtual void setEnabled(bool enabled) {}

protected:
  rclcpp::Node::SharedPtr node_;
  bool initialized_{false};
  bool tracking_{false};
};

}  // namespace vio
}  // namespace juppiter

#endif  // VIO_ESTIMATOR_INTERFACES__VIO_ESTIMATOR_HPP_
