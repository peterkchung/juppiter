// About: LIO Estimator Interface
// Defines the contract for all LiDAR-Inertial Odometry implementations
// Supports: FAST-LIO2, DLIO, SR-LIO++, LIO-SAM

#ifndef LIO_ESTIMATOR_INTERFACES__LIO_ESTIMATOR_HPP_
#define LIO_ESTIMATOR_INTERFACES__LIO_ESTIMATOR_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace juppiter
{
namespace lio
{

/**
 * @brief Health status for LIO estimator
 * Used by FDIIR framework for mode switching decisions
 */
struct LioHealthStatus
{
  bool is_healthy{true};
  float health_score{1.0f};           // 0.0 to 1.0
  float covariance_norm{0.0f};         // Position uncertainty metric
  float point_cloud_density{0.0f};     // Points per cubic meter
  float registration_residual{0.0f};   // ICP/registration quality
  bool sync_violation{false};          // Time sync issues
  std::vector<std::string> active_faults;
  rclcpp::Time last_update;
};

/**
 * @brief Abstract interface for LiDAR-Inertial Odometry estimators
 * 
 * All LIO implementations (FAST-LIO2, DLIO, SR-LIO++, etc.) must implement
 * this interface to be compatible with the juppiter fusion core.
 */
class LioEstimator
{
public:
  virtual ~LioEstimator() = default;

  /**
   * @brief Initialize the estimator with ROS node and configuration
   * @param node Shared pointer to ROS node for logging and parameter access
   * @param config_path Path to YAML configuration file
   * @return true if initialization successful
   */
  virtual bool initialize(
    rclcpp::Node::SharedPtr node,
    const std::string & config_path) = 0;

  /**
   * @brief Process incoming point cloud data
   * @param point_cloud LiDAR scan data
   */
  virtual void processPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) = 0;

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
  virtual LioHealthStatus getHealthStatus() const = 0;

  /**
   * @brief Reset estimator to initial state
   * Useful for recovery after fault clearance
   */
  virtual void reset() = 0;

  /**
   * @brief Get implementation name
   * @return String identifier (e.g., "fast_lio2", "dlio")
   */
  virtual std::string getName() const = 0;

  /**
   * @brief Check if estimator has valid map/odometry output
   * @return true if ready to contribute to fusion
   */
  virtual bool isReady() const = 0;

  /**
   * @brief Get computational load metric (0.0 to 1.0)
   * For dynamic resource management in edge deployment
   * @return CPU load estimate
   */
  virtual float getComputeLoad() const { return 0.0f; }

  /**
   * @brief Set dynamic downsampling factor for resource adaptation
   * @param factor Downsample factor (>1.0 reduces computation)
   */
  virtual void setDownsampleFactor(float factor) {}

protected:
  rclcpp::Node::SharedPtr node_;
  bool initialized_{false};
};

}  // namespace lio
}  // namespace juppiter

#endif  // LIO_ESTIMATOR_INTERFACES__LIO_ESTIMATOR_HPP_
