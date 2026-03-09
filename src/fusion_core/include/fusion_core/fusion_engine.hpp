# About: Fusion Engine
# Core confidence-weighted fusion logic for multi-estimator navigation

#ifndef FUSION_CORE__FUSION_ENGINE_HPP_
#define FUSION_CORE__FUSION_ENGINE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "lio_estimator_interfaces/lio_estimator.hpp"
#include "vio_estimator_interfaces/vio_estimator.hpp"

namespace juppiter
{
namespace fusion
{

/**
 * @brief Operating modes for the fusion system
 */
enum class FusionMode
{
  NOMINAL,           // All estimators active, LIO primary
  DEGRADED_LIO,      // LIO degraded, VIO primary
  DEGRADED_VIO,      // VIO degraded, LIO primary
  DEGRADED_KINEMATIC, // Kinematic degraded, LIO/VIO only
  SAFE_STOP          // Only kinematic, initiate stop
};

/**
 * @brief Health-weighted odometry input
 */
struct OdometryInput
{
  nav_msgs::msg::Odometry odometry;
  float health_score{1.0f};
  float covariance_norm{0.0f};
  rclcpp::Time timestamp;
  std::string source;  // "lio", "vio", "kinematic"
};

/**
 * @brief Fused output with uncertainty
 */
struct FusedOdometry
{
  nav_msgs::msg::Odometry odometry;
  float overall_health{1.0f};
  float fused_covariance_norm{0.0f};
  FusionMode mode{FusionMode::NOMINAL};
  std::map<std::string, float> source_weights;
};

/**
 * @brief Core fusion engine implementing confidence-weighted fusion
 * 
 * Implements the fusion strategy:
 * weight_i = health_i / (covariance_norm_i + epsilon)
 * fused_pose = sum(weight_i * pose_i) / sum(weights)
 */
class FusionEngine
{
public:
  FusionEngine();
  ~FusionEngine() = default;

  /**
   * @brief Initialize fusion engine with configuration
   * @param node ROS node for parameter access and logging
   * @param config_path Path to YAML fusion configuration
   * @return true if initialization successful
   */
  bool initialize(
    rclcpp::Node::SharedPtr node,
    const std::string & config_path);

  /**
   * @brief Add LIO odometry input
   * @param lio_health LIO estimator health status
   */
  void updateLioInput(const lio::LioHealthStatus & lio_health);

  /**
   * @brief Add VIO odometry input
   * @param vio_health VIO estimator health status
   */
  void updateVioInput(const vio::VioHealthStatus & vio_health);

  /**
   * @brief Add kinematic odometry input (wheel+IMU EKF)
   * @param odometry Kinematic odometry message
   * @param covariance_norm Uncertainty metric
   */
  void updateKinematicInput(
    const nav_msgs::msg::Odometry::SharedPtr & odometry,
    float covariance_norm);

  /**
   * @brief Compute fused odometry from current inputs
   * @return Fused odometry with mode and health information
   */
  FusedOdometry computeFusion();

  /**
   * @brief Get current operating mode
   * @return Current fusion mode
   */
  FusionMode getCurrentMode() const;

  /**
   * @brief Get mode as string for topic publishing
   * @return Mode string ("nominal", "degraded_lio", etc.)
   */
  std::string getModeString() const;

  /**
   * @brief Reset fusion state (e.g., after mode switch)
   */
  void reset();

  /**
   * @brief Check if fusion can proceed (minimum inputs available)
   * @return true if at least one valid input exists
   */
  bool isFusionReady() const;

  /**
   * @brief Get individual source weights for diagnostics
   * @return Map of source name to weight
   */
  std::map<std::string, float> getSourceWeights() const;

private:
  rclcpp::Node::SharedPtr node_;
  bool initialized_{false};

  // Configuration parameters
  float epsilon_{0.001f};
  float ema_alpha_{0.25f};
  
  // Nominal weights from config
  float nominal_lio_weight_{0.50f};
  float nominal_vio_weight_{0.35f};
  float nominal_kinematic_weight_{0.15f};

  // Mode-specific weight adjustments
  std::map<FusionMode, std::map<std::string, float>> mode_weights_;

  // Health thresholds
  float nominal_threshold_{0.75f};
  float degraded_threshold_{0.55f};

  // Current inputs
  OdometryInput lio_input_;
  OdometryInput vio_input_;
  OdometryInput kinematic_input_;

  // Current mode
  FusionMode current_mode_{FusionMode::NOMINAL};
  int mode_persistence_counter_{0};
  int cycles_to_confirm_{3};

  // Computed weights (smoothed with EMA)
  float smoothed_lio_weight_{0.50f};
  float smoothed_vio_weight_{0.35f};
  float smoothed_kinematic_weight_{0.15f};

  // Private methods
  float computeWeight(const OdometryInput & input);
  FusionMode determineMode();
  bool shouldSwitchMode(FusionMode new_mode);
  void applyModeWeights(FusionMode mode);
  float exponentialMovingAverage(float current, float previous, float alpha);
  geometry_msgs::msg::Pose interpolatePoses(
    const geometry_msgs::msg::Pose & from,
    const geometry_msgs::msg::Pose & to,
    float t);
};

}  // namespace fusion
}  // namespace juppiter

#endif  // FUSION_CORE__FUSION_ENGINE_HPP_
