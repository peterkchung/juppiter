// About: OpenVINS Adapter Implementation
// Wraps rpng/open_vins MSCKF for juppiter VioEstimator interface
// Optimized for edge tier with KLT tracking and selective updates

#ifndef OPENVINS__OPENVINS_ADAPTER_HPP_
#define OPENVINS__OPENVINS_ADAPTER_HPP_

#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "vio_estimator_interfaces/vio_estimator.hpp"

// Try to include OpenVINS headers, fallback gracefully if not installed
#ifdef OPENVINS_AVAILABLE
#include "ov_msckf/VioManager.h"
#include "ov_core/utils/dataset_reader.h"
#include "ov_core/utils/print.h"
#endif

#include <opencv2/opencv.hpp>

namespace juppiter
{
namespace vio
{

/**
 * @brief OpenVINS adapter implementing VioEstimator interface
 * 
 * Wraps rpng/open_vins MSCKF VIO with edge optimizations:
 * - KLT tracking (configurable to ORB)
 * - Selective frame skipping when LIO healthy
 * - Health metrics extraction for fusion
 * - Static initialization support
 */
class OpenVINSAdapter : public VioEstimator
{
public:
  OpenVINSAdapter();
  ~OpenVINSAdapter() override;

  // VioEstimator interface implementation
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

  // OpenVINS-specific methods
  void configureForProfile(const std::string & profile);
  void setLIOHealth(float health_score);
  bool isInitialized() const { return initialized_; }

private:
  rclcpp::Node::SharedPtr node_;
  bool initialized_{false};
  bool enabled_{true};
  bool tracking_{false};
  bool ov_initialized_{false};  // OpenVINS internal initialization state

  // Configuration
  std::string compute_profile_{"edge"};
  std::string feature_tracker_{"klt"};
  std::string calibration_file_;
  
  struct {
    int max_features{150};
    int min_features{50};
    float target_fps{10.0};  // Edge: 10Hz, Dev: 20Hz
    bool skip_if_lio_healthy{true};
    float lio_healthy_threshold{0.95f};
    int grid_x{5};
    int grid_y{4};
  } config_;

  // External health input
  float lio_health_score_{1.0f};
  std::mutex health_mutex_;

  // Frame timing
  float target_period_ms_{100.0f};  // 10Hz = 100ms
  rclcpp::Time last_process_time_;
  int frames_skipped_{0};
  int frames_processed_{0};

  // Buffers for synchronization
  std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;
  std::mutex imu_mutex_;
  
  // Latest odometry
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  mutable std::mutex odom_mutex_;
  rclcpp::Time last_odom_time_;

  // OpenVINS components (when available)
#ifdef OPENVINS_AVAILABLE
  std::shared_ptr<ov_msckf::VioManager> vio_manager_;
  std::shared_ptr<ov_core::Printer> printer_;
#else
  // Stub implementation
  struct {
    double timestamp{0.0};
    double position[3]{0.0, 0.0, 0.0};
    double orientation[4]{1.0, 0.0, 0.0, 0.0};  // w, x, y, z
    double velocity[3]{0.0, 0.0, 0.0};
    int tracked_features{0};
    float health_score{0.5f};
  } stub_state_;
#endif

  // Private methods
  void loadParameters(const std::string & config_path);
  bool shouldSkipFrame();
  bool waitForStaticInitialization();
  void feedIMU(const sensor_msgs::msg::Imu::SharedPtr & imu);
  void feedStereo(const cv::Mat & left, const cv::Mat & right, double timestamp);
  void extractOdometry();
  VioHealthStatus computeHealthMetrics() const;
  cv::Mat preprocessImage(const cv::Mat & image);
  
  // Utility
  double getCurrentTimestamp() const;
  void logDebug(const std::string & msg) const;
  void logInfo(const std::string & msg) const;
  void logWarn(const std::string & msg) const;
};

}  // namespace vio
}  // namespace juppiter

#endif  // OPENVINS__OPENVINS_ADAPTER_HPP_
