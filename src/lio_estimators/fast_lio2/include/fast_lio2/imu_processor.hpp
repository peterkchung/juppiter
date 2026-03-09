# About: IMU Processor for FAST-LIO2
# IMU integration and bias estimation

#ifndef FAST_LIO2__IMU_PROCESSOR_HPP_
#define FAST_LIO2__IMU_PROCESSOR_HPP_

#include <vector>
#include <Eigen/Dense>

namespace juppiter
{
namespace lio
{

/**
 * @brief IMU sample with timestamp
 */
struct ImuSample
{
  Eigen::Vector3d accel;
  Eigen::Vector3d gyro;
  double timestamp;
};

/**
 * @brief IMU processor for bias estimation and integration
 * 
 * Handles IMU data preprocessing including:
 * - Bias estimation during initialization
 * - IMU integration for state prediction
 * - Gravity vector estimation
 */
class ImuProcessor
{
public:
  ImuProcessor();
  ~ImuProcessor();

  /**
   * @brief Initialize with gravity direction
   */
  void initialize(const Eigen::Vector3d & gravity);

  /**
   * @brief Process IMU measurement
   * @param accel Accelerometer reading (m/s^2)
   * @param gyro Gyroscope reading (rad/s)
   * @param dt Time step (seconds)
   */
  void processImu(const Eigen::Vector3d & accel,
                  const Eigen::Vector3d & gyro,
                  double dt);

  /**
   * @brief Estimate IMU bias from stationary samples
   */
  void estimateBias(const std::vector<ImuSample> & samples);

  /**
   * @brief Get current bias estimates
   */
  Eigen::Vector3d getAccelBias() const { return bias_accel_; }
  Eigen::Vector3d getGyroBias() const { return bias_gyro_; }

private:
  Eigen::Vector3d bias_accel_;
  Eigen::Vector3d bias_gyro_;
  Eigen::Vector3d gravity_;
  bool initialized_;
};

}  // namespace lio
}  // namespace juppiter

#endif  // FAST_LIO2__IMU_PROCESSOR_HPP_
