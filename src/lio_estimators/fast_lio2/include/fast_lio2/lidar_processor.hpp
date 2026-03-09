// About: LiDAR Processor for FAST-LIO2
// Point cloud preprocessing and undistortion

#ifndef FAST_LIO2__LIDAR_PROCESSOR_HPP_
#define FAST_LIO2__LIDAR_PROCESSOR_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "fast_lio2/imu_processor.hpp"

namespace juppiter
{
namespace lio
{

/**
 * @brief LiDAR point cloud preprocessing
 * 
 * Handles:
 * - Downsampling (voxel grid filter)
 * - Outlier removal
 * - Point cloud undistortion using IMU data
 */
class LidarProcessor
{
public:
  LidarProcessor();
  ~LidarProcessor();

  /**
   * @brief Set voxel grid leaf size for downsampling
   */
  void setDownsampleLeafSize(float size);

  /**
   * @brief Process point cloud (downsample + outlier removal)
   * @return Filtered point cloud
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr process(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & input_cloud);

  /**
   * @brief Undistort point cloud using IMU motion data
   * Compensates for motion during LiDAR scan acquisition
   */
  void undistortPointCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
    const std::vector<ImuSample> & imu_samples);

private:
  float downsample_leaf_size_;
};

}  // namespace lio
}  // namespace juppiter

#endif  // FAST_LIO2__LIDAR_PROCESSOR_HPP_
