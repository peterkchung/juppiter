// About: LiDAR Processor Stub
// Point cloud preprocessing for FAST-LIO2

#include "fast_lio2/lidar_processor.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace juppiter
{
namespace lio
{

LidarProcessor::LidarProcessor()
: downsample_leaf_size_(0.1f)
{
}

LidarProcessor::~LidarProcessor() = default;

void LidarProcessor::setDownsampleLeafSize(float size)
{
  downsample_leaf_size_ = size;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LidarProcessor::process(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & input_cloud)
{
  // Downsample
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(input_cloud);
  voxel_grid.setLeafSize(downsample_leaf_size_, downsample_leaf_size_, downsample_leaf_size_);
  voxel_grid.filter(*downsampled);
  
  // Remove outliers
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud(downsampled);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*filtered);
  
  return filtered;
}

void LidarProcessor::undistortPointCloud(
  pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
  const std::vector<ImuSample> & imu_samples)
{
  // TODO: Implement point cloud undistortion using IMU data
  // This compensates for motion during LiDAR scan acquisition
}

}  // namespace lio
}  // namespace juppiter
