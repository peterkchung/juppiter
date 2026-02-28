// Copyright 2026 Arconic Labs
// About: Stereo rectification and depth map computation via StereoSGBM.
// Pure C++/OpenCV — no ROS dependency.

#pragma once

#include <array>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace sensor_bridge
{

struct StereoParams
{
  // Image dimensions
  int width = 752;
  int height = 480;

  // Camera 0 (left) intrinsics
  double cam0_fx, cam0_fy, cam0_cx, cam0_cy;
  std::array<double, 5> cam0_d;

  // Camera 1 (right) intrinsics
  double cam1_fx, cam1_fy, cam1_cx, cam1_cy;
  std::array<double, 5> cam1_d;

  // Stereo extrinsics
  double baseline_m;
  std::array<double, 9> R_cam0_cam1;  // 3x3 row-major
  std::array<double, 3> T_cam0_cam1;

  // StereoSGBM tuning
  int num_disparities = 96;
  int block_size = 11;
};

class StereoDepth {
public:
  explicit StereoDepth(const StereoParams & params);

  // Compute a 32FC1 depth map from a rectified stereo pair.
  // Inputs are raw (unrectified) grayscale images.
  // Returns depth in meters; invalid pixels are 0.
  cv::Mat compute(const cv::Mat & left, const cv::Mat & right) const;

  // Focal length of rectified camera (for CameraInfo).
  double rectified_fx() const {return rectified_fx_;}

  // Principal point of rectified camera.
  double rectified_cx() const {return rectified_cx_;}
  double rectified_cy() const {return rectified_cy_;}

private:
  StereoParams params_;

  // Rectification maps
  cv::Mat map1_left_, map2_left_;
  cv::Mat map1_right_, map2_right_;

  // Rectified projection info
  double rectified_fx_ = 0.0;
  double rectified_cx_ = 0.0;
  double rectified_cy_ = 0.0;
  double baseline_m_ = 0.0;

  cv::Ptr<cv::StereoSGBM> sgbm_;
};

}  // namespace sensor_bridge
