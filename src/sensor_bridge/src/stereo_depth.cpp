// About: Stereo rectification and StereoSGBM depth computation.

#include "stereo_depth.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace sensor_bridge {

StereoDepth::StereoDepth(const StereoParams& params) : params_(params) {
  // Build intrinsic matrices
  cv::Mat K0 = (cv::Mat_<double>(3, 3) <<
      params.cam0_fx, 0, params.cam0_cx,
      0, params.cam0_fy, params.cam0_cy,
      0, 0, 1);
  cv::Mat K1 = (cv::Mat_<double>(3, 3) <<
      params.cam1_fx, 0, params.cam1_cx,
      0, params.cam1_fy, params.cam1_cy,
      0, 0, 1);

  cv::Mat D0(1, 5, CV_64F, const_cast<double*>(params.cam0_d.data()));
  cv::Mat D1(1, 5, CV_64F, const_cast<double*>(params.cam1_d.data()));

  // Rotation and translation from cam0 to cam1
  cv::Mat R(3, 3, CV_64F, const_cast<double*>(params.R_cam0_cam1.data()));
  cv::Mat T(3, 1, CV_64F, const_cast<double*>(params.T_cam0_cam1.data()));

  cv::Size img_size(params.width, params.height);

  // Compute rectification transforms
  cv::Mat R1, R2, P1, P2, Q;
  cv::stereoRectify(K0, D0, K1, D1, img_size, R, T,
                    R1, R2, P1, P2, Q,
                    cv::CALIB_ZERO_DISPARITY, 0, img_size);

  // Build undistort+rectify maps
  cv::initUndistortRectifyMap(K0, D0, R1, P1, img_size, CV_32FC1,
                              map1_left_, map2_left_);
  cv::initUndistortRectifyMap(K1, D1, R2, P2, img_size, CV_32FC1,
                              map1_right_, map2_right_);

  // Extract rectified camera parameters from P1
  rectified_fx_ = P1.at<double>(0, 0);
  rectified_cx_ = P1.at<double>(0, 2);
  rectified_cy_ = P1.at<double>(1, 2);
  baseline_m_ = params.baseline_m;

  // Create StereoSGBM matcher
  sgbm_ = cv::StereoSGBM::create(
      0,                       // minDisparity
      params.num_disparities,  // numDisparities
      params.block_size,       // blockSize
      8 * params.block_size * params.block_size,   // P1
      32 * params.block_size * params.block_size,  // P2
      1,     // disp12MaxDiff
      0,     // preFilterCap (auto)
      10,    // uniquenessRatio
      100,   // speckleWindowSize
      32,    // speckleRange
      cv::StereoSGBM::MODE_SGBM_3WAY);
}

cv::Mat StereoDepth::compute(const cv::Mat& left, const cv::Mat& right) const {
  // Rectify
  cv::Mat rect_left, rect_right;
  cv::remap(left, rect_left, map1_left_, map2_left_, cv::INTER_LINEAR);
  cv::remap(right, rect_right, map1_right_, map2_right_, cv::INTER_LINEAR);

  // Compute disparity (16-bit fixed-point, 4 fractional bits)
  cv::Mat disparity_16s;
  sgbm_->compute(rect_left, rect_right, disparity_16s);

  // Convert to float disparity
  cv::Mat disparity_f;
  disparity_16s.convertTo(disparity_f, CV_32F, 1.0 / 16.0);

  // Convert disparity to depth: depth = fx * baseline / disparity
  cv::Mat depth(disparity_f.size(), CV_32FC1, 0.0f);
  for (int y = 0; y < disparity_f.rows; ++y) {
    const float* disp_row = disparity_f.ptr<float>(y);
    float* depth_row = depth.ptr<float>(y);
    for (int x = 0; x < disparity_f.cols; ++x) {
      float d = disp_row[x];
      if (d > 0.0f) {
        depth_row[x] = static_cast<float>(rectified_fx_ * baseline_m_) / d;
      }
    }
  }

  return depth;
}

}  // namespace sensor_bridge
