// About: EuRoC MAV dataset parser. Loads CSV timestamps and images on demand.
// Pure C++/OpenCV — no ROS dependency.

#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace sensor_bridge {

struct ImuSample {
  int64_t timestamp_ns;
  double gx, gy, gz;  // gyroscope [rad/s]
  double ax, ay, az;   // accelerometer [m/s^2]
};

struct ImageEntry {
  int64_t timestamp_ns;
  std::string filename;
};

class EurocReader {
 public:
  // mav0_path should point to the mav0/ directory of a EuRoC sequence.
  explicit EurocReader(const std::string& mav0_path);

  // Number of image frames in cam0.
  size_t num_frames() const { return cam0_entries_.size(); }

  // Number of IMU samples.
  size_t num_imu_samples() const { return imu_samples_.size(); }

  // Load a grayscale image from cam0 or cam1 by index.
  cv::Mat load_image(int cam_id, size_t index) const;

  // Access image timestamps.
  int64_t image_timestamp_ns(size_t index) const;

  // Access all IMU samples (sorted by timestamp).
  const std::vector<ImuSample>& imu_samples() const { return imu_samples_; }

  // Access image entries for a camera.
  const std::vector<ImageEntry>& image_entries(int cam_id) const;

 private:
  std::filesystem::path mav0_path_;
  std::vector<ImageEntry> cam0_entries_;
  std::vector<ImageEntry> cam1_entries_;
  std::vector<ImuSample> imu_samples_;

  void load_image_csv(const std::filesystem::path& csv_path,
                      std::vector<ImageEntry>& entries);
  void load_imu_csv(const std::filesystem::path& csv_path);
};

}  // namespace sensor_bridge
