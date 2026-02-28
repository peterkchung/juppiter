// Copyright 2026 Arconic Labs
// About: EuRoC MAV dataset parser implementation.

#include "euroc_reader.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>

namespace sensor_bridge
{

EurocReader::EurocReader(const std::string & mav0_path)
: mav0_path_(mav0_path)
{
  if (!std::filesystem::is_directory(mav0_path_)) {
    throw std::runtime_error("EuRoC mav0 directory not found: " + mav0_path);
  }

  load_image_csv(mav0_path_ / "cam0" / "data.csv", cam0_entries_);
  load_image_csv(mav0_path_ / "cam1" / "data.csv", cam1_entries_);
  load_imu_csv(mav0_path_ / "imu0" / "data.csv");

  if (cam0_entries_.empty()) {
    throw std::runtime_error("No cam0 images found in " + mav0_path);
  }
}

cv::Mat EurocReader::load_image(int cam_id, size_t index) const
{
  const auto & entries = (cam_id == 0) ? cam0_entries_ : cam1_entries_;
  if (index >= entries.size()) {
    throw std::out_of_range("Image index out of range");
  }

  auto cam_dir = (cam_id == 0) ? "cam0" : "cam1";
  auto path = mav0_path_ / cam_dir / "data" / entries[index].filename;
  cv::Mat img = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);

  if (img.empty()) {
    throw std::runtime_error("Failed to load image: " + path.string());
  }
  return img;
}

int64_t EurocReader::image_timestamp_ns(size_t index) const
{
  if (index >= cam0_entries_.size()) {
    throw std::out_of_range("Image index out of range");
  }
  return cam0_entries_[index].timestamp_ns;
}

const std::vector<ImageEntry> & EurocReader::image_entries(int cam_id) const
{
  return (cam_id == 0) ? cam0_entries_ : cam1_entries_;
}

void EurocReader::load_image_csv(
  const std::filesystem::path & csv_path,
  std::vector<ImageEntry> & entries)
{
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open CSV: " + csv_path.string());
  }

  std::string line;
  // Skip header line
  std::getline(file, line);

  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {continue;}

    std::istringstream ss(line);
    std::string ts_str, filename;
    if (std::getline(ss, ts_str, ',') && std::getline(ss, filename)) {
      ImageEntry entry;
      entry.timestamp_ns = std::stoll(ts_str);
      // Trim whitespace from filename
      auto start = filename.find_first_not_of(" \t\r\n");
      if (start != std::string::npos) {
        filename = filename.substr(start);
      }
      entry.filename = filename;
      entries.push_back(entry);
    }
  }
}

void EurocReader::load_imu_csv(const std::filesystem::path & csv_path)
{
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open IMU CSV: " + csv_path.string());
  }

  std::string line;
  // Skip header line
  std::getline(file, line);

  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {continue;}

    std::istringstream ss(line);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(ss, token, ',')) {
      tokens.push_back(token);
    }

    // EuRoC IMU CSV: timestamp, gx, gy, gz, ax, ay, az
    if (tokens.size() >= 7) {
      ImuSample sample;
      sample.timestamp_ns = std::stoll(tokens[0]);
      sample.gx = std::stod(tokens[1]);
      sample.gy = std::stod(tokens[2]);
      sample.gz = std::stod(tokens[3]);
      sample.ax = std::stod(tokens[4]);
      sample.ay = std::stod(tokens[5]);
      sample.az = std::stod(tokens[6]);
      imu_samples_.push_back(sample);
    }
  }
}

}  // namespace sensor_bridge
