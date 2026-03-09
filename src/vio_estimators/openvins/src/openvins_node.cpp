// About: OpenVINS ROS 2 Node

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "openvins/openvins_estimator.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("openvins_node");
  
  auto estimator = std::make_shared<juppiter::vio::OpenVinsEstimator>();
  
  // Parameters
  std::string config_path = node->declare_parameter<std::string>(
    "config_path", "config/compute_profiles/edge.yaml");
  
  // Camera info (load from calibration)
  sensor_msgs::msg::CameraInfo left_info;
  sensor_msgs::msg::CameraInfo right_info;
  
  if (!estimator->initialize(node, config_path, left_info, right_info)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize OpenVINS");
    return 1;
  }
  
  // Subscribers
  auto left_sub = node->create_subscription<sensor_msgs::msg::Image>(
    "/stereo/left/image_raw", 10,
    [estimator](const sensor_msgs::msg::Image::SharedPtr msg) {
      // Buffer for stereo sync
    });
    
  auto right_sub = node->create_subscription<sensor_msgs::msg::Image>(
    "/stereo/right/image_raw", 10,
    [estimator](const sensor_msgs::msg::Image::SharedPtr msg) {
      // Buffer for stereo sync
    });
    
  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", 100,
    [estimator](const sensor_msgs::msg::Imu::SharedPtr msg) {
      estimator->processImu(msg);
    });
  
  // Publishers
  auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(
    "/vio/odom", 10);
    
  auto health_pub = node->create_publisher<std_msgs::msg::String>(
    "/vio/health", 10);
  
  // Timer
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(50),
    [estimator, odom_pub, health_pub]() {
      auto odom = estimator->getOdometry();
      if (odom) {
        odom_pub->publish(*odom);
      }
      
      auto health = estimator->getHealthStatus();
      std_msgs::msg::String msg;
      msg.data = std::string("{\"score\": ") + 
                 std::to_string(health.health_score) + "}";
      health_pub->publish(msg);
    });
  
  RCLCPP_INFO(node->get_logger(), "OpenVINS node started (edge tier)");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
