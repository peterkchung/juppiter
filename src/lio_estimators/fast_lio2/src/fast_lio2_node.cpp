// About: FAST-LIO2 ROS 2 Node
// Wrapper node exposing FAST-LIO2 through the LioEstimator interface

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "fast_lio2/fast_lio2_estimator.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("fast_lio2_node");
  
  // Create estimator
  auto estimator = std::make_shared<juppiter::lio::FastLio2Estimator>();
  
  // Initialize with parameters
  std::string config_path = node->declare_parameter<std::string>("config_path", 
    "config/compute_profiles/dev.yaml");
  
  if (!estimator->initialize(node, config_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize FAST-LIO2 estimator");
    return 1;
  }
  
  // Subscribe to sensor topics
  auto lidar_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar/points", 10,
    [estimator](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      estimator->processPointCloud(msg);
    });
    
  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", 100,
    [estimator](const sensor_msgs::msg::Imu::SharedPtr msg) {
      estimator->processImu(msg);
    });
  
  // Publish odometry
  auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(
    "/lio/odom", 10);
    
  auto health_pub = node->create_publisher<std_msgs::msg::String>(
    "/lio/health", 10);
  
  // Timer for publishing odometry and health
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    [estimator, odom_pub, health_pub, node]() {
      // Publish odometry
      auto odom = estimator->getOdometry();
      if (odom) {
        odom_pub->publish(*odom);
      }
      
      // Publish health
      auto health = estimator->getHealthStatus();
      std_msgs::msg::String health_msg;
      health_msg.data = std::string("{\"score\": ") + 
                        std::to_string(health.health_score) + 
                        ", \"healthy\": " + 
                        (health.is_healthy ? "true" : "false") + "}";
      health_pub->publish(health_msg);
    });
  
  RCLCPP_INFO(node->get_logger(), "FAST-LIO2 node started");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
