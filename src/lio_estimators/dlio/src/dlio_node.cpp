// About: DLIO ROS 2 Node
// Wrapper node exposing DLIO through LioEstimator interface

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "dlio/dlio_estimator.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("dlio_node");
  
  // Create estimator
  auto estimator = std::make_shared<juppiter::lio::DlioEstimator>();
  
  // Initialize
  std::string config_path = node->declare_parameter<std::string>("config_path", 
    "config/compute_profiles/edge.yaml");
  
  if (!estimator->initialize(node, config_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize DLIO estimator");
    return 1;
  }
  
  // Subscribers
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
  
  // Publishers
  auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(
    "/lio/odom", 10);
    
  auto health_pub = node->create_publisher<std_msgs::msg::String>(
    "/lio/health", 10);
  
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
  
  RCLCPP_INFO(node->get_logger(), "DLIO node started (edge tier)");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
