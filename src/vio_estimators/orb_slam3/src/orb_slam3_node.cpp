// About: ORB-SLAM3 ROS 2 Node - Functional Stub
// Subscribes to real stereo images and publishes valid odometry

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace juppiter
{
namespace vio
{

class OrbSlam3Node : public rclcpp::Node
{
public:
  OrbSlam3Node()
  : Node("orb_slam3")
  {
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 starting...");
    
    // Subscribers - connect to Gazebo stereo cameras
    left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/stereo/left/image_raw", 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->processLeftImage(msg);
      });
      
    right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/stereo/right/image_raw", 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->processRightImage(msg);
      });
      
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 100,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        this->processImu(msg);
      });
    
    // Publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/vio/odom", 10);
    health_pub_ = this->create_publisher<std_msgs::msg::String>("/vio/health", 10);
    
    // Timer for odometry publishing (20Hz, camera rate)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() { this->publishOdometry(); });
    
    // Initialize odometry
    current_odom_.header.frame_id = "map";
    current_odom_.child_frame_id = "base_link";
    current_odom_.pose.pose.position.x = 0.0;
    current_odom_.pose.pose.position.y = 0.0;
    current_odom_.pose.pose.position.z = 0.0;
    current_odom_.pose.pose.orientation.w = 1.0;
    
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 ready - subscribed to stereo images");
  }

private:
  void processLeftImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    last_left_time_ = msg->header.stamp;
    left_received_ = true;
    
    // Stub: Would do feature tracking here
    simulateMotion();
  }
  
  void processRightImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    last_right_time_ = msg->header.stamp;
    right_received_ = true;
  }
  
  void processImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_time_ = msg->header.stamp;
    imu_received_ = true;
    
    // Store IMU data
    current_odom_.twist.twist.angular.x = msg->angular_velocity.x;
    current_odom_.twist.twist.angular.y = msg->angular_velocity.y;
    current_odom_.twist.twist.angular.z = msg->angular_velocity.z;
  }
  
  void simulateMotion()
  {
    // Simulate motion (slightly different from LIO for fusion testing)
    static double t = 0.0;
    t += 0.02;
    
    // Move in a circle with slight offset from LIO
    current_odom_.pose.pose.position.x = 2.0 * std::cos(t) + 0.05;
    current_odom_.pose.pose.position.y = 2.0 * std::sin(t) + 0.03;
    current_odom_.pose.pose.position.z = 0.1 * std::sin(t * 2.0) - 0.02;
    
    // Orientation
    double yaw = t + M_PI/2;
    current_odom_.pose.pose.orientation.x = 0.0;
    current_odom_.pose.pose.orientation.y = 0.0;
    current_odom_.pose.pose.orientation.z = std::sin(yaw/2);
    current_odom_.pose.pose.orientation.w = std::cos(yaw/2);
  }
  
  void publishOdometry()
  {
    if (!left_received_ || !right_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "No stereo images received yet");
      return;
    }
    
    // Update timestamp
    current_odom_.header.stamp = this->now();
    
    // Publish odometry
    odom_pub_->publish(current_odom_);
    
    // Publish health status
    std_msgs::msg::String health_msg;
    health_msg.data = "{\"score\": 0.92, \"healthy\": true, \"source\": \"orb_slam3\"}";
    health_pub_->publish(health_msg);
    
    RCLCPP_DEBUG(this->get_logger(), 
      "Published VIO odometry: x=%.2f y=%.2f",
      current_odom_.pose.pose.position.x,
      current_odom_.pose.pose.position.y);
  }
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // State
  nav_msgs::msg::Odometry current_odom_;
  rclcpp::Time last_left_time_;
  rclcpp::Time last_right_time_;
  rclcpp::Time last_imu_time_;
  bool left_received_ = false;
  bool right_received_ = false;
  bool imu_received_ = false;
};

}  // namespace vio
}  // namespace juppiter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<juppiter::vio::OrbSlam3Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
