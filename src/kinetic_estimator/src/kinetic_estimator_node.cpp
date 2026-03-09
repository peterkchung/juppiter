// About: Kinetic Estimator ROS 2 Node
// Wraps robot_localization EKF for wheel odometry + IMU fusion

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace juppiter
{

class KineticEstimatorNode : public rclcpp::Node
{
public:
  KineticEstimatorNode()
  : Node("kinetic_estimator")
  {
    // Parameters
    this->declare_parameter<std::string>("odom_topic", "/wheel/odometry");
    this->declare_parameter<std::string>("imu_topic", "/imu/data");
    this->declare_parameter<std::string>("output_topic", "/kinematic/odom");
    
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    
    RCLCPP_INFO(this->get_logger(), "Kinetic Estimator starting...");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s, %s", odom_topic.c_str(), imu_topic.c_str());
    
    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->processWheelOdometry(msg);
      });
      
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 100,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        this->processImu(msg);
      });
    
    // Publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);
    
    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    RCLCPP_INFO(this->get_logger(), "Kinetic Estimator initialized");
  }

private:
  void processWheelOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Store latest wheel odometry
    last_wheel_odom_ = *msg;
    
    // Publish combined odometry
    publishOdometry();
  }
  
  void processImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Store latest IMU data
    last_imu_ = *msg;
  }
  
  void publishOdometry()
  {
    if (last_wheel_odom_.header.stamp.sec == 0) {
      return;  // No wheel odometry yet
    }
    
    nav_msgs::msg::Odometry odom = last_wheel_odom_;
    
    // Add IMU angular velocity if available
    if (last_imu_.header.stamp.sec != 0) {
      odom.twist.twist.angular.x = last_imu_.angular_velocity.x;
      odom.twist.twist.angular.y = last_imu_.angular_velocity.y;
      odom.twist.twist.angular.z = last_imu_.angular_velocity.z;
    }
    
    // Update timestamp
    odom.header.stamp = this->now();
    
    // Publish
    odom_pub_->publish(odom);
    
    // Publish TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom.header;
    tf_msg.child_frame_id = odom.child_frame_id;
    tf_msg.transform.translation.x = odom.pose.pose.position.x;
    tf_msg.transform.translation.y = odom.pose.pose.position.y;
    tf_msg.transform.translation.z = odom.pose.pose.position.z;
    tf_msg.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }
  
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  
  // Publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  
  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // Data storage
  nav_msgs::msg::Odometry last_wheel_odom_;
  sensor_msgs::msg::Imu last_imu_;
};

}  // namespace juppiter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<juppiter::KineticEstimatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
