// About: OpenVINS ROS 2 Node
// MSCKF-based VIO for juppiter edge tier
// Wraps rpng/open_vins with juppiter interface

#include <chrono>
#include <string>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

#include "openvins/openvins_adapter.hpp"
#include "vio_estimator_interfaces/vio_estimator.hpp"

namespace juppiter
{
namespace vio
{

class OpenVINSNode : public rclcpp::Node
{
public:
  OpenVINSNode()
  : Node("openvins"),
    adapter_(std::make_shared<OpenVINSAdapter>())
  {
    RCLCPP_INFO(get_logger(), "OpenVINS node starting...");
    declare_parameters();
    RCLCPP_INFO(get_logger(), "Parameters declared");
  }
  
  void initialize()
  {
    std::string compute_profile = get_parameter("compute_profile").as_string();
    RCLCPP_INFO(get_logger(), "Compute profile: %s", compute_profile.c_str());
    
    sensor_msgs::msg::CameraInfo left_info;
    sensor_msgs::msg::CameraInfo right_info;
    
    if (!adapter_->initialize(this->shared_from_this(), 
                              compute_profile + ".yaml",
                              left_info, right_info)) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize OpenVINS adapter");
      throw std::runtime_error("OpenVINS initialization failed");
    }
    
    adapter_->configureForProfile(compute_profile);
    setup_subscribers();
    setup_publishers();
    setup_timer();
    
    if (compute_profile == "edge" || compute_profile == "low_power") {
      setup_lio_health_subscriber();
    }
    
    RCLCPP_INFO(get_logger(), "OpenVINS node initialized successfully");
    RCLCPP_INFO(get_logger(), "Feature tracker: %s", 
                get_parameter("feature_tracker").as_string().c_str());
    RCLCPP_INFO(get_logger(), "Target FPS: %.1f", 
                get_parameter("target_fps").as_double());
  }

private:
  void declare_parameters()
  {
    declare_parameter("compute_profile", "edge");
    declare_parameter("feature_tracker", "klt");
    declare_parameter("target_fps", 10.0);
    declare_parameter("topic_left", "/stereo/left/image_raw");
    declare_parameter("topic_right", "/stereo/right/image_raw");
    declare_parameter("topic_imu", "/imu/data");
    declare_parameter("calibration_file", "$(find openvins)/config/camera_simulation.yaml");
    declare_parameter("skip_if_lio_healthy", true);
    declare_parameter("lio_healthy_threshold", 0.95);
    declare_parameter("wait_for_static_init", true);
    declare_parameter("init_duration_sec", 1.0);
    declare_parameter("pub_image_track", false);
  }
  
  void setup_subscribers()
  {
    std::string topic_left = get_parameter("topic_left").as_string();
    left_sub_ = create_subscription<sensor_msgs::msg::Image>(
      topic_left, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        left_buffer_ = msg;
        check_stereo_sync();
      });
    
    std::string topic_right = get_parameter("topic_right").as_string();
    right_sub_ = create_subscription<sensor_msgs::msg::Image>(
      topic_right, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        right_buffer_ = msg;
        check_stereo_sync();
      });
    
    std::string topic_imu = get_parameter("topic_imu").as_string();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      topic_imu, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        adapter_->processImu(msg);
        imu_count_++;
      });
    
    RCLCPP_INFO(get_logger(), "Subscribers created");
  }
  
  void setup_publishers()
  {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/vio/odom", 10);
    health_pub_ = create_publisher<std_msgs::msg::String>("/vio/health", 10);
    
    bool pub_debug = get_parameter("pub_image_track").as_bool();
    if (pub_debug) {
      debug_img_pub_ = create_publisher<sensor_msgs::msg::Image>("/vio/debug_image", 10);
    }
    
    RCLCPP_INFO(get_logger(), "Publishers created");
  }
  
  void setup_timer()
  {
    double target_fps = get_parameter("target_fps").as_double();
    int period_ms = static_cast<int>(1000.0 / target_fps);
    
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      [this]() { publish_outputs(); });
    
    RCLCPP_INFO(get_logger(), "Timer: %d ms (%.1f Hz)", period_ms, target_fps);
  }
  
  void setup_lio_health_subscriber()
  {
    lio_health_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/lio/health_score", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        adapter_->setLIOHealth(msg->data);
      });
    
    RCLCPP_INFO(get_logger(), "LIO health subscriber created");
  }
  
  void check_stereo_sync()
  {
    if (!left_buffer_ || !right_buffer_) return;
    
    double left_time = left_buffer_->header.stamp.sec + 
                       left_buffer_->header.stamp.nanosec * 1e-9;
    double right_time = right_buffer_->header.stamp.sec + 
                        right_buffer_->header.stamp.nanosec * 1e-9;
    
    if (std::abs(left_time - right_time) > 0.01) {
      return;
    }
    
    adapter_->processStereo(left_buffer_, right_buffer_);
    left_buffer_.reset();
    right_buffer_.reset();
    stereo_pairs_processed_++;
  }
  
  void publish_outputs()
  {
    auto odom = adapter_->getOdometry();
    if (odom) {
      odom_pub_->publish(*odom);
    }
    
    auto health = adapter_->getHealthStatus();
    std_msgs::msg::String msg;
    msg.data = serialize_health(health);
    health_pub_->publish(msg);
    
    if (stereo_pairs_processed_ % 100 == 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000,
        "VIO: health=%.2f, features=%d",
        health.health_score, health.tracked_features);
    }
  }
  
  std::string serialize_health(const VioHealthStatus& health)
  {
    std::string json = "{";
    json += "\"healthy\":" + std::string(health.is_healthy ? "true" : "false") + ",";
    json += "\"score\":" + std::to_string(health.health_score) + ",";
    json += "\"covariance\":" + std::to_string(health.covariance_norm) + ",";
    json += "\"features\":" + std::to_string(health.tracked_features) + ",";
    json += "\"reprojection_error\":" + std::to_string(health.reprojection_error) + ",";
    json += "\"source\":\"openvins\"";
    
    if (!health.active_faults.empty()) {
      json += ",\"faults\":[";
      for (size_t i = 0; i < health.active_faults.size(); i++) {
        if (i > 0) json += ",";
        json += "\"" + health.active_faults[i] + "\"";
      }
      json += "]";
    }
    
    json += "}";
    return json;
  }
  
  std::shared_ptr<OpenVINSAdapter> adapter_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lio_health_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr left_buffer_;
  sensor_msgs::msg::Image::SharedPtr right_buffer_;
  int stereo_pairs_processed_ = 0;
  int imu_count_ = 0;
};

}  // namespace vio
}  // namespace juppiter

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<juppiter::vio::OpenVINSNode>();
    node->initialize();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("openvins"), "Fatal error: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
