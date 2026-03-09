// About: Fusion Node
// ROS 2 node for fusion_core integration

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

#include "fusion_core/fusion_engine.hpp"
#include "fusion_core/mode_manager.hpp"

namespace juppiter
{
namespace fusion
{

class FusionNode : public rclcpp::Node
{
public:
  FusionNode()
  : Node("fusion_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("compute_profile", "dev");
    this->declare_parameter<std::string>("fusion_config", "config/fusion/weights_nominal.yaml");
    this->declare_parameter<std::string>("health_config", "config/fusion/health_thresholds.yaml");
    this->declare_parameter<double>("fusion_rate_hz", 20.0);
    
    // Load parameters
    std::string compute_profile = this->get_parameter("compute_profile").as_string();
    std::string fusion_config = this->get_parameter("fusion_config").as_string();
    std::string health_config = this->get_parameter("health_config").as_string();
    double fusion_rate = this->get_parameter("fusion_rate_hz").as_double();
    
    RCLCPP_INFO(this->get_logger(), 
      "FusionNode starting with profile: %s", compute_profile.c_str());
    
    // Initialize fusion engine
    engine_ = std::make_unique<FusionEngine>();
    if (!engine_->initialize(this->shared_from_this(), fusion_config)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize fusion engine");
      return;
    }
    
    // Initialize mode manager
    mode_manager_ = std::make_unique<ModeManager>();
    if (!mode_manager_->initialize(this->shared_from_this(), health_config)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize mode manager");
      return;
    }
    
    // Register mode change callback
    mode_manager_->registerModeChangeCallback(
      [this](const ModeTransition & transition) {
        this->onModeChange(transition);
      });
    
    // Create subscribers
    lio_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/lio/odom", 10,
      std::bind(&FusionNode::onLioOdometry, this, std::placeholders::_1));
      
    vio_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/vio/odom", 10,
      std::bind(&FusionNode::onVioOdometry, this, std::placeholders::_1));
      
    kinematic_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/kinematic/odom", 10,
      std::bind(&FusionNode::onKinematicOdometry, this, std::placeholders::_1));
    
    // Create publishers
    fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/perception/odom", 10);
    mode_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/perception/mode", 10);
    
    // Create fusion timer
    fusion_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / fusion_rate)),
      std::bind(&FusionNode::fusionLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "FusionNode initialized successfully");
  }

private:
  void onLioOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Extract covariance norm (simplified)
    float cov_norm = std::sqrt(
      msg->pose.covariance[0] + msg->pose.covariance[7] + msg->pose.covariance[14]);
    
    lio::LioHealthStatus health;
    health.health_score = 1.0f;  // TODO: Get from actual health monitor
    health.covariance_norm = cov_norm;
    
    engine_->updateLioInput(health);
  }
  
  void onVioOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    float cov_norm = std::sqrt(
      msg->pose.covariance[0] + msg->pose.covariance[7] + msg->pose.covariance[14]);
    
    vio::VioHealthStatus health;
    health.health_score = 1.0f;
    health.covariance_norm = cov_norm;
    
    engine_->updateVioInput(health);
  }
  
  void onKinematicOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    float cov_norm = std::sqrt(
      msg->pose.covariance[0] + msg->pose.covariance[7] + msg->pose.covariance[14]);
    
    engine_->updateKinematicInput(msg, cov_norm);
  }
  
  void fusionLoop()
  {
    // Update mode manager transition state
    mode_manager_->updateTransition(0.05f);  // Assume 50ms loop
    
    // Compute fusion
    if (engine_->isFusionReady()) {
      FusedOdometry result = engine_->computeFusion();
      
      // Publish fused odometry
      fused_odom_pub_->publish(result.odometry);
      
      // Publish mode
      std_msgs::msg::String mode_msg;
      mode_msg.data = engine_->getModeString();
      mode_pub_->publish(mode_msg);
      
      // Check for mode transitions
      FusionMode evaluated_mode = mode_manager_->evaluateMode(
        result.source_weights.at("lio"),
        result.source_weights.at("vio"),
        result.source_weights.at("kinematic"));
        
      if (mode_manager_->shouldSwitchMode(evaluated_mode)) {
        mode_manager_->executeModeSwitch(evaluated_mode, "Health threshold crossed");
      }
    }
  }
  
  void onModeChange(const ModeTransition & transition)
  {
    RCLCPP_INFO(this->get_logger(), 
      "Mode changed from %s to %s",
      mode_manager_->getCurrentMode(),
      transition.to);
    
    // TODO: Update fusion engine weights based on new mode
  }
  
  // Core components
  std::unique_ptr<FusionEngine> engine_;
  std::unique_ptr<ModeManager> mode_manager_;
  
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kinematic_odom_sub_;
  
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr fusion_timer_;
};

}  // namespace fusion
}  // namespace juppiter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<juppiter::fusion::FusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
