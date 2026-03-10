// About: Fusion Node
// ROS 2 node for fusion_core with on-demand health queries

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

#include "fusion_core/fusion_engine.hpp"
#include "fusion_core/mode_manager.hpp"
#include "fusion_core/health_client.hpp"
#include "common_msgs/msg/perception_health.hpp"

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
    
    // Health client configuration (edge compute optimized)
    this->declare_parameter<double>("health_service_timeout_ms", 50.0);
    this->declare_parameter<double>("health_cache_ttl_ms", 100.0);  // 2 cycles at 20Hz
    this->declare_parameter<double>("health_decay_rate", 0.05);     // 5% per cycle
    
    RCLCPP_INFO(this->get_logger(), 
      "FusionNode created with profile: %s", 
      this->get_parameter("compute_profile").as_string().c_str());
  }
  
  void initialize()
  {
    // Load parameters
    std::string compute_profile = this->get_parameter("compute_profile").as_string();
    std::string fusion_config = this->get_parameter("fusion_config").as_string();
    std::string health_config = this->get_parameter("health_config").as_string();
    double fusion_rate = this->get_parameter("fusion_rate_hz").as_double();
    double health_timeout = this->get_parameter("health_service_timeout_ms").as_double();
    double health_cache_ttl = this->get_parameter("health_cache_ttl_ms").as_double();
    double health_decay = this->get_parameter("health_decay_rate").as_double();
    
    RCLCPP_INFO(this->get_logger(), 
      "FusionNode initializing with profile: %s", compute_profile.c_str());
    
    // Initialize health client (on-demand queries with caching)
    health_client_ = std::make_unique<HealthClient>(this->shared_from_this());
    if (!health_client_->initialize(health_timeout, health_cache_ttl, health_decay)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize health client");
      return;
    }
    
    // Initialize fusion engine
    engine_ = std::make_unique<FusionEngine>();
    if (!engine_->initialize(this->shared_from_this(), fusion_config)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize fusion engine");
      return;
    }
    
    // Initialize mode manager (single authority for mode decisions)
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
    health_pub_ = this->create_publisher<common_msgs::msg::PerceptionHealth>(
      "/perception/health", 10);  // Forward health for diagnostics
    
    // Create fusion timer
    fusion_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / fusion_rate)),
      std::bind(&FusionNode::fusionLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "FusionNode initialized successfully");
  }

private:
  void onLioOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    engine_->updateLioOdometry(msg);
  }
  
  void onVioOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    engine_->updateVioOdometry(msg);
  }
  
  void onKinematicOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    engine_->updateKinematicOdometry(msg);
  }
  
  void fusionLoop()
  {
    // Get health from sensor_core (on-demand with caching)
    auto health_opt = health_client_->getHealth();
    
    if (!health_opt.has_value()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Health unavailable, using fallback with decay");
    }
    
    // Use health for fusion
    auto health = health_opt.value_or(health_client_->getCachedHealth());
    
    // Update fusion engine with health data
    engine_->updateHealth(health);
    
    // Update mode manager transition state
    mode_manager_->updateTransition(0.05f);  // Assume 50ms loop
    
    // Compute fusion
    if (engine_->isFusionReady()) {
      FusedOdometry result = engine_->computeFusion();
      
      // Determine mode based on health (fusion_core is authority)
      FusionMode evaluated_mode = mode_manager_->evaluateMode(
        health.lio_health,
        health.vio_health,
        health.kinematic_health,
        health.overall_health);
        
      if (mode_manager_->shouldSwitchMode(evaluated_mode)) {
        mode_manager_->executeModeSwitch(evaluated_mode, "Health threshold crossed");
      }
      
      // Update result with authoritative mode
      result.mode = evaluated_mode;
      
      // Publish fused odometry
      fused_odom_pub_->publish(result.odometry);
      
      // Publish mode
      std_msgs::msg::String mode_msg;
      mode_msg.data = mode_manager_->modeToString(evaluated_mode);
      mode_pub_->publish(mode_msg);
      
      // Update health with authoritative mode and publish
      health.mode = static_cast<uint8_t>(evaluated_mode);
      health.mode_string = mode_msg.data;
      health.overall_health = result.overall_health;
      health_pub_->publish(health);
    }
  }
  
  void onModeChange(const ModeTransition & transition)
  {
    RCLCPP_INFO(this->get_logger(), 
      "Mode changed: %s", 
      mode_manager_->modeToString(transition.new_mode).c_str());
    
    // Update fusion engine weights based on new mode
    engine_->setMode(transition.new_mode);
  }
  
  // Core components
  std::unique_ptr<FusionEngine> engine_;
  std::unique_ptr<ModeManager> mode_manager_;
  std::unique_ptr<HealthClient> health_client_;
  
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kinematic_odom_sub_;
  
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<common_msgs::msg::PerceptionHealth>::SharedPtr health_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr fusion_timer_;
};

}  // namespace fusion
}  // namespace juppiter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<juppiter::fusion::FusionNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
