// About: Health Client Implementation
// On-demand health queries with caching and fallback strategies

#include "fusion_core/health_client.hpp"

#include <chrono>

namespace juppiter
{
namespace fusion
{

HealthClient::HealthClient(rclcpp::Node::SharedPtr node)
: node_(node)
{
}

bool HealthClient::initialize(
  double service_timeout_ms,
  double cache_ttl_ms,
  double health_decay_rate)
{
  service_timeout_ms_ = service_timeout_ms;
  cache_ttl_ms_ = cache_ttl_ms;
  health_decay_rate_ = health_decay_rate;

  // Create service client
  client_ = node_->create_client<common_msgs::srv::GetPerceptionHealth>(
    "/perception/get_health");

  // Wait for service to be available (with timeout)
  if (!client_->wait_for_service(std::chrono::milliseconds(static_cast<int>(service_timeout_ms_ * 2)))) {
    RCLCPP_WARN(node_->get_logger(), 
      "Health service not available yet, will retry on demand");
  }

  RCLCPP_INFO(node_->get_logger(), 
    "HealthClient initialized (timeout: %.1fms, cache_ttl: %.1fms, decay: %.2f)",
    service_timeout_ms_, cache_ttl_ms_, health_decay_rate_);

  return true;
}

std::optional<common_msgs::msg::PerceptionHealth> HealthClient::getHealth()
{
  auto now = node_->now();
  
  // Check if cache is still valid
  if (has_valid_cache_) {
    auto age_ms = (now - last_update_time_).seconds() * 1000.0;
    if (age_ms < cache_ttl_ms_) {
      // Cache hit - return cached health
      return cached_health_;
    }
  }

  // Cache expired or invalid - request fresh health
  auto fresh_health = requestHealth();
  
  if (fresh_health.has_value()) {
    // Update cache
    cached_health_ = fresh_health.value();
    last_update_time_ = now;
    has_valid_cache_ = true;
    return fresh_health;
  }

  // Service call failed - use cached health with decay
  if (has_valid_cache_) {
    applyHealthDecay(cached_health_);
    return cached_health_;
  }

  // No cache available - return fallback
  return createFallbackHealth();
}

common_msgs::msg::PerceptionHealth HealthClient::getCachedHealth() const
{
  return cached_health_;
}

bool HealthClient::isServiceAvailable() const
{
  return client_->service_is_ready();
}

double HealthClient::getHealthAgeSeconds() const
{
  if (!has_valid_cache_) {
    return std::numeric_limits<double>::max();
  }
  return (node_->now() - last_update_time_).seconds();
}

bool HealthClient::refreshHealth()
{
  auto fresh_health = requestHealth();
  if (fresh_health.has_value()) {
    cached_health_ = fresh_health.value();
    last_update_time_ = node_->now();
    has_valid_cache_ = true;
    return true;
  }
  return false;
}

std::optional<common_msgs::msg::PerceptionHealth> HealthClient::requestHealth()
{
  if (!client_->service_is_ready()) {
    return std::nullopt;
  }

  auto request = std::make_shared<common_msgs::srv::GetPerceptionHealth::Request>();
  
  auto future = client_->async_send_request(request);
  
  // Wait with timeout
  auto status = future.wait_for(std::chrono::milliseconds(static_cast<int>(service_timeout_ms_)));
  
  if (status == std::future_status::ready) {
    try {
      auto response = future.get();
      return response->health;
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(node_->get_logger(), "Health service call failed: %s", e.what());
      return std::nullopt;
    }
  }

  // Timeout
  RCLCPP_DEBUG(node_->get_logger(), "Health service call timeout");
  return std::nullopt;
}

void HealthClient::applyHealthDecay(common_msgs::msg::PerceptionHealth & health)
{
  // Apply configurable decay rate
  float decay_factor = static_cast<float>(1.0 - health_decay_rate_);
  
  health.lio_health *= decay_factor;
  health.vio_health *= decay_factor;
  health.kinematic_health *= decay_factor;
  health.lidar_health *= decay_factor;
  health.camera_health *= decay_factor;
  health.imu_health *= decay_factor;
  
  // Recalculate overall
  health.overall_health = std::min(
    health.lio_health, 
    std::min(health.vio_health, health.kinematic_health));
  
  // Check if we need to enter safe_stop
  auto age_s = (node_->now() - last_update_time_).seconds();
  if (age_s > max_health_age_s_) {
    health.mode = common_msgs::msg::PerceptionHealth::MODE_SAFE_STOP;
    health.overall_health = 0.0f;
    health.fault_flags |= common_msgs::msg::PerceptionHealth::FAULT_SERVICE_UNAVAILABLE;
    health.service_responsive = false;
    health.mode_string = "safe_stop";
  }
}

common_msgs::msg::PerceptionHealth HealthClient::createFallbackHealth()
{
  common_msgs::msg::PerceptionHealth health;
  
  // Conservative fallback - assume degraded mode
  health.stamp = node_->now();
  health.overall_health = 0.5f;
  health.lio_health = 0.5f;
  health.vio_health = 0.5f;
  health.kinematic_health = 0.5f;
  health.lidar_health = 0.5f;
  health.camera_health = 0.5f;
  health.imu_health = 0.5f;
  health.time_sync_skew_ms = 0.0f;
  health.time_sync_valid = false;
  health.fault_flags = common_msgs::msg::PerceptionHealth::FAULT_SERVICE_UNAVAILABLE;
  health.mode = common_msgs::msg::PerceptionHealth::MODE_SAFE_STOP;
  health.mode_string = "safe_stop";
  health.service_responsive = false;
  
  return health;
}

}  // namespace fusion
}  // namespace juppiter
