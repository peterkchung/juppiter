// About: Health Client
// ROS 2 service client for querying sensor_core health with caching
// Implements 100ms cache TTL and fallback strategies for edge compute

#ifndef FUSION_CORE__HEALTH_CLIENT_HPP_
#define FUSION_CORE__HEALTH_CLIENT_HPP_

#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "common_msgs/msg/perception_health.hpp"
#include "common_msgs/srv/get_perception_health.hpp"

namespace juppiter
{
namespace fusion
{

/**
 * @brief Health client with caching for on-demand sensor_core queries
 * 
 * Features:
 * - 100ms cache TTL (2 fusion cycles at 20Hz)
 * - Service timeout handling
 * - Configurable health decay on service failure
 * - Fallback to last known health with exponential decay
 */
class HealthClient
{
public:
  explicit HealthClient(rclcpp::Node::SharedPtr node);
  
  /**
   * @brief Initialize the health client
   * @param service_timeout_ms Timeout for service calls (default: 50ms)
   * @param cache_ttl_ms Cache time-to-live (default: 100ms)
   * @param health_decay_rate Rate of health decay per cycle on failure (default: 0.05)
   * @return true if initialization successful
   */
  bool initialize(
    double service_timeout_ms = 50.0,
    double cache_ttl_ms = 100.0,
    double health_decay_rate = 0.05);

  /**
   * @brief Get health with caching
   * @return Health if available, std::nullopt if cache expired and service unavailable
   */
  std::optional<common_msgs::msg::PerceptionHealth> getHealth();

  /**
   * @brief Get cached health without requesting fresh data
   * @return Last known health, even if stale
   */
  common_msgs::msg::PerceptionHealth getCachedHealth() const;

  /**
   * @brief Check if health service is available
   * @return true if service responded within timeout
   */
  bool isServiceAvailable() const;

  /**
   * @brief Get time since last successful health update
   * @return Time in seconds
   */
  double getHealthAgeSeconds() const;

  /**
   * @brief Force refresh of health data
   * @return true if refresh successful
   */
  bool refreshHealth();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<common_msgs::srv::GetPerceptionHealth>::SharedPtr client_;
  
  // Configuration
  double service_timeout_ms_{50.0};
  double cache_ttl_ms_{100.0};
  double health_decay_rate_{0.05};
  double max_health_age_s_{2.0};  // Safe stop after 2 seconds

  // Cache
  common_msgs::msg::PerceptionHealth cached_health_;
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  bool has_valid_cache_{false};

  // Request fresh health from service
  std::optional<common_msgs::msg::PerceptionHealth> requestHealth();
  
  // Apply decay to cached health
  void applyHealthDecay(common_msgs::msg::PerceptionHealth & health);
  
  // Fallback health when service unavailable
  common_msgs::msg::PerceptionHealth createFallbackHealth();
};

}  // namespace fusion
}  // namespace juppiter

#endif  // FUSION_CORE__HEALTH_CLIENT_HPP_
