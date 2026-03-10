// About: Mode Manager
// Single authority for fusion mode determination and transitions

#ifndef FUSION_CORE__MODE_MANAGER_HPP_
#define FUSION_CORE__MODE_MANAGER_HPP_

#include <string>
#include <vector>
#include <functional>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "common_msgs/msg/perception_health.hpp"
#include "fusion_core/fusion_engine.hpp"

namespace juppiter
{
namespace fusion
{

/**
 * @brief Mode transition event for callbacks
 */
struct ModeTransition
{
  FusionMode from;
  FusionMode to;
  rclcpp::Time timestamp;
  std::string reason;
  std::map<std::string, float> health_scores;
};

/**
 * @brief Callback type for mode change events
 */
using ModeChangeCallback = std::function<void(const ModeTransition &)>;

/**
 * @brief Single authority for mode determination with hysteresis and safety checks
 * 
 * Consolidates all mode determination logic:
 * - Nominal: all estimators healthy (overall_health >= 0.75)
 * - Degraded LIO: LIO health < 0.75 (VIO becomes primary)
 * - Degraded VIO: VIO health < 0.75 (LIO becomes primary)
 * - Safe Stop: overall health < 0.55 or max age exceeded
 * 
 * Features:
 * - Persistence counters to prevent mode flutter
 * - Covariance inflation during transitions
 * - Discontinuity limits (position/yaw jumps)
 */
class ModeManager
{
public:
  ModeManager();
  ~ModeManager() = default;

  /**
   * @brief Initialize mode manager
   * @param node ROS node for parameter access
   * @param config_path Path to YAML configuration
   * @return true if initialization successful
   */
  bool initialize(
    rclcpp::Node::SharedPtr node,
    const std::string & config_path);

  /**
   * @brief Register callback for mode transitions
   * @param callback Function to call on mode change
   */
  void registerModeChangeCallback(ModeChangeCallback callback);

  /**
   * @brief Determine mode from perception health (single authority)
   * 
   * This is THE ONLY function that should determine system mode.
   * All mode decisions go through here.
   * 
   * @param health Complete health status from sensor_core
   * @return Authoritative mode for the system
   */
  FusionMode determineMode(const common_msgs::msg::PerceptionHealth & health);

  /**
   * @brief Check if mode switch should occur
   * Applies persistence rules and safety checks
   * @param candidate_mode Mode recommended by determineMode
   * @return true if switch should happen now
   */
  bool shouldSwitchMode(FusionMode candidate_mode);

  /**
   * @brief Execute mode switch
   * Triggers callbacks and prepares transition state
   * @param new_mode Mode to switch to
   * @param reason Human-readable reason for switch
   */
  void executeModeSwitch(FusionMode new_mode, const std::string & reason);

  /**
   * @brief Get current mode
   * @return Current operating mode
   */
  FusionMode getCurrentMode() const;

  /**
   * @brief Check if currently in transition
   * @return true if within transition window
   */
  bool isInTransition() const;

  /**
   * @brief Get transition progress (0.0 to 1.0)
   * @return Progress through current transition
   */
  float getTransitionProgress() const;

  /**
   * @brief Update transition state (call periodically)
   * @param dt Time delta since last update
   */
  void updateTransition(float dt);

  /**
   * @brief Get transition history
   * @return List of recent mode transitions
   */
  std::vector<ModeTransition> getTransitionHistory() const;

  /**
   * @brief Convert mode to string for topic publishing
   * @param mode Fusion mode
   * @return String representation ("nominal", "degraded_lio", etc.)
   */
  std::string modeToString(FusionMode mode) const;

  /**
   * @brief Convert string to mode
   * @param str Mode string
   * @return FusionMode value
   */
  FusionMode stringToMode(const std::string & str) const;

  /**
   * @brief Force mode (for manual override/testing)
   * @param mode Mode to force
   * @param reason Override reason
   */
  void forceMode(FusionMode mode, const std::string & reason);

private:
  rclcpp::Node::SharedPtr node_;
  bool initialized_{false};

  // Current state
  FusionMode current_mode_{FusionMode::NOMINAL};
  FusionMode candidate_mode_{FusionMode::NOMINAL};
  int persistence_counter_{0};
  
  // Health thresholds (single source of truth)
  float nominal_threshold_{0.75f};
  float degraded_threshold_{0.55f};
  float safe_stop_threshold_{0.55f};
  
  // Configuration
  int cycles_to_confirm_{3};
  float max_position_jump_m_{0.30f};
  float max_yaw_jump_deg_{5.0f};
  float transition_duration_sec_{2.0f};
  float covariance_inflation_factor_{2.0f};
  
  // Transition state
  bool in_transition_{false};
  float transition_progress_{0.0f};
  rclcpp::Time transition_start_time_;
  
  // Callbacks
  std::vector<ModeChangeCallback> callbacks_;
  
  // History
  std::vector<ModeTransition> transition_history_;
  size_t max_history_size_{100};
  
  // Health scores at last evaluation
  std::map<std::string, float> last_health_scores_;

  // Private methods
  void notifyCallbacks(const ModeTransition & transition);
  void recordTransition(const ModeTransition & transition);
  bool checkDiscontinuityLimits(
    FusionMode from,
    FusionMode to);
  FusionMode determineDegradedMode(
    float lio_health,
    float vio_health,
    float kinematic_health);
};

}  // namespace fusion
}  // namespace juppiter

#endif  // FUSION_CORE__MODE_MANAGER_HPP_
