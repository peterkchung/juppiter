# About: Mode Manager
# Manages fusion mode transitions and handoff logic

#ifndef FUSION_CORE__MODE_MANAGER_HPP_
#define FUSION_CORE__MODE_MANAGER_HPP_

#include <string>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
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
 * @brief Manages mode transitions with hysteresis and safety checks
 * 
 * Ensures smooth handoffs between estimators with:
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
   * @brief Evaluate mode based on current health scores
   * @param lio_health LIO health score
   * @param vio_health VIO health score
   * @param kinematic_health Kinematic health score
   * @return Recommended mode (may not trigger immediate switch)
   */
  FusionMode evaluateMode(
    float lio_health,
    float vio_health,
    float kinematic_health);

  /**
   * @brief Check if mode switch should occur
   * Applies persistence rules and safety checks
   * @param candidate_mode Mode recommended by evaluation
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
  std::string modeToString(FusionMode mode) const;
  FusionMode stringToMode(const std::string & str) const;
  void notifyCallbacks(const ModeTransition & transition);
  void recordTransition(const ModeTransition & transition);
  bool checkDiscontinuityLimits(
    FusionMode from,
    FusionMode to);
};

}  // namespace fusion
}  // namespace juppiter

#endif  // FUSION_CORE__MODE_MANAGER_HPP_
