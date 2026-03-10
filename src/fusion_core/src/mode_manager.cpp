// About: Mode Manager Implementation
// Single authority for mode determination with hysteresis and safety checks

#include "fusion_core/mode_manager.hpp"

#include <algorithm>

namespace juppiter
{
namespace fusion
{

ModeManager::ModeManager()
{
}

bool ModeManager::initialize(
  rclcpp::Node::SharedPtr node,
  const std::string & config_path)
{
  node_ = node;
  
  // TODO: Load YAML configuration for thresholds
  // nominal_threshold_ = config["nominal_threshold"];
  // degraded_threshold_ = config["degraded_threshold"];
  // safe_stop_threshold_ = config["safe_stop_threshold"];
  
  RCLCPP_INFO(node_->get_logger(), 
    "ModeManager initialized (single authority for mode decisions)");
  
  initialized_ = true;
  return true;
}

void ModeManager::registerModeChangeCallback(ModeChangeCallback callback)
{
  callbacks_.push_back(callback);
}

FusionMode ModeManager::determineMode(const common_msgs::msg::PerceptionHealth & health)
{
  // Store health scores for callbacks
  last_health_scores_["lio"] = health.lio_health;
  last_health_scores_["vio"] = health.vio_health;
  last_health_scores_["kinematic"] = health.kinematic_health;
  last_health_scores_["lidar"] = health.lidar_health;
  last_health_scores_["camera"] = health.camera_health;
  last_health_scores_["imu"] = health.imu_health;
  last_health_scores_["overall"] = health.overall_health;
  
  // Single authority for mode determination
  // Check safe_stop first (highest priority)
  if (health.overall_health < safe_stop_threshold_) {
    return FusionMode::SAFE_STOP;
  }
  
  // Check for degraded modes
  if (health.overall_health < nominal_threshold_) {
    // Determine which estimator is degraded
    return determineDegradedMode(
      health.lio_health, 
      health.vio_health, 
      health.kinematic_health);
  }
  
  // Check for critical sensor faults that should trigger safe_stop
  if (health.fault_flags & common_msgs::msg::PerceptionHealth::FAULT_SERVICE_UNAVAILABLE) {
    // Service timeout - check if we've exceeded max age
    // This is handled by health_client with decay, but double-check here
    if (health.overall_health < safe_stop_threshold_) {
      return FusionMode::SAFE_STOP;
    }
  }
  
  // All clear - nominal mode
  return FusionMode::NOMINAL;
}

FusionMode ModeManager::determineDegradedMode(
  float lio_health,
  float vio_health,
  float kinematic_health)
{
  // Priority order for degraded mode selection:
  // 1. If LIO is degraded -> DEGRADED_LIO (VIO becomes primary)
  // 2. If VIO is degraded -> DEGRADED_VIO (LIO becomes primary)
  // 3. If both are ok but overall degraded -> check which is worse
  
  bool lio_degraded = lio_health < degraded_threshold_;
  bool vio_degraded = vio_health < degraded_threshold_;
  bool kinematic_degraded = kinematic_health < degraded_threshold_;
  
  if (lio_degraded && !vio_degraded) {
    // LIO bad, VIO good -> use VIO primarily
    return FusionMode::DEGRADED_LIO;
  }
  
  if (vio_degraded && !lio_degraded) {
    // VIO bad, LIO good -> use LIO primarily
    return FusionMode::DEGRADED_VIO;
  }
  
  if (lio_degraded && vio_degraded) {
    // Both LIO and VIO degraded
    if (!kinematic_degraded) {
      // At least kinematic is working - rely on that
      return FusionMode::DEGRADED_KINEMATIC;
    } else {
      // Everything degraded -> safe_stop
      return FusionMode::SAFE_STOP;
    }
  }
  
  // Neither is below threshold individually, but overall is degraded
  // Use the healthier one as primary
  if (lio_health >= vio_health) {
    return FusionMode::DEGRADED_VIO;  // VIO is worse, so mark as degraded_vio
  } else {
    return FusionMode::DEGRADED_LIO;  // LIO is worse, so mark as degraded_lio
  }
}

bool ModeManager::shouldSwitchMode(FusionMode candidate_mode)
{
  if (candidate_mode == current_mode_) {
    persistence_counter_ = 0;
    return false;
  }
  
  persistence_counter_++;
  if (persistence_counter_ >= cycles_to_confirm_) {
    persistence_counter_ = 0;
    return checkDiscontinuityLimits(current_mode_, candidate_mode);
  }
  
  return false;
}

void ModeManager::executeModeSwitch(FusionMode new_mode, const std::string & reason)
{
  ModeTransition transition;
  transition.from = current_mode_;
  transition.to = new_mode;
  transition.timestamp = node_->now();
  transition.reason = reason;
  transition.health_scores = last_health_scores_;
  
  RCLCPP_INFO(node_->get_logger(), 
    "Mode transition: %s -> %s (reason: %s)",
    modeToString(current_mode_).c_str(),
    modeToString(new_mode).c_str(),
    reason.c_str());
  
  // Update state
  current_mode_ = new_mode;
  
  // Record and notify
  recordTransition(transition);
  notifyCallbacks(transition);
  
  // Begin transition period
  in_transition_ = true;
  transition_start_time_ = node_->now();
  transition_progress_ = 0.0f;
}

FusionMode ModeManager::getCurrentMode() const
{
  return current_mode_;
}

bool ModeManager::isInTransition() const
{
  return in_transition_;
}

float ModeManager::getTransitionProgress() const
{
  return transition_progress_;
}

void ModeManager::updateTransition(float dt)
{
  if (!in_transition_) {
    return;
  }
  
  // Update progress
  transition_progress_ += dt / transition_duration_sec_;
  
  if (transition_progress_ >= 1.0f) {
    transition_progress_ = 1.0f;
    in_transition_ = false;
    
    RCLCPP_INFO(node_->get_logger(), 
      "Mode transition complete: now in %s mode",
      modeToString(current_mode_).c_str());
  }
}

std::vector<ModeTransition> ModeManager::getTransitionHistory() const
{
  return transition_history_;
}

std::string ModeManager::modeToString(FusionMode mode) const
{
  switch (mode) {
    case FusionMode::NOMINAL: return "nominal";
    case FusionMode::DEGRADED_LIO: return "degraded_lio";
    case FusionMode::DEGRADED_VIO: return "degraded_vio";
    case FusionMode::DEGRADED_KINEMATIC: return "degraded_kinematic";
    case FusionMode::SAFE_STOP: return "safe_stop";
    default: return "unknown";
  }
}

FusionMode ModeManager::stringToMode(const std::string & str) const
{
  if (str == "nominal") return FusionMode::NOMINAL;
  if (str == "degraded_lio") return FusionMode::DEGRADED_LIO;
  if (str == "degraded_vio") return FusionMode::DEGRADED_VIO;
  if (str == "degraded_kinematic") return FusionMode::DEGRADED_KINEMATIC;
  if (str == "safe_stop") return FusionMode::SAFE_STOP;
  return FusionMode::NOMINAL;
}

void ModeManager::forceMode(FusionMode mode, const std::string & reason)
{
  RCLCPP_WARN(node_->get_logger(), 
    "Manual mode override: %s -> %s (reason: %s)",
    modeToString(current_mode_).c_str(),
    modeToString(mode).c_str(),
    reason.c_str());
  
  executeModeSwitch(mode, "MANUAL_OVERRIDE: " + reason);
}

void ModeManager::notifyCallbacks(const ModeTransition & transition)
{
  for (auto & callback : callbacks_) {
    callback(transition);
  }
}

void ModeManager::recordTransition(const ModeTransition & transition)
{
  transition_history_.push_back(transition);
  
  // Limit history size
  if (transition_history_.size() > max_history_size_) {
    transition_history_.erase(transition_history_.begin());
  }
}

bool ModeManager::checkDiscontinuityLimits(FusionMode from, FusionMode to)
{
  // TODO: Implement actual discontinuity checking
  // For now, allow all transitions
  return true;
}

}  // namespace fusion
}  // namespace juppiter
