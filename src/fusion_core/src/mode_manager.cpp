// About: Mode Manager Implementation
// Mode transition logic with hysteresis and safety checks

#include "fusion_core/mode_manager.hpp"

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
  
  // TODO: Load YAML configuration
  
  RCLCPP_INFO(node_->get_logger(), 
    "ModeManager initialized (config: %s)", 
    config_path.c_str());
  
  initialized_ = true;
  return true;
}

void ModeManager::registerModeChangeCallback(ModeChangeCallback callback)
{
  callbacks_.push_back(callback);
}

FusionMode ModeManager::evaluateMode(
  float lio_health,
  float vio_health,
  float kinematic_health)
{
  // Store health scores
  last_health_scores_["lio"] = lio_health;
  last_health_scores_["vio"] = vio_health;
  last_health_scores_["kinematic"] = kinematic_health;
  
  // Determine appropriate mode
  float min_health = std::min({lio_health, vio_health, kinematic_health});
  
  if (min_health < 0.55f) {
    return FusionMode::SAFE_STOP;
  } else if (min_health < 0.75f) {
    // Determine which sensor is degraded
    if (lio_health < 0.75f) {
      return FusionMode::DEGRADED_LIO;
    } else if (vio_health < 0.75f) {
      return FusionMode::DEGRADED_VIO;
    } else {
      return FusionMode::DEGRADED_KINEMATIC;
    }
  }
  
  return FusionMode::NOMINAL;
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

void ModeManager::forceMode(FusionMode mode, const std::string & reason)
{
  RCLCPP_WARN(node_->get_logger(), 
    "Manual mode override: %s -> %s (reason: %s)",
    modeToString(current_mode_).c_str(),
    modeToString(mode).c_str(),
    reason.c_str());
  
  executeModeSwitch(mode, "MANUAL_OVERRIDE: " + reason);
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
