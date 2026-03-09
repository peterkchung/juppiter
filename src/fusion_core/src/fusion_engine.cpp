// About: Fusion Engine Implementation
// Confidence-weighted multi-estimator fusion

#include "fusion_core/fusion_engine.hpp"

namespace juppiter
{
namespace fusion
{

FusionEngine::FusionEngine()
{
  // Default initialization - will be overwritten by initialize()
}

bool FusionEngine::initialize(
  rclcpp::Node::SharedPtr node,
  const std::string & config_path)
{
  node_ = node;
  
  // TODO: Load YAML configuration from config_path
  // For now, use default values
  
  RCLCPP_INFO(node_->get_logger(), 
    "FusionEngine initialized with defaults (config: %s)", 
    config_path.c_str());
  
  initialized_ = true;
  return true;
}

void FusionEngine::updateLioInput(const lio::LioHealthStatus & lio_health)
{
  lio_input_.health_score = lio_health.health_score;
  lio_input_.covariance_norm = lio_health.covariance_norm;
  lio_input_.source = "lio";
  lio_input_.timestamp = node_->now();
}

void FusionEngine::updateVioInput(const vio::VioHealthStatus & vio_health)
{
  vio_input_.health_score = vio_health.health_score;
  vio_input_.covariance_norm = vio_health.covariance_norm;
  vio_input_.source = "vio";
  vio_input_.timestamp = node_->now();
}

void FusionEngine::updateKinematicInput(
  const nav_msgs::msg::Odometry::SharedPtr & odometry,
  float covariance_norm)
{
  kinematic_input_.odometry = *odometry;
  kinematic_input_.covariance_norm = covariance_norm;
  kinematic_input_.source = "kinematic";
  kinematic_input_.timestamp = node_->now();
}

FusedOdometry FusionEngine::computeFusion()
{
  FusedOdometry result;
  
  // Compute weights for each input
  float w_lio = computeWeight(lio_input_);
  float w_vio = computeWeight(vio_input_);
  float w_kinematic = computeWeight(kinematic_input_);
  
  // Apply mode-specific weight adjustments
  applyModeWeights(current_mode_);
  
  // TODO: Implement actual pose interpolation
  // For now, return placeholder
  result.mode = current_mode_;
  result.overall_health = (lio_input_.health_score * w_lio + 
                           vio_input_.health_score * w_vio + 
                           kinematic_input_.health_score * w_kinematic) / 
                          (w_lio + w_vio + w_kinematic + epsilon_);
  
  result.source_weights["lio"] = w_lio;
  result.source_weights["vio"] = w_vio;
  result.source_weights["kinematic"] = w_kinematic;
  
  return result;
}

FusionMode FusionEngine::getCurrentMode() const
{
  return current_mode_;
}

std::string FusionEngine::getModeString() const
{
  switch (current_mode_) {
    case FusionMode::NOMINAL: return "nominal";
    case FusionMode::DEGRADED_LIO: return "degraded_lio";
    case FusionMode::DEGRADED_VIO: return "degraded_vio";
    case FusionMode::DEGRADED_KINEMATIC: return "degraded_kinematic";
    case FusionMode::SAFE_STOP: return "safe_stop";
    default: return "unknown";
  }
}

void FusionEngine::reset()
{
  // Reset all inputs
  lio_input_ = OdometryInput();
  vio_input_ = OdometryInput();
  kinematic_input_ = OdometryInput();
  
  // Reset mode
  current_mode_ = FusionMode::NOMINAL;
  mode_persistence_counter_ = 0;
}

bool FusionEngine::isFusionReady() const
{
  // Need at least one valid input
  return (lio_input_.health_score > 0.0f || 
          vio_input_.health_score > 0.0f || 
          kinematic_input_.health_score > 0.0f);
}

std::map<std::string, float> FusionEngine::getSourceWeights() const
{
  return {
    {"lio", smoothed_lio_weight_},
    {"vio", smoothed_vio_weight_},
    {"kinematic", smoothed_kinematic_weight_}
  };
}

float FusionEngine::computeWeight(const OdometryInput & input)
{
  if (input.covariance_norm < epsilon_) {
    return input.health_score / epsilon_;
  }
  return input.health_score / input.covariance_norm;
}

FusionMode FusionEngine::determineMode()
{
  // Evaluate health scores and determine appropriate mode
  // TODO: Implement full logic with hysteresis
  
  float min_health = std::min({lio_input_.health_score, 
                               vio_input_.health_score, 
                               kinematic_input_.health_score});
  
  if (min_health < degraded_threshold_) {
    return FusionMode::SAFE_STOP;
  } else if (min_health < nominal_threshold_) {
    // Determine which sensor is degraded
    if (lio_input_.health_score < nominal_threshold_) {
      return FusionMode::DEGRADED_LIO;
    } else if (vio_input_.health_score < nominal_threshold_) {
      return FusionMode::DEGRADED_VIO;
    } else {
      return FusionMode::DEGRADED_KINEMATIC;
    }
  }
  
  return FusionMode::NOMINAL;
}

bool FusionEngine::shouldSwitchMode(FusionMode new_mode)
{
  if (new_mode == current_mode_) {
    mode_persistence_counter_ = 0;
    return false;
  }
  
  mode_persistence_counter_++;
  if (mode_persistence_counter_ >= cycles_to_confirm_) {
    mode_persistence_counter_ = 0;
    return true;
  }
  
  return false;
}

void FusionEngine::applyModeWeights(FusionMode mode)
{
  // Apply mode-specific weight adjustments
  // TODO: Load from configuration
}

float FusionEngine::exponentialMovingAverage(float current, float previous, float alpha)
{
  return alpha * current + (1.0f - alpha) * previous;
}

geometry_msgs::msg::Pose FusionEngine::interpolatePoses(
  const geometry_msgs::msg::Pose & from,
  const geometry_msgs::msg::Pose & to,
  float t)
{
  // TODO: Implement proper pose interpolation (SLERP for orientation)
  geometry_msgs::msg::Pose result;
  return result;
}

}  // namespace fusion
}  // namespace juppiter
