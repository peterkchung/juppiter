# Fusion Core

The Fusion Core implements confidence-weighted multi-estimator fusion, combining outputs from LIO, VIO, and Kinematic estimators into a single robust pose estimate.

## Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Fusion Core                               │
│                                                                │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                    │
│  │  LIO     │  │  VIO     │  │ Kinetic  │                    │
│  │  Input   │  │  Input   │  │  Input   │                    │
│  │ /lio/odom│  │ /vio/odom│  │/kinematic│                    │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘                    │
│       │            │            │                            │
│       └────────────┼────────────┘                            │
│                    ▼                                          │
│       ┌──────────────────────────────┐                       │
│       │    Weight Calculation        │                       │
│       │  • Health-based weights      │                       │
│       │  • Consistency checks        │                       │
│       │  • Dynamic adjustment       │                       │
│       └──────────────┬───────────────┘                       │
│                      ▼                                        │
│       ┌──────────────────────────────┐                       │
│       │   Confidence-Weighted EKF    │                       │
│       │  State: [x,y,z,r,p,y,vx,vy, │                       │
│       │         vz,wx,wy,wz]       │                       │
│       └──────────────┬───────────────┘                       │
│                      ▼                                        │
│       ┌──────────────────────────────┐                       │
│       │   Output: /perception/odom   │                       │
│       └──────────────────────────────┘                       │
└─────────────────────────────────────────────────────────────┘
```

## Estimator Inputs

The Fusion Core subscribes to three odometry sources:

| Topic | Source | Typical Rate | Accuracy |
|-------|--------|--------------|----------|
| `/lio/odom` | FAST-LIO2 or DLIO | 10-50 Hz | High (cm-level) |
| `/vio/odom` | ORB-SLAM3 or OpenVINS | 10-30 Hz | Medium (dm-level) |
| `/kinematic/odom` | robot_localization EKF | 50 Hz | Baseline (m-level) |

## Fusion Algorithm

### Confidence-Weighted EKF

The Extended Kalman Filter (EKF) fuses estimates by weighting each by its confidence:

```
State Vector (13 elements):
  x = [px, py, pz,       // Position (m)
       roll, pitch, yaw, // Orientation (rad)
       vx, vy, vz,       // Linear velocity (m/s)
       wx, wy, wz]       // Angular velocity (rad/s)

Measurement Update:
  x_fused = x_pred + K * (z - h(x_pred))
  
  where:
    z = measurement from estimator
    h() = observation model
    K = Kalman gain (incorporates confidence)
```

### Weight Calculation

Base weights are defined in configuration:

```yaml
fusion:
  weights_nominal:
    lio: 0.50      # Primary estimator
    vio: 0.35      # Secondary estimator  
    kinetic: 0.15  # Baseline fallback
```

**Dynamic Weight Adjustment:**

```cpp
// Adjust based on health
if (lio_health < 0.5) {
    w_lio *= 0.5;      // Reduce LIO influence
    w_vio *= 1.3;      // Increase VIO
    w_kinetic *= 1.2;  // Increase kinetic
}

// Normalize to sum to 1.0
double sum = w_lio + w_vio + w_kinetic;
w_lio /= sum;
w_vio /= sum;
w_kinetic /= sum;
```

### Consistency Check

Before fusion, each estimator is checked for consistency:

```cpp
// Mahalanobis distance test
// If estimator diverges significantly from others, reduce weight

double mahalanobis_dist = 
    sqrt((z - x_pred).transpose() * S.inverse() * (z - x_pred));

if (mahalanobis_dist > CONSISTENCY_THRESHOLD) {
    weight *= 0.5;  // Outlier - reduce influence
    log_warning("Estimator inconsistent: " + estimator_name);
}
```

## State Estimation

### Prediction Step (Motion Model)

```
State prediction using IMU data:
  x_k|k-1 = f(x_k-1, u_k)
  
  where u = [ax, ay, az, wx, wy, wz]  // IMU measurements

Prediction equations:
  p_k = p_k-1 + v_k-1 * dt + 0.5 * R(q_k-1) * a * dt^2
  q_k = q_k-1 ⊗ exp(0.5 * w * dt)       // Quaternion integration
  v_k = v_k-1 + R(q_k-1) * a * dt
```

### Update Step (Measurement Fusion)

```
When odometry message received from any estimator:
  
  // Compute innovation
  y = z - h(x_pred)
  
  // Compute innovation covariance
  S = H * P_pred * H' + R
  where R = measurement covariance scaled by 1/weight
  
  // Compute Kalman gain
  K = P_pred * H' * S^-1
  
  // Update state
  x = x_pred + K * y
  
  // Update covariance
  P = (I - K * H) * P_pred
```

## Mode-Aware Fusion

Different modes use different fusion strategies:

### NOMINAL Mode

All estimators healthy, standard weighted fusion:

```
Weights: LIO=0.50, VIO=0.35, Kinetic=0.15
Strategy: Full confidence-weighted EKF
```

### DEGRADED_LIO Mode

LiDAR issues, rely more on VIO:

```
Weights: LIO=0.20, VIO=0.65, Kinetic=0.15
Strategy: VIO as primary, LIO has reduced influence
Additional: Increase innovation gate for LIO (more tolerant to errors)
```

### DEGRADED_VIO Mode

Camera issues, rely more on LIO:

```
Weights: LIO=0.70, VIO=0.15, Kinetic=0.15
Strategy: LIO dominant, VIO backup only
Additional: Monitor LIO covariance closely
```

### SAFE_STOP Mode

Critical failure, stop using all estimators:

```
Weights: All = 0.0
Strategy: Publish last known good pose with high uncertainty
Action: Signal navigation stack to initiate safe stop
```

## Configuration

Fusion parameters in `config/fusion/weights_nominal.yaml`:

```yaml
fusion_core:
  # Nominal weights (sum should equal 1.0)
  weights:
    lio: 0.50
    vio: 0.35
    kinetic: 0.15
  
  # Dynamic adjustment limits
  min_weight: 0.05       # Never go below 5%
  max_weight: 0.90       # Never exceed 90%
  
  # Consistency check
  consistency_threshold: 3.0  # Mahalanobis distance
  outlier_rejection: true
  
  # Covariance scaling based on weights
  covariance_scale:
    high_confidence: 0.5   # Weight > 0.6
    medium_confidence: 1.0 # Weight 0.3-0.6
    low_confidence: 2.0    # Weight < 0.3
  
  # Process noise
  process_noise:
    position: 0.01     # m^2/s
    orientation: 0.001 # rad^2/s
    velocity: 0.1      # (m/s)^2/s
    angular_vel: 0.01  # (rad/s)^2/s
```

## Implementation Details

### Node Structure

```cpp
class FusionNode : public rclcpp::Node {
private:
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kinetic_sub_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr weights_pub_;
    
    // Core algorithm
    std::unique_ptr<ConfidenceFusionEKF> fusion_ekf_;
    
    // Health monitoring
    std::shared_ptr<HealthMonitor> health_monitor_;
    ModeManager mode_manager_;
    
    // Weights
    FusionWeights current_weights_;
};
```

### Callback Flow

```cpp
// When LIO publishes new odometry
void lioCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 1. Check LIO health
    double lio_health = health_monitor_->getHealth("lio");
    
    // 2. Calculate weight based on health
    double weight = calculateWeight(lio_health, base_weight_lio_);
    
    // 3. Consistency check
    if (!checkConsistency(*msg)) {
        weight *= 0.5;
        log_inconsistency("lio");
    }
    
    // 4. Update EKF
    fusion_ekf_->updateFromLIO(*msg, weight);
    
    // 5. Publish fused result
    publishFusedOdometry();
}
```

## Output Topics

The Fusion Core publishes:

```bash
# Main output: fused pose estimate
ros2 topic echo /perception/odom

# Type: nav_msgs/Odometry
# Contains:
#   - pose.position (x, y, z)
#   - pose.orientation (quaternion)
#   - twist (linear and angular velocity)
#   - covariance matrices

# Current weights
ros2 topic echo /perception/estimator_weights

# Example output (JSON):
{
  "lio": 0.50,
  "vio": 0.35,
  "kinetic": 0.15,
  "mode": "nominal",
  "timestamp": "2024-01-15T10:30:45.123Z"
}
```

## Performance Metrics

Based on lunar simulation benchmarks:

| Scenario | Position RMSE | Orientation RMSE | Latency |
|----------|---------------|------------------|---------|
| All Healthy | 0.08m | 1.2° | 25ms |
| LIO Degraded | 0.12m | 1.8° | 25ms |
| VIO Degraded | 0.10m | 1.5° | 25ms |
| LIO Failed | 0.18m | 2.5° | 25ms |

## Tuning Guide

### When to Adjust Weights

**Increase LIO weight if:**
- Operating in textureless environments (few visual features)
- High dust/scatter affecting cameras
- Need highest accuracy

**Increase VIO weight if:**
- Operating in feature-rich environments
- LiDAR has limited range/FOV
- LiDAR experiencing dropouts

**Increase Kinetic weight if:**
- Both LIO and VIO are degraded
- Operating at very slow speeds
- Need continuity during temporary outages

### Parameter Tuning

Edit `config/fusion/weights_nominal.yaml`:

```yaml
# Example: Vision-dominant configuration
weights:
  lio: 0.30      # Reduce LiDAR
  vio: 0.60      # Increase vision
  kinetic: 0.10  # Minimal kinetic

# Example: LiDAR-dominant configuration  
weights:
  lio: 0.70      # High LiDAR
  vio: 0.15      # Minimal vision
  kinetic: 0.15
```

## Next Topics

- [Sensor Core](sensor-core.md) - How health affects fusion
- [Configuration Guide](../user-guides/configuration.md) - Tuning parameters
- [FDIIR Testing](../user-guides/fdiir-testing.md) - Validation scenarios
