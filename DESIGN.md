# Design — juppiter

Architecture and design decisions for the **Fault-Tolerant Multi-Sensor Perception System** with FDIIR (Fault Detection, Isolation, Identification, Recovery) principles and confidence-weighted sensor fusion.

## High-Level Architecture

Multi-sensor perception stack with plugin-based provider architecture and integrated FDIIR:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    sensor_core (FDIIR orchestrator)                         │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐            │
│  │ FaultDetector    │  │ FaultIsolator    │  │ RecoveryManager  │            │
│  │ (health + sync)  │  │ (mode manager)   │  │ (degradation)    │            │
│  └──────────────────┘  └──────────────────┘  └──────────────────┘            │
│                                                                              │
│  Topics: /perception/health, /perception/mode, /perception/faults          │
└─────────────────────────────────────────────────────────────────────────────┘
                            │
         ┌──────────────────┼──────────────────┐
         │                  │                  │
┌───────▼──────┐  ┌────────▼──────┐  ┌───────▼──────┐
│euroc_provider│  │lidar_provider │  │other_provider│
│ (EuRoC data) │  │ (real lidar) │  │  (future)    │
└──────────────┘  └──────────────┘  └──────────────┘
         │                  │
         ▼                  ▼
 /stereo/left/       /lidar/points
 /stereo/right/          │
 /imu/data                │
     │                    │
     └────────┬───────────┘
              ▼
     ┌────────────────────┐
     │  fusion_core       │
     │  ┌──────────────┐  │
     │  │ Estimator A  │  │  (LIO - LiDAR-Inertial)
     │  │ Estimator B  │  │  (VIO - Visual-Inertial)
     │  │ Estimator C  │  │  (INS - Pure IMU fallback)
     │  └──────────────┘  │
     │  Confidence Fusion │
     └────────────────────┘
              │
     ┌────────▼────────┐
     │/perception/odom  │
     │/perception/faults │
     └───────────────────┘
```

## Package Architecture

### sensor_interfaces (Interface Layer)

**Purpose:** Header-only package defining contracts for all sensor implementations with FDIIR support.

**Key Components:**
- `SensorDriver` — Abstract base class for all sensor providers
- `SensorCapability` — Capability discovery flags (COLOR_IMAGE, LIDAR_POINTS, etc.)
- `HealthStatus` — Standardized health reporting with fault classification
- `TimeSyncStatus` — Time synchronization monitoring data
- `SensorCalibration` — Calibration data structures with versioning
- `FaultFlags` — Standardized fault classification (STALE, SYNC_VIOLATION, COVARIANCE_EXCEEDED, etc.)

**Design Principles:**
- Pure interfaces, no implementation
- No ROS dependencies in base interface (only rclcpp::Node for initialization)
- Versioned calibration artifacts support
- Extensible fault classification system

### sensor_core (Orchestration Layer)

**Purpose:** Central coordinator that loads providers, monitors health, and manages time sync.

**Key Components:**

**1. SensorBridgeNode**
- Main ROS 2 node
- Loads providers via `pluginlib::ClassLoader`
- Manages provider lifecycle (initialize → start → stop → shutdown)
- Runs monitoring loop at configurable rate (default: 10Hz)
- Publishes `/perception/health` and `/perception/mode`

**2. TimeSynchronizer**
- Registers all sensors for timestamp monitoring
- Computes inter-sensor skew (mean, p95, max)
- Reports sync status against PRD thresholds (5ms mean, 10ms p95)
- Thread-safe operations with mutex protection

**3. HealthMonitor**
- Aggregates health from all registered drivers
- Computes overall health score per PRD formula
- Determines system mode:
  - `nominal` (health ≥ 0.75)
  - `degraded_lidar` or `degraded_vision` (0.55 ≤ health < 0.75)
  - `safe_stop` (health < 0.55)

### euroc_provider (Provider Example)

**Purpose:** Concrete SensorDriver implementation for EuRoC dataset replay with simulated fault injection.

**Structure:**
- `EurocDriver` — Implements `SensorDriver` interface
- `EurocReader` — Pure C++ CSV parsing and image loading (no ROS)
- `StereoDepth` — Pure C++ stereo rectification and depth computation (no ROS)
- `FaultInjector` — Simulates sensor faults for FDIIR testing

**Topics Published:**
- `/stereo/left/image_raw` — Left camera (mono8)
- `/stereo/right/image_raw` — Right camera (mono8)
- `/camera/depth/image_rect_raw` — Stereo depth (32FC1)
- `/imu/data` — IMU measurements
- `/camera/color/camera_info` — Camera intrinsics

## FDIIR Architecture

The system implements Fault Detection, Isolation, Identification, and Recovery following aerospace-grade principles adapted for lunar robotics.

### Fault Detection Layer

**Health Monitor (Active Detection):**
```cpp
struct HealthStatus {
  bool is_healthy;
  float health_score;              // 0.0 to 1.0
  std::vector<std::string> active_faults;
  // FAULT_STALE: data age > threshold
  // FAULT_SYNC_VIOLATION: inter-sensor skew > threshold  
  // FAULT_COVARIANCE_EXCEEDED: pose uncertainty > threshold
  // FAULT_RATE_DROP: actual rate < expected rate
};
```

**Detection Methods:**
1. **Temporal Monitoring** — Staleness detection (configurable per sensor type)
   - Camera: >100ms without frame
   - IMU: >50ms without sample
   - Lidar: >200ms without scan

2. **Synchronization Monitoring** — Time skew detection via `TimeSynchronizer`
   - Target: ≤5ms mean skew across all sensors
   - Warning: >10ms p95 skew
   - Critical: >50ms max skew triggers isolation

3. **Quality Monitoring** — Statistical anomaly detection
   - Covariance norm monitoring
   - Mahalanobis distance for outlier detection
   - Innovation sequence analysis (future)

4. **Cross-Validation** — Sensor-to-sensor consistency checks
   - Stereo pair validation (epipolar constraint check)
   - IMU-Lidar consistency (prediction vs measurement)
   - Vision-Lidar geometric consistency

### Fault Isolation Layer

**Mode Determination:**
The `FaultIsolator` classifies faults into operational modes:

| Mode | Condition | Active Sensors | Action |
|------|-----------|------------------|---------|
| `nominal` | health ≥ 0.75 | All sensors | Full fusion |
| `degraded_vision` | 0.55 ≤ health < 0.75, vision failing | Lidar + IMU | LIO priority |
| `degraded_lidar` | 0.55 ≤ health < 0.75, lidar failing | Vision + IMU | VIO priority |
| `degraded_imu` | 0.55 ≤ health < 0.75, IMU failing | Vision + Lidar | Visual-only + warn |
| `safe_stop` | health < 0.55 | Minimum viable | Initiate stop sequence |

**Fault Classification:**
```cpp
enum class FaultType {
  // Detection level
  SENSOR_OFFLINE,           // No data received
  SENSOR_STALE,             // Data too old
  
  // Isolation level
  SYNC_VIOLATION,           // Time sync failure
  QUALITY_DEGRADED,         // Covariance too high
  RATE_DEGRADED,            // Publishing slower than expected
  
  // Identification level
  CALIBRATION_DRIFT,        // Extrinsic mismatch detected
  ENVIRONMENT_DEGRADED,     // Extreme lighting/dust
  HARDWARE_FAULT            // Physical sensor failure
};
```

### Fault Identification Layer

**Root Cause Analysis:**
The system attempts to identify the underlying cause:

1. **Sensor-Specific Diagnosis:**
   - Camera: Frame drops, blur detection, exposure issues
   - Lidar: Point cloud density, return intensity
   - IMU: Bias drift, saturation detection

2. **Environmental Factors:**
   - Lighting changes (vision degradation in shadows)
   - Dust/debris (lidar multi-return analysis)
   - Thermal effects (IMU temperature compensation)

3. **Calibration Health:**
   - Online extrinsic verification via hand-eye calibration
   - Intrinsic drift monitoring
   - Version mismatch detection

**Fault Injection for Testing:**
```cpp
// EurocProvider supports fault simulation
fault_injector_.configure({
  .drop_rate = 0.1,        // Drop 10% of frames
  .delay_ms = 50,          // Add 50ms latency
  .noise_sigma = 0.5       // Add Gaussian noise
});
```

### Recovery Layer

**Recovery Strategies:**

1. **Graceful Degradation:**
   - Automatic mode switching based on available sensors
   - Confidence-weighted fusion adjustment
   - Reduced update rates in degraded modes

2. **Sensor Reinitialization:**
   - Automatic reconnection attempts for transient faults
   - Calibration re-verification on reconnection
   - Warm-start from last known good pose

3. **Estimator Handoff:**
   - Smooth transition between LIO/VIO/INS modes
   - Discontinuity limits: ≤0.30m position, ≤5° yaw
   - Covariance inflation during transitions

4. **Safe Stop Protocol:**
   - Triggered when health < 0.55
   - Gradual deceleration using last valid odometry
   - Alert ground control with fault report

**Recovery State Machine:**
```
nominal → degraded_*: Fault detected
   ↓           ↓
degraded_* → nominal: Fault cleared (health restored)
degraded_* → safe_stop: Fault worsens (health < 0.55)
   ↓
safe_stop → degraded_*: Manual recovery intervention
```

## Sensor Fusion Architecture

Multi-estimator fusion with confidence-weighted redundancy for lunar GNSS-denied navigation.

### Estimator Design

**Three Parallel Estimators:**

1. **LIO Estimator (LiDAR-Inertial Odometry)**
   - Library: FAST-LIO2 or LIO-SAM
   - Inputs: `/lidar/points`, `/imu/data`
   - Strengths: Robust to lighting, dust-resistant
   - Weaknesses: Degenerates in featureless terrain
   - Health Indicators: Point cloud density, registration residuals

2. **VIO Estimator (Visual-Inertial Odometry)**
   - Library: ORB-SLAM3 (stereo-inertial) or VINS-Fusion
   - Inputs: `/stereo/left`, `/stereo/right`, `/imu/data`
   - Strengths: High precision in textured environments
   - Weaknesses: Fails in shadows, motion blur, dust
   - Health Indicators: Tracked features, reprojection error

3. **INS Estimator (Inertial Navigation System)**
   - Library: Custom EKF with IMU integration
   - Inputs: `/imu/data` only
   - Strengths: Always available, high rate (200Hz)
   - Weaknesses: Unbounded drift without corrections
   - Health Indicators: Bias stability, temperature compensation

### Fusion Strategies

**Strategy 1: Confidence-Weighted EKF (Current)**
```cpp
struct EstimatorOutput {
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist velocity;
  float confidence;        // 0.0 to 1.0 based on health
  float covariance_norm;   // Uncertainty metric
};

// Weighted fusion
fused_pose = (w_lio * pose_lio + w_vio * pose_vio + w_ins * pose_ins) / (w_lio + w_vio + w_ins);
where w_i = confidence_i / covariance_norm_i
```

**Strategy 2: Factor Graph Optimization (Future)**
- Backend: GTSAM or g2o
- Factors: IMU preintegration, visual features, lidar registration
- Robust cost functions for outlier rejection
- Fixed-lag smoothing for real-time performance

**Strategy 3: Mode-Specific Fusion:**
- `nominal`: All three estimators active, confidence-weighted
- `degraded_lidar`: VIO + INS fusion only
- `degraded_vision`: LIO + INS fusion only
- `safe_stop`: INS only with high uncertainty, stop command

### Temporal Alignment

**Synchronization Strategy:**
```
Lidar (10Hz) ──┐
               ├─→ Temporal Interpolation → Unified timestep
Vision (20Hz) ─┤
               │
IMU (200Hz) ───┘
```

- IMU buffer: Ring buffer for last 5 seconds
- Lidar/vision: Interpolated to common timestamp
- Maximum latency: 50ms between sensors for fusion

### Confidence Calculation

**Per-Estimator Confidence:**
```cpp
float compute_confidence(const EstimatorHealth& health) {
  // Base confidence from health score
  float base = health.score;
  
  // Penalties
  float penalty = 0.0f;
  if (health.covariance_norm > 0.5f) penalty += 0.2f;
  if (health.innovation_mahalanobis > 3.0f) penalty += 0.3f;
  if (health.sync_violation) penalty += 0.1f;
  
  // Environmental factors (future)
  if (health.illumination < 10.0f) penalty += 0.15f;  // Too dark
  if (health.point_density < 100) penalty += 0.15f;   // Sparse lidar
  
  return std::max(0.0f, base - penalty);
}
```

### Handoff Management

**Smooth Transitions:**
- Detect mode change before acting (3 consecutive cycles below threshold)
- Covariance inflation during handoff: ×2 for 2 seconds
- Position discontinuity limit: 0.30m
- Orientation discontinuity limit: 5° yaw

**Handoff Sequence:**
```
1. Fault detected in primary estimator
2. Isolator confirms fault (3 cycles)
3. Mode switches to backup estimator
4. Covariance inflated
5. Smooth interpolation to new estimate
6. Gradual covariance reduction over 2s
```

### Output Interface

**Published Topics:**
```
/perception/odom          nav_msgs/Odometry    # Fused pose & twist
/perception/odom/uncertainty std_msgs/Float32  # Fused covariance norm
/perception/faults        diagnostic_msgs/DiagnosticArray  # Active faults
/perception/mode          std_msgs/String      # Current operational mode
/perception/estimator_health custom_msgs/EstimatorHealthArray
```

**Health Message Structure:**
```cpp
struct EstimatorHealth {
  string name;              // "lio", "vio", "ins"
  float confidence;         // 0.0 to 1.0
  float covariance_norm;    // Position uncertainty
  bool is_active;           // Contributing to fusion
  string[] active_faults;   // Fault flags
};
```

## Design Decisions

### 1. Plugin-Based Architecture

**Decision:** Use `pluginlib` for runtime provider loading.

**Rationale:**
- New sensors can be added without recompiling core
- Enables third-party provider packages
- Matches ROS 2 ecosystem conventions
- Supports fault injection testing via specialized providers

**Trade-off:** Slightly more complex than static linking, but enables the modular vision and FDIIR testing.

### 2. Interface Separation

**Decision:** Separate `sensor_interfaces` (pure contracts) from `sensor_core` (orchestration).

**Rationale:**
- Prevents circular dependencies
- Providers only depend on interfaces, not implementation
- Enables testing with mock implementations
- Supports FDIIR testing with simulated fault providers

### 3. FDIIR Integration

**Decision:** Implement FDIIR at system level rather than in individual estimators.

**Rationale:**
- Centralized fault management enables cross-sensor validation
- Consistent recovery strategies across all estimators
- Mode switching based on holistic system health, not individual sensor health
- Enables graceful degradation with multiple fallback options

**Trade-off:** Adds complexity to core, but provides aerospace-grade reliability.

### 4. Multi-Estimator Parallelism

**Decision:** Run LIO, VIO, and INS estimators in parallel rather than single fused estimator.

**Rationale:**
- Independent failure modes: fault in one doesn't crash others
- Enables A/B comparison for fault detection
- Instant handoff to healthy estimator on fault
- Supports confidence-weighted fusion

**Trade-off:** 3× compute overhead, requires multi-core ARM processors (Jetson Orin Nano).

### 5. Time Sync Monitoring

**Decision:** Monitor timestamp skew in core, don't force hardware sync on providers.

**Rationale:**
- Providers can use software timestamps or hardware PPS
- Core reports violations but doesn't force sync method
- Allows gradual migration from software to hardware sync
- FDIIR layer uses sync status as fault indicator

**Thresholds (per PRD):**
- Target: ≤ 5ms mean skew
- Warning: ≤ 10ms p95 skew
- Alert: > 10ms p95 skew (triggers degraded mode)

### 6. Health Score Calculation with FDIIR

**Decision:** Extend simple weighted average to full FDIIR-aware scoring.

**Current Formula:**
```
raw = 0.40 * lidar_score + 0.35 * vision_score + 0.25 * imu_score
penalty = sync_penalty + fault_penalty + environmental_penalty
overall = clamp(raw - penalty, 0.0, 1.0)

Mode determination:
- nominal: overall >= 0.75
- degraded_*: 0.55 <= overall < 0.75
- safe_stop: overall < 0.55
```

**Future additions:**
- Innovation sequence monitoring (Mahalanobis distance)
- Environmental context (lighting, terrain type)
- EMA smoothing (alpha=0.25) for hysteresis
- Predictive health based on trend analysis

### 7. Provider Isolation

**Decision:** Each provider runs as separate ROS 2 node (via Node composition).

**Rationale:**
- Crash isolation — one provider failure doesn't kill others
- Independent lifecycle management
- Easier debugging and profiling
- Supports fault injection testing

**Trade-off:** More resource overhead than single-process, but acceptable for safety-critical applications.

### 8. Pure C++ Components

**Decision:** Keep computation (stereo depth, parsing) in pure C++/OpenCV, separate from ROS.

**Rationale:**
- Testable without ROS dependencies
- Reusable in non-ROS contexts
- Easier unit testing with standard test frameworks
- Enables FDIIR testing in pure C++ without ROS overhead

### 9. Confidence-Weighted Fusion

**Decision:** Weight sensor fusion by confidence rather than fixed weights.

**Rationale:**
- Adapts to real-time sensor health
- Degrades gracefully as individual sensors fail
- Enables smooth transitions between fusion modes
- Mathematically principled (inverse covariance weighting)

**Implementation:**
```cpp
// Confidence inversely proportional to covariance
weight_i = confidence_i / (covariance_norm_i + epsilon);
fused_estimate = weighted_mean(estimates, weights);
```

## Testing & Validation

### FDIIR Testing Framework

**Unit Tests (C++):**
- Health score calculation validation
- Mode transition logic
- Fault injection simulation
- Temporal interpolation accuracy

**Integration Tests (ROS 2):**
- Multi-sensor fault scenarios
- Mode handoff verification
- End-to-end latency measurement
- Recovery sequence validation

**Fault Injection Scenarios:**

1. **Sensor Dropout:**
   - Drop 50% of lidar scans → Verify LIO degraded, VIO takes over
   - Drop all vision frames → Verify degraded_vision mode
   - Drop IMU for 500ms → Verify degraded_imu mode

2. **Timing Violations:**
   - Inject 100ms latency in camera → Verify sync violation detection
   - Intermittent sync failures → Verify hysteresis in mode switching

3. **Quality Degradation:**
   - Inject high covariance → Verify confidence weight reduction
   - Sparse point cloud → Verify LIO confidence drop
   - Motion blur → Verify VIO feature tracking failure

4. **Recovery Testing:**
   - Fault clear after 10s → Verify return to nominal
   - Partial recovery → Verify appropriate degraded mode
   - Cascading faults → Verify safe_stop trigger

### CI/CD Pipeline

**Tier 1 (Fast Feedback):**
```bash
colcon build --symlink-install
colcon test --packages-select sensor_interfaces sensor_core
```

**Tier 2 (Integration):**
```bash
# Launch with fault injection
ros2 launch juppiter test_fault_injection.launch.py scenario:=lidar_dropout

# Verify behavior
ros2 topic echo /perception/mode  # Should show degraded_lidar
ros2 topic echo /perception/faults  # Should list active faults
```

**Tier 3 (Full Validation):**
- EuRoC dataset with simulated faults
- Hardware-in-the-loop with real sensors
- Long-duration stability tests (24hr)

## Future Extensions

### Advanced FDIIR (Phase 2)

**Innovation Monitoring:**
- Mahalanobis distance tracking per estimator
- Innovation sequence whitening tests
- Predictive fault detection via trend analysis

**Environmental Adaptation:**
- Illumination-aware vision confidence adjustment
- Terrain-aware lidar quality metrics
- Temperature-compensated IMU bias estimation

**Advanced Recovery:**
- Automatic sensor re-initialization
- Warm-start from last known good state
- Ground control notification with detailed fault reports

### Multi-Agent FDIIR (Phase 3)

**Cross-Robot Validation:**
- Inter-robot pose consistency checks
- Distributed fault detection via consensus
- Collaborative recovery strategies

### Terrain Perception

Add `terrain_perception` package:
- Lidar-based geometry analysis
- Traversability scoring (slope, roughness, clearance)
- Hazard detection (rocks, crater edges, steep slopes)

### Calibration Management

Extend with calibration versioning:
```yaml
calibration:
  version: "1.0.0"
  sensors:
    - name: "camera_left"
      extrinsics: { parent: "base_link", ... }
    - name: "lidar"
      extrinsics: { parent: "base_link", ... }
```

On-boot validation and runtime drift monitoring.

### Factor Graph Fusion (Phase 4)

Replace EKF with factor graph backend:
- Backend: GTSAM or g2o
- Robust cost functions (Huber, Cauchy)
- Fixed-lag smoothing (1-2 second window)
- Outlier rejection at factor level

## Branch Strategy

- **`main`** — Stable releases, merge commits from develop
- **`develop`** — Integration branch, merge commits from feature branches
- **feature branches** — Branch off develop, PR back to develop

Rationale: Preserve full history, avoid divergence issues from squash merging.
