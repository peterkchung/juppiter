# System Architecture

juppiter is built on a multi-layer architecture designed for fault tolerance and extensibility. This document provides a detailed view of each layer and how they interact.

## High-Level System Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  Configuration Layer (YAML-driven)                                          │
├─────────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐              │
│  │   Dev Profile   │  │  Edge Profile   │  │ Low Power       │              │
│  │ FAST-LIO2 +     │  │ DLIO +          │  │ DLIO only       │              │
│  │ ORB-SLAM3       │  │ OpenVINS        │  │                 │              │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘              │
│           │                    │                    │                        │
└───────────┼────────────────────┼────────────────────┼────────────────────────┘
            │                    │                    │
┌───────────▼────────────────────▼────────────────────▼────────────────────────┐
│                         Estimator Layer                                       │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐              │
│  │      LIO         │  │      VIO         │  │   Kinematic      │              │
│  │  (Primary)       │  │  (Secondary)     │  │  (Baseline)      │              │
│  ├──────────────────┤  ├──────────────────┤  ├──────────────────┤              │
│  │ • LiDAR points   │  │ • Stereo images  │  │ • Wheel encoders │              │
│  │ • IMU data       │  │ • IMU data       │  │ • IMU data       │              │
│  │ • Outputs:       │  │ • Outputs:       │  │ • Outputs:       │              │
│  │   /lio/odom      │  │   /vio/odom      │  │   /kinematic/odom│              │
│  └────────┬─────────┘  └────────┬─────────┘  └────────┬─────────┘              │
│           │                   │                    │                        │
│           └───────────────────┼────────────────────┘                        │
│                               │                                             │
└───────────────────────────────▼─────────────────────────────────────────────┘
                                │
┌───────────────────────────────▼─────────────────────────────────────────────┐
│                         Fusion Core                                          │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │              Confidence-Weighted EKF Fusion                           │ │
│  │  ┌─────────────────────────────────────────────────────────────────┐ │ │
│  │  │ State: [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]      │ │ │
│  │  │                                                                  │ │ │
│  │  │ Weights (nominal):                                              │ │ │
│  │  │   • LIO:    0.50  (primary, most accurate)                      │ │ │
│  │  │   • VIO:    0.35  (secondary, visual backup)                    │ │ │
│  │  │   • Kinematic: 0.15 (baseline, always available)                │ │ │
│  │  │                                                                  │ │ │
│  │  │ Dynamic adjustment based on:                                     │ │ │
│  │  │   • Health scores from sensor_core                             │ │ │
│  │  │   • Innovation covariance (consistency check)                    │ │ │
│  │  │   • Dropout detection (zero weight if offline)                 │ │ │
│  │  └─────────────────────────────────────────────────────────────────┘ │ │
│  │                                                                     │ │
│  │  Output: /perception/odom                                          │ │
│  └─────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
                                │
┌───────────────────────────────▼─────────────────────────────────────────────┐
│                      Sensor Core (FDIIR Framework)                          │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │  Fault Detection, Isolation, Identification, Recovery                  │ │
│  │                                                                        │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                  │ │
│  │  │   Fault      │  │   Fault      │  │   Recovery   │                  │ │
│  │  │  Detector    │  │  Isolator    │  │   Manager    │                  │ │
│  │  │              │  │              │  │              │                  │ │
│  │  │ • Stale data │  │ • Mode       │  │ • Degrade    │                  │ │
│  │  │ • Sync drift │  │   manager    │  │   gracefully │                  │ │
│  │  │ • Rate drop  │  │ • Fault      │  │ • Switch     │                  │ │
│  │  │ • Covariance │  │   mapping    │  │   estimators │                  │ │
│  │  └──────────────┘  └──────────────┘  └──────────────┘                  │ │
│  │                                                                        │ │
│  │  Topics: /perception/health, /perception/mode, /perception/faults    │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
                                │
┌───────────────────────────────▼─────────────────────────────────────────────┐
│                      Sensor Provider Layer                                    │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │  Plugin-based architecture via pluginlib                               │ │
│  │                                                                        │ │
│  │  Interface: sensor_interfaces::SensorDriver                            │ │
│  │                                                                        │ │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐        │ │
│  │  │  euroc_provider │  │  lidar_provider │  │  real_provider  │        │ │
│  │  │  (EuRoC replay) │  │  (Livox Mid-360)│  │  (Your sensor)  │        │ │
│  │  │  [Test only]    │  │  [Future]       │  │  [Plugin]       │        │ │
│  │  └─────────────────┘  └─────────────────┘  └─────────────────┘        │ │
│  │                                                                        │ │
│  │  Each provider implements:                                             │ │
│  │   • initialize() - Setup sensor connection                           │ │
│  │   • start() - Begin publishing                                         │ │
│  │   • stop() - Halt publishing                                           │ │
│  │   • getHealthStatus() - Return health metrics                        │ │
│  │   • getCapabilities() - Advertise supported data types             │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Component Details

### 1. Configuration Layer

The system is configured via YAML files located in `config/`:

- `compute_profiles/` - Hardware-specific estimator configurations
- `sensors/` - Sensor calibration and parameters
- `fusion/` - Fusion algorithm weights and thresholds

Configuration is loaded at runtime, allowing the same binary to run on different hardware tiers by changing a single parameter.

### 2. Estimator Layer

Three independent estimators run concurrently:

**LIO (LiDAR-Inertial Odometry)**
- Primary estimator with highest accuracy
- Uses LiDAR point clouds + IMU
- Options: FAST-LIO2 (dev), DLIO (edge)
- Publishes: `/lio/odom`

**VIO (Visual-Inertial Odometry)**
- Secondary estimator, visual backup
- Uses stereo images + IMU
- Options: ORB-SLAM3 (dev), OpenVINS (edge)
- Publishes: `/vio/odom`

**Kinematic**
- Baseline estimator, always available
- Uses wheel odometry + IMU EKF
- Runs: robot_localization package
- Publishes: `/kinematic/odom`

### 3. Fusion Core

Combines estimates using confidence-weighted EKF:

```cpp
// Simplified fusion logic
State x_fused = 
    w_lio * x_lio + 
    w_vio * x_vio + 
    w_kin * x_kinematic;

// Weights are dynamic based on health:
// - Healthy LIO: w_lio = 0.50
// - Degraded LIO: w_lio = 0.25, w_vio increases
// - LIO offline: w_lio = 0.0, VIO becomes primary
```

### 4. Sensor Core (FDIIR)

Implements the FDIIR framework:

**Fault Detection**
- Monitors timestamp gaps (detects stale data)
- Tracks inter-sensor skew (detects sync issues)
- Measures publication rates (detects drops)
- Validates covariance bounds

**Fault Isolation**
- Identifies which sensor/estimator is faulty
- Maps faults to system degradation modes

**Recovery Management**
- Gracefully degrades to remaining healthy sensors
- Transitions between operational modes
- Attempts restart/recovery where possible

### 5. Sensor Provider Layer

Plugin-based sensor interface:

```cpp
// All providers implement:
class SensorDriver {
    virtual bool initialize(rclcpp::Node::SharedPtr node) = 0;
    virtual bool start() = 0;
    virtual bool stop() = 0;
    virtual HealthStatus getHealthStatus() = 0;
    virtual SensorCapability getCapabilities() = 0;
};
```

## Data Flow Example

Here's how a single LiDAR scan propagates through the system:

1. **Sensor Provider** (simulated or real LiDAR) publishes `/lidar/points`
2. **Sensor Core** timestamps the message and checks health
3. **LIO Estimator** processes point cloud + IMU → publishes `/lio/odom`
4. **Fusion Core** receives `/lio/odom`, `/vio/odom`, `/kinematic/odom`
5. **Fusion Core** weights each by confidence, outputs `/perception/odom`
6. **Navigation stack** consumes `/perception/odom` for path planning

## Time Synchronization

Critical for multi-sensor fusion. The TimeSynchronizer ensures:

- Mean inter-sensor skew: ≤ 5ms (target)
- p95 inter-sensor skew: ≤ 10ms (target)
- All timestamps use ROS 2 `rclcpp::Time` with steady clock

## Mode Management

The system operates in four modes:

| Mode | Health Score | Description |
|------|---------------|-------------|
| `nominal` | ≥ 0.75 | All sensors healthy, normal operation |
| `degraded_lio` | 0.55 - 0.74 | LiDAR issues, using VIO as primary |
| `degraded_vio` | 0.55 - 0.74 | Camera issues, using LIO as primary |
| `safe_stop` | < 0.55 | Critical failure, autonomous stop required |

Mode transitions are logged and published on `/perception/mode`.

## Package Structure

```
src/
├── sensor_interfaces/     # Abstract sensor contracts
├── sensor_core/           # FDIIR orchestration
├── fusion_core/           # Confidence-weighted fusion
├── lio_estimators/        # LIO implementations
│   ├── lio_estimator_interfaces/  # Abstract LIO
│   ├── fast_lio2/         # FAST-LIO2 integration
│   └── dlio/              # DLIO integration
├── vio_estimators/        # VIO implementations
│   ├── vio_estimator_interfaces/  # Abstract VIO
│   ├── orb_slam3/         # ORB-SLAM3 integration
│   └── openvins/          # OpenVINS integration
├── kinetic_estimator/     # robot_localization wrapper
└── gazebo_lunar_sim/      # Simulation environment
```

## Next Topics

- [Compute Profiles](compute-profiles.md) - Hardware-specific configurations
- [Sensor Core](sensor-core.md) - Deep dive into FDIIR
- [Fusion Core](fusion-core.md) - Confidence fusion algorithms
- [Interfaces](interfaces.md) - Plugin development guide
