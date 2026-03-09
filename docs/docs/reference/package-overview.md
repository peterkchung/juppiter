# Package Overview

Complete reference for all packages in the juppiter source tree.

## Package Tree

```
src/
├── common_msgs/                   # Custom ROS message definitions
├── sensor_interfaces/            # Abstract sensor contracts
├── sensor_core/                  # FDIIR orchestration
├── fusion_core/                  # Confidence-weighted fusion
├── gazebo_lunar_sim/             # Simulation environment
├── kinetic_estimator/            # Wheel odometry + IMU EKF
├── lio_estimators/               # LiDAR-Inertial Odometry
│   ├── lio_estimator_interfaces/ # Abstract LIO interface
│   ├── fast_lio2/                # FAST-LIO2 integration
│   └── dlio/                     # Direct LiDAR-Inertial Odometry
├── vio_estimators/               # Visual-Inertial Odometry
│   ├── vio_estimator_interfaces/ # Abstract VIO interface
│   ├── orb_slam3/                # ORB-SLAM3 integration
│   └── openvins/                 # OpenVINS integration
└── juppiter_integration_tests/   # Integration test suite
```

## common_msgs

**Purpose:** Custom ROS 2 message and service definitions.

**Type:** Interface package (headers only)

**Messages:**
- `HealthStatus.msg` - Sensor health metrics
- `FusionWeights.msg` - Current fusion weights
- `SystemMode.msg` - Operational mode

**Services:**
- `ResetEstimators.srv` - Reset all estimators
- `GetHealth.srv` - Query health status

**Dependencies:**
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`

## sensor_interfaces

**Purpose:** Header-only package defining contracts for all sensor implementations.

**Type:** Interface package (headers only)

**Key Components:**

### SensorDriver
Abstract base class for all sensor providers.

```cpp
class SensorDriver {
public:
    virtual bool initialize(rclcpp::Node::SharedPtr node) = 0;
    virtual bool start() = 0;
    virtual bool stop() = 0;
    virtual void shutdown() = 0;
    virtual HealthStatus getHealthStatus() = 0;
    virtual SensorCapability getCapabilities() = 0;
};
```

### Supporting Types

- `HealthStatus` - Standardized health reporting
- `SensorCapability` - Capability discovery flags
- `FaultFlags` - Fault classification (STALE, SYNC_VIOLATION, etc.)
- `TimeSyncStatus` - Time synchronization monitoring
- `SensorCalibration` - Calibration data with versioning

**Dependencies:**
- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`

## sensor_core

**Purpose:** Central orchestration and FDIIR framework implementation.

**Type:** Executable package

**Nodes:**

### sensor_bridge_node
Main orchestration node that:
- Loads sensor providers via pluginlib
- Manages provider lifecycle
- Monitors health at 10Hz
- Publishes system health and mode

**Topics Published:**
- `/perception/health` - JSON health metrics
- `/perception/mode` - System mode (nominal/degraded/safe_stop)
- `/perception/faults` - Active fault log

**Parameters:**
- `config_path` - Path to compute profile YAML
- `monitoring_rate_hz` - Health check frequency

### Components

**TimeSynchronizer:**
- Registers all sensors for timestamp monitoring
- Computes inter-sensor skew (mean, p95, max)
- Reports against PRD thresholds (5ms mean, 10ms p95)

**HealthMonitor:**
- Aggregates health from all registered drivers
- Computes overall health score
- Determines system mode transitions

**FaultDetector:**
- Detects stale data (timeout = 2x expected period)
- Monitors publication rates
- Checks timestamp synchronization
- Validates covariance bounds

**Dependencies:**
- `sensor_interfaces`
- `common_msgs`
- `pluginlib`
- `rclcpp_components`

## fusion_core

**Purpose:** Confidence-weighted multi-estimator fusion.

**Type:** Executable package

**Node:** fusion_node

**Algorithm:**
Extended Kalman Filter (EKF) with confidence weighting:
- LIO primary weight: 0.50 (nominal)
- VIO secondary weight: 0.35
- Kinematic baseline weight: 0.15

**Topics Subscribed:**
- `/lio/odom` - LIO pose estimate
- `/vio/odom` - VIO pose estimate
- `/kinematic/odom` - Wheel odometry

**Topics Published:**
- `/perception/odom` - Fused pose estimate
- `/perception/estimator_weights` - Current weights (JSON)

**Parameters:**
- `config_path` - Path to fusion weights YAML
- `min_weight` - Minimum allowed weight (default: 0.05)
- `max_weight` - Maximum allowed weight (default: 0.90)

**Dependencies:**
- `common_msgs`
- `nav_msgs`
- `rclcpp`

## gazebo_lunar_sim

**Purpose:** Gazebo Harmonic simulation environment for lunar rover.

**Type:** Executable package with models and launch files

**Launch Files:**

### simulation.launch.py
Main simulation launcher with parameters:
- `compute_profile` - dev/edge/low_power
- `scenario` - FDIIR test scenario (optional)
- `world` - Gazebo world file

**Models:**
- `rover.urdf.xacro` - Lunar rover URDF
  - Differential drive
  - Livox Mid-360 LiDAR
  - Stereo camera pair
  - 9-axis IMU

**Worlds:**
- `lunar_flat.sdf` - Flat lunar terrain
- `lunar_craters.sdf` - Terrain with craters (future)

**Topics Published:**
- `/lidar/points` - Livox point cloud
- `/stereo/left/image_raw` - Left camera
- `/stereo/right/image_raw` - Right camera
- `/imu/data` - IMU data
- `/wheel/odometry` - Ground truth odometry

**Dependencies:**
- `gazebo_ros`
- `ros_gz_bridge`
- `robot_state_publisher`

## kinetic_estimator

**Purpose:** robot_localization EKF wrapper for wheel odometry + IMU.

**Type:** Executable package

**Node:** kinetic_estimator_node

**Inputs:**
- `/wheel/odometry` - Wheel encoder odometry
- `/imu/data` - IMU measurements

**Output:**
- `/kinematic/odom` - Fused wheel + IMU pose

**Configuration:**
Via YAML in compute profile:
```yaml
kinetic:
  ekf_config:
    frequency: 50.0
    odom0: "/wheel/odometry"
    imu0: "/imu/data"
```

**Dependencies:**
- `robot_localization`
- `nav_msgs`
- `sensor_msgs`

## lio_estimators

### lio_estimator_interfaces

**Purpose:** Abstract interface for LIO estimators.

**Type:** Interface package (headers only)

**Key Class:** `LIOEstimator`

```cpp
class LIOEstimator {
public:
    virtual bool initialize(rclcpp::Node::SharedPtr node, 
                           const std::string& config_path) = 0;
    virtual void processPointCloud(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg) = 0;
    virtual void processImu(
        const sensor_msgs::msg::Imu::SharedPtr msg) = 0;
    virtual nav_msgs::msg::Odometry getOdometry() = 0;
    virtual EstimatorHealth getHealth() = 0;
    virtual bool reset() = 0;
};
```

### fast_lio2

**Purpose:** FAST-LIO2 integration (dev profile).

**Type:** Executable package

**Status:** In development

**Characteristics:**
- Higher accuracy than DLIO
- Higher CPU usage (~40%)
- IKDTree for efficient nearest neighbor search
- ikd-Tree for dynamic point cloud handling

### dlio

**Purpose:** Direct LiDAR-Inertial Odometry integration (edge/low_power profiles).

**Type:** Executable package

**Status:** In development

**Characteristics:**
- Faster than FAST-LIO2
- Lower CPU usage (~35%)
- Optimized for ARM processors
- GICP-based point cloud registration

**Dependencies:**
- `lio_estimator_interfaces`
- `pcl_ros`
- `pcl_conversions`

## vio_estimators

### vio_estimator_interfaces

**Purpose:** Abstract interface for VIO estimators.

**Type:** Interface package (headers only)

**Key Class:** `VIOEstimator`

```cpp
class VIOEstimator {
public:
    virtual bool initialize(rclcpp::Node::SharedPtr node,
                           const std::string& config_path) = 0;
    virtual void processStereoImages(
        const sensor_msgs::msg::Image::SharedPtr left,
        const sensor_msgs::msg::Image::SharedPtr right) = 0;
    virtual void processImu(
        const sensor_msgs::msg::Imu::SharedPtr msg) = 0;
    virtual nav_msgs::msg::Odometry getOdometry() = 0;
    virtual EstimatorHealth getHealth() = 0;
    virtual bool reset() = 0;
};
```

### orb_slam3

**Purpose:** ORB-SLAM3 integration (dev profile).

**Type:** Executable package

**Status:** In development

**Characteristics:**
- Feature-based SLAM
- Loop closure detection
- Map reuse
- Higher CPU usage (~30%)

### openvins

**Purpose:** OpenVINS integration (edge profile).

**Type:** Executable package

**Status:** In development

**Characteristics:**
- MSCKF-based VIO
- Optimized for ARM
- Lower CPU usage (~25%)
- Real-time capable on Pi 5

**Dependencies:**
- `vio_estimator_interfaces`
- `cv_bridge`
- `image_transport`

## juppiter_integration_tests

**Purpose:** Integration test suite for validating component interactions.

**Type:** Test package (not built by default)

**Tests:**
- Sensor provider loading
- Time synchronization
- Health monitoring
- Fusion algorithm
- End-to-end pipeline

**Launch Files:**
- `test_sensor_core.launch.py`
- `test_fusion.launch.py`
- `test_full_pipeline.launch.py`

**Dependencies:**
- All other juppiter packages
- `launch_testing`

## Dependency Graph

```
                    ┌─────────────────┐
                    │   common_msgs   │
                    └────────┬────────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
        ▼                    ▼                    ▼
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│sensor_core   │    │ fusion_core  │    │ sensor_intf  │
└──────┬───────┘    └──────┬───────┘    └──────┬───────┘
       │                   │                   │
       │            ┌──────┴──────┐            │
       │            │             │            │
       ▼            ▼             ▼            ▼
┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐
│ gazebo   │  │ kinetic  │  │  lio     │  │  vio     │
│ lunar_sim│  │ estimator│  │  estimators  │  estimators  │
└──────────┘  └──────────┘  └──────────┘  └──────────┘
```

## Building Packages

### Build All

```bash
colcon build --symlink-install
```

### Build Single Package

```bash
colcon build --packages-select sensor_core
```

### Build with Tests

```bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
```

### Clean Build

```bash
rm -rf build/ install/ log/
colcon build --symlink-install
```

## Package Versioning

All packages follow the same version scheme:
- Version stored in `package.xml`
- Format: `MAJOR.MINOR.PATCH`
- Updated together for releases

Example `package.xml`:
```xml
<package format="3">
  <name>sensor_core</name>
  <version>0.1.0</version>
  <description>FDIIR orchestration</description>
  ...
</package>
```

## Next Topics

- [Architecture Overview](../architecture/overview.md) - System architecture
- [Sensor Integration Guide](../user-guides/sensor-integration.md) - Adding packages
- [ROS Topics Reference](ros-topics.md) - Topic catalog
