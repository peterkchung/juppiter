# juppiter: Extreme-Conditions Lunar Perception Pipeline

A modular, multi-sensor perception stack for robotic systems operating in extreme environments. Built for lunar robotics applications targeting GNSS-denied, communication-delayed environments with redundant sensing (lidar + stereo + IMU) and graceful degradation under dust, thermal cycling, and harsh illumination.

## Vision

Enable safe autonomous navigation in extreme conditions through:
- **Multi-sensor fusion** with confidence-weighted redundancy
- **Real-time hazard detection** for terrain safety
- **Graceful degradation** when sensors fail or degrade
- **Deterministic time synchronization** across heterogeneous sensors
- **Edge-compute optimization** for constrained hardware (Raspberry Pi 5 / Jetson Orin Nano)

## Status

Active development toward PRD v1.0 (Extreme-Conditions Lunar Perception Pipeline).

**Completed:**
- Modular sensor architecture with plugin-based providers
- Time synchronization framework (target: ≤5ms mean skew, ≤10ms p95)
- Health monitoring and mode management (nominal/degraded/safe_stop)
- **Multi-tier compute architecture** (dev/edge/low_power profiles)
- **Modular estimator interfaces** (LIO/VIO abstraction layers)
- **Fusion core** with confidence-weighted multi-estimator fusion
- **Gazebo Harmonic simulation** environment with lunar rover
- **FDIIR test scenarios** for automated validation
- EuRoC dataset provider for testing and validation (moved to `test/`)

**In Progress:**
- FAST-LIO2 and DLIO LIO estimator implementations
- ORB-SLAM3 and OpenVINS VIO estimator implementations
- Integration testing with simulation environment
- Calibration versioning and validation

## Stack

- **Middleware:** ROS 2 Kilted Kaiju
- **Architecture:** Plugin-based sensor providers via `pluginlib` + modular estimator interfaces
- **Compute Tiers:**
  - **Dev:** FAST-LIO2 + ORB-SLAM3 (workstation-class hardware)
  - **Edge:** DLIO + OpenVINS (Raspberry Pi 5 8GB / Jetson Orin Nano)
  - **Low Power:** DLIO only (Raspberry Pi 4B 4GB minimal)
- **Simulation:** Gazebo Harmonic with lunar lighting and rigid terrain
- **Fusion:** Confidence-weighted EKF fusion (LIO primary → VIO secondary → Kinematic fallback)
- **Containerization:** Docker (`osrf/ros:kilted-desktop` base)

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    Multi-Tier Compute Architecture                          │
│  ┌─────────────────────────────────────────────────────────────────────────┐│
│  │ Configuration Layer (YAML-driven)                                      ││
│  │  dev: FAST-LIO2 + ORB-SLAM3                                           ││
│  │  edge: DLIO + OpenVINS                                                ││
│  │  low_power: DLIO only                                                 ││
│  └─────────────────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
    ┌───────────────────────────────┼───────────────────────────────┐
    │                               │                               │
┌───▼──────┐                 ┌──────▼──────┐                 ┌──────▼──────┐
│   LIO    │                 │    VIO      │                 │  Kinematic  │
│ (Primary)│                 │  (Secondary)│                 │ (Baseline)  │
├──────────┤                 ├─────────────┤                 ├─────────────┤
│FAST-LIO2 │                 │ ORB-SLAM3   │                 │robot_loc    │
│  (dev)   │                 │   (dev)     │                 │ wheel+IMU   │
│   DLIO   │                 │  OpenVINS   │                 │    EKF      │
│  (edge)  │                 │   (edge)    │                 │             │
└──────────┘                 └─────────────┘                 └─────────────┘
    │                               │                               │
    └───────────────────────────────┼───────────────────────────────┘
                                    │
                    ┌───────────────▼───────────────┐
                    │      fusion_core              │
                    │  Confidence-weighted fusion   │
                    │  - LIO primary weight: 0.50   │
                    │  - VIO secondary: 0.35        │
                    │  - Kinematic: 0.15            │
                    │  Mode: nominal/degraded/safe  │
                    └───────────────┬───────────────┘
                                    │
                    ┌───────────────▼───────────────┐
                    │  sensor_core (FDIIR)          │
                    │  - Health monitoring          │
                    │  - Mode management            │
                    │  - Time synchronization       │
                    └───────────────┬───────────────┘
                                    │
                    ┌───────────────▼───────────────┐
                    │ /perception/odom              │
                    │ /perception/mode             │
                    │ /perception/health           │
                    └───────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│  Gazebo Lunar Simulation (Harmonic)                                        │
│  - Rover with differential drive                                            │
│  - Livox Mid-360 solid-state LiDAR                                         │
│  - Stereo camera (1280x720, lunar lighting)                                │
│  - 9-axis IMU (200Hz)                                                       │
│  - Rigid terrain (Phase 1)                                                  │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Project Structure

```
juppiter/
├── .github/workflows/          # CI/CD automation
│   ├── ci.yml                  # colcon build + test on PRs
│   └── auto-merge.yml          # Auto-merge to develop
├── config/
│   ├── euroc_cam.yaml          # Camera intrinsics (legacy)
│   ├── compute_profiles/       # Multi-tier architecture configs
│   │   ├── dev.yaml           # FAST-LIO2 + ORB-SLAM3
│   │   ├── edge.yaml          # DLIO + OpenVINS
│   │   └── low_power.yaml     # DLIO only
│   ├── sensors/               # Simulated sensor configurations
│   │   ├── simulated_livox.yaml
│   │   ├── simulated_stereo.yaml
│   │   └── simulated_imu.yaml
│   └── fusion/                # Fusion algorithm configs
│       ├── weights_nominal.yaml
│       └── health_thresholds.yaml
├── docker/                     # Container development environment
│   ├── Dockerfile.dev
│   ├── docker-compose.yml
│   └── entrypoint.sh
├── launch/
│   └── replay.launch.py        # Launch file for sensor replay
├── scripts/
│   └── download_euroc.sh       # EuRoC dataset downloader
├── src/
│   ├── common_msgs/            # Custom ROS 2 message definitions
│   ├── sensor_interfaces/      # Abstract sensor contracts (pluginlib)
│   ├── sensor_core/            # Orchestration & FDIIR framework
│   │   ├── sensor_bridge_node  # Main orchestrator
│   │   ├── time_synchronizer   # Time sync monitoring
│   │   └── health_monitor      # Health aggregation & mode management
│   ├── fusion_core/            # NEW: Confidence-weighted multi-estimator fusion
│   │   ├── fusion_engine.cpp   # Core fusion algorithm
│   │   ├── mode_manager.cpp    # Mode transition logic
│   │   └── fusion_node.cpp     # ROS 2 integration
│   ├── gazebo_lunar_sim/       # NEW: Gazebo Harmonic simulation
│   │   ├── urdf/rover.urdf.xacro
│   │   ├── launch/simulation.launch.py
│   │   ├── worlds/lunar_flat.sdf
│   │   └── config/simulation.yaml
│   ├── kinetic_estimator/      # NEW: robot_localization EKF
│   │   └── kinetic_estimator_node.cpp
│   ├── lio_estimators/
│   │   └── lio_estimator_interfaces/  # NEW: Abstract LIO interface
│   │       └── lio_estimator.hpp
│   └── vio_estimators/
│       └── vio_estimator_interfaces/  # NEW: Abstract VIO interface
│           └── vio_estimator.hpp
├── test/
│   ├── euroc_provider/         # MOVED: EuRoC dataset for CI testing
│   └── fdiir_scenarios/        # NEW: Automated FDIIR validation
│       ├── lidar_dropout.yaml
│       ├── camera_degradation.yaml
│       ├── wheel_slip.yaml
│       └── cascading_fault.yaml
├── simulation/                 # NEW: Gazebo assets (future)
│   ├── materials/lunar_regolith/
│   ├── models/craters/
│   └── lighting/polar_sun/
├── docs/                       # Documentation (future)
├── DESIGN.md
├── CHANGELOG.md
└── README.md
```

## Getting Started

**Prerequisites:** Docker and Docker Compose. WSL2 with WSLg for GUI forwarding on Windows.

### Container Workflow

Build the dev container:

```bash
docker compose -f docker/docker-compose.yml build
```

Start the container:

```bash
docker compose -f docker/docker-compose.yml up -d
```

Enter the container:

```bash
docker compose -f docker/docker-compose.yml exec dev bash
```

**Inside the container:** Your prompt becomes `root@juppiter-dev:/ws#`

Stop the container when done:

```bash
docker compose -f docker/docker-compose.yml down
```

### Building

Inside the container:

```bash
colcon build --symlink-install
```

### Running

**Option A: Gazebo Simulation (Recommended for Development)**

Launch the lunar rover simulation with Gazebo Harmonic:

```bash
ros2 launch gazebo_lunar_sim simulation.launch.py compute_profile:=dev
```

This starts:
- Gazebo Harmonic with lunar lighting
- Simulated rover with differential drive
- Livox Mid-360 LiDAR publishing to `/lidar/points`
- Stereo cameras publishing to `/stereo/left/image_raw` and `/stereo/right/image_raw`
- 9-axis IMU publishing to `/imu/data` (200Hz)
- Kinetic estimator (wheel odometry + IMU EKF)

**Option B: EuRoC Dataset Replay (Legacy Testing)**

```bash
ros2 launch launch/replay.launch.py dataset_path:=/ws/data/MH_01_easy/mav0
```

**Verify operation:**

```bash
# List all topics
ros2 topic list

# Check sensor rates
ros2 topic hz /lidar/points         # ~10 Hz (LiDAR)
ros2 topic hz /stereo/left/image_raw # ~20 Hz (Camera)
ros2 topic hz /imu/data             # ~200 Hz (IMU)
ros2 topic hz /wheel/odometry       # ~50 Hz (Wheel odometry)

# Monitor system health and mode
ros2 topic echo /perception/health
ros2 topic echo /perception/mode

# View fused odometry
ros2 topic echo /perception/odom
```

### Published Topics

#### Perception Pipeline

| Topic | Type | Description |
|-------|------|-------------|
| `/perception/odom` | nav_msgs/Odometry | Fused pose estimate (LIO+VIO+Kinematic) |
| `/perception/mode` | std_msgs/String | Current mode: nominal/degraded_lio/degraded_vio/safe_stop |
| `/perception/health` | std_msgs/String | System health score and active faults (JSON) |
| `/perception/estimator_weights` | std_msgs/String | Current fusion weights by source |

#### Estimator Outputs

| Topic | Type | Description |
|-------|------|-------------|
| `/lio/odom` | nav_msgs/Odometry | LIO pose estimate (FAST-LIO2/DLIO) |
| `/vio/odom` | nav_msgs/Odometry | VIO pose estimate (ORB-SLAM3/OpenVINS) |
| `/kinematic/odom` | nav_msgs/Odometry | Wheel odometry + IMU EKF output |

#### Simulation Sensors (Gazebo)

| Topic | Type | Description |
|-------|------|-------------|
| `/lidar/points` | sensor_msgs/PointCloud2 | Livox Mid-360 point cloud |
| `/stereo/left/image_raw` | sensor_msgs/Image | Left camera (1280x720, 20Hz) |
| `/stereo/right/image_raw` | sensor_msgs/Image | Right camera (1280x720, 20Hz) |
| `/stereo/left/camera_info` | sensor_msgs/CameraInfo | Left camera intrinsics |
| `/stereo/right/camera_info` | sensor_msgs/CameraInfo | Right camera intrinsics |
| `/imu/data` | sensor_msgs/Imu | 9-axis IMU (200Hz) |
| `/wheel/odometry` | nav_msgs/Odometry | Differential drive odometry |

#### Legacy (EuRoC Testing)

| Topic | Type | Description |
|-------|------|-------------|
| `/stereo/left/image_raw` | sensor_msgs/Image | Left grayscale image (EuRoC dataset) |
| `/stereo/right/image_raw` | sensor_msgs/Image | Right grayscale image (EuRoC dataset) |
| `/camera/depth/image_rect_raw` | sensor_msgs/Image | Stereo depth map (computed) |
| `/imu/data` | sensor_msgs/Imu | IMU measurements (EuRoC dataset) |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | Camera intrinsics |

## Architecture Details

### Adding a New Sensor Provider

1. Create new package (e.g., `lidar_provider`):
```bash
cd src
mkdir lidar_provider/{src,include/lidar_provider}
```

2. Implement `SensorDriver` interface:
```cpp
#include "sensor_interfaces/sensor_driver.hpp"

class LidarDriver : public sensor_interfaces::SensorDriver {
  // Implement all virtual methods
};
```

3. Register with pluginlib in `lidar_provider_plugin.xml`

4. Load via configuration:
```yaml
providers: ['lidar_provider::LidarDriver', 'euroc_provider::EurocDriver']
```

No core recompilation required!

## Development

See [DESIGN.md](DESIGN.md) for architecture notes and [CHANGELOG.md](CHANGELOG.md) for recent changes.

### Branch Guidelines

- **`main`** — stable releases only. PRs from `develop`, merge committed.
- **`develop`** — integration branch. PRs from feature branches.
- **feature branches** — branch off `develop`, open PR back into `develop`.

### Testing

#### Unit & Integration Tests

```bash
# Build all packages
colcon build --symlink-install

# Run tests for core packages
colcon test --packages-select sensor_interfaces sensor_core fusion_core

# View test results
colcon test-result --verbose
```

#### FDIIR Validation (Simulation)

Run automated FDIIR test scenarios in simulation:

```bash
# Test LiDAR dropout → VIO takeover
ros2 launch gazebo_lunar_sim simulation.launch.py scenario:=lidar_dropout

# Test camera degradation → LIO priority
ros2 launch gazebo_lunar_sim simulation.launch.py scenario:=camera_degradation

# Test wheel slip on regolith
ros2 launch gazebo_lunar_sim simulation.launch.py scenario:=wheel_slip

# Test cascading failures → safe_stop
ros2 launch gazebo_lunar_sim simulation.launch.py scenario:=cascading_fault
```

Test scenarios are defined in `test/fdiir_scenarios/*.yaml` with:
- Fault injection schedules (sensor dropout, rate degradation, etc.)
- Expected behavior assertions
- Success criteria and metrics

#### Manual Verification

```bash
# Monitor health status
ros2 topic echo /perception/health

# Monitor mode transitions
ros2 topic echo /perception/mode

# View estimator weights
ros2 topic echo /perception/estimator_weights

# RViz visualization
rviz2 -d /ws/src/gazebo_lunar_sim/config/rviz.rviz
```

## License

Apache-2.0. See [LICENSE](LICENSE).
