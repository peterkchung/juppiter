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
- EuRoC dataset provider for testing and validation

**In Progress:**
- Lidar provider integration
- Calibration versioning and validation
- Multi-estimator fusion (LIO + VIO)

## Stack

- **Middleware:** ROS 2 Kilted Kaiju
- **Architecture:** Plugin-based sensor providers via `pluginlib`
- **Lidar-Inertial:** FAST-LIO2 or LIO-SAM (planned)
- **Stereo-VIO:** ORB-SLAM3 (stereo-inertial) (planned)
- **Fusion:** EKF-based multi-estimator fusion (starting), factor-graph (future)
- **Containerization:** Docker (`osrf/ros:kilted-desktop` base)

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    sensor_core (orchestrator)               │
│  ┌─────────────────┐  ┌─────────────────┐                     │
│  │ TimeSynchronizer│  │ HealthMonitor   │                     │
│  │ (skew detection)│  │ (mode manager)  │                     │
│  └─────────────────┘  └─────────────────┘                     │
│           │                      │                          │
│           ▼                      ▼                          │
│    /perception/health    /perception/mode                   │
└─────────────────────────────────────────────────────────────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
┌───────▼──────┐  ┌────────▼──────┐  ┌───────▼──────┐
│euroc_provider│  │lidar_provider│  │other_provider│
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
    │  (LIO + VIO merge) │
    └────────────────────┘
```

## Project Structure

```
juppiter/
├── .github/workflows/          # CI/CD automation
│   ├── ci.yml                  # colcon build + test on PRs
│   └── auto-merge.yml          # Auto-merge to develop
├── config/
│   └── euroc_cam.yaml          # Camera intrinsics and replay defaults
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
│   │   ├── msg/
│   │   │   ├── Detection2D.msg
│   │   │   ├── Detection2DArray.msg
│   │   │   └── ObjectResult.msg
│   │   └── srv/
│   │       └── QueryObjects.srv
│   ├── sensor_interfaces/      # NEW: Abstract sensor contracts
│   │   └── include/
│   │       ├── sensor_driver.hpp       # Plugin interface
│   │       └── time_sync_types.hpp     # Calibration structures
│   ├── sensor_core/            # NEW: Orchestration & monitoring
│   │   ├── include/
│   │   │   ├── sensor_bridge_node.hpp  # Main orchestrator
│   │   │   ├── time_synchronizer.hpp   # Time sync monitoring
│   │   │   └── health_monitor.hpp      # Health aggregation
│   │   └── src/
│   │       ├── sensor_bridge_node.cpp
│   │       ├── time_synchronizer.cpp
│   │       ├── health_monitor.cpp
│   │       └── main.cpp
│   └── euroc_provider/         # NEW: EuRoC dataset provider
│       ├── include/
│       │   ├── euroc_driver.hpp        # SensorDriver impl
│       │   ├── euroc_reader.hpp        # CSV/image parsing
│       │   └── stereo_depth.hpp        # Stereo depth computation
│       └── src/
│           ├── euroc_driver.cpp
│           ├── euroc_reader.cpp
│           └── stereo_depth.cpp
├── .dev/
│   ├── PRD_Extreme_Conditions_Lunar_Perception.md    # Active PRD
│   └── ...
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

**Terminal 1** (inside container): Launch the sensor bridge with EuRoC provider:

```bash
ros2 launch /ws/launch/replay.launch.py dataset_path:=/ws/data/MH_01_easy/mav0
```

**Terminal 2** (new shell inside container): Verify topics:

```bash
docker compose -f docker/docker-compose.yml exec dev bash
ros2 topic list
ros2 topic hz /stereo/left/image_raw    # ~20 Hz
ros2 topic hz /imu/data                # ~200 Hz
ros2 topic echo /perception/health     # Health status
```

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/stereo/left/image_raw` | sensor_msgs/Image | Left grayscale image |
| `/stereo/right/image_raw` | sensor_msgs/Image | Right grayscale image |
| `/camera/depth/image_rect_raw` | sensor_msgs/Image | Stereo depth map |
| `/imu/data` | sensor_msgs/Imu | IMU measurements |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | Camera intrinsics |
| `/perception/health` | std_msgs/String | Health score and mode (JSON) |
| `/perception/mode` | std_msgs/String | Current mode: nominal/degraded/safe_stop |

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

```bash
# Linting (currently active)
colcon test --packages-select sensor_interfaces sensor_core euroc_provider
colcon test-result --verbose

# Manual verification
ros2 topic echo /perception/health
```

## License

Apache-2.0. See [LICENSE](LICENSE).
