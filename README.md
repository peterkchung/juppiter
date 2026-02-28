# juppiter: Real-Time Edge-Computed Object Pose Tracking

A perception system for mobile robots that localizes in dynamic environments, builds semantically labeled maps, and tracks 6D object poses in real-time. Targeting a Raspberry Pi 5 with an Intel RealSense D435i.

## Status

Early development. Workspace scaffolding and Docker dev environment are functional.
Early development. EuRoC dataset replay pipeline is operational.

## Stack

- **Middleware:** ROS 2 Kilted Kaiju
- **SLAM:** ORB-SLAM3 (stereo-inertial)
- **Detection:** YOLOv8n via ONNX Runtime C++
- **6D Pose:** PnP + ICP + EKF
- **Containerization:** Docker (`ros:kilted` base)

## Project Structure

```
juppiter/
├── .github/workflows/
│   └── ci.yml                  # CI: colcon build + test on PRs
├── config/
│   └── euroc_cam.yaml          # Camera intrinsics and replay defaults
├── docker/
│   ├── Dockerfile.dev          # Dev container (ROS 2 Kilted)
│   ├── docker-compose.yml      # Compose config with WSLg forwarding
│   └── entrypoint.sh           # Sources ROS 2 overlay on entry
├── launch/
│   └── replay.launch.py        # Launch file for dataset replay
├── scripts/
│   └── download_euroc.sh       # Downloads EuRoC MAV dataset sequences
├── src/
│   ├── common_msgs/            # Custom message/service definitions
│   │   ├── msg/
│   │   │   ├── Detection2D.msg
│   │   │   ├── Detection2DArray.msg
│   │   │   └── ObjectResult.msg
│   │   └── srv/
│   │       └── QueryObjects.srv
│   └── sensor_bridge/          # EuRoC replay node with stereo depth
│       ├── include/sensor_bridge/
│       │   └── sensor_bridge_node.hpp
│       └── src/
│           ├── main.cpp
│           ├── sensor_bridge_node.cpp
│           ├── euroc_reader.{hpp,cpp}
│           └── stereo_depth.{hpp,cpp}
├── DESIGN.md
├── CHANGELOG.md
└── README.md
```

## Getting Started

**Prerequisites:** Docker and Docker Compose. WSL2 with WSLg for GUI forwarding on Windows.

```bash
# Build the dev container
docker compose -f docker/docker-compose.yml build

# Open a shell in the container
docker compose -f docker/docker-compose.yml run --rm dev bash

# Rebuild packages after changes (inside container)
colcon build --symlink-install
```

### Dataset Replay

```bash
# Download a EuRoC sequence (from host)
bash scripts/download_euroc.sh MH_01_easy

# Inside the container, run the replay node
ros2 launch /ws/launch/replay.launch.py dataset_path:=/ws/data/MH_01_easy/mav0

# Verify topics (in another terminal)
ros2 topic list
ros2 topic hz /camera/color/image_raw   # ~20 Hz
ros2 topic hz /camera/imu               # ~200 Hz
```

Published topics:
- `/camera/color/image_raw` — grayscale image (mono8)
- `/camera/depth/image_rect_raw` — stereo depth map (32FC1)
- `/camera/imu` — IMU data (gyro + accel)
- `/camera/color/camera_info` — camera intrinsics

## Development

See [DESIGN.md](DESIGN.md) for architecture notes.

### Branch Guidelines

- **`main`** — stable releases only. PRs from `develop`, squash merged.
- **`develop`** — integration branch. PRs from feature branches, merge committed to preserve history.
- **feature branches** — branch off `develop`, open a PR back into `develop` when ready.

## License

Apache-2.0. See [LICENSE](LICENSE).
