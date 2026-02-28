# juppiter: Real-Time Edge-Computed Object Pose Tracking

A perception system for mobile robots that localizes in dynamic environments, builds semantically labeled maps, and tracks 6D object poses in real-time. Targeting a Raspberry Pi 5 with an Intel RealSense D435i.

## Status

Early development. Workspace scaffolding and Docker dev environment are functional.

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
├── docker/
│   ├── Dockerfile.dev          # Dev container (ROS 2 Kilted)
│   ├── docker-compose.yml      # Compose config with WSLg forwarding
│   └── entrypoint.sh           # Sources ROS 2 overlay on entry
├── src/
│   └── common_msgs/            # Custom message/service definitions
│       ├── msg/
│       │   ├── Detection2D.msg
│       │   ├── Detection2DArray.msg
│       │   └── ObjectResult.msg
│       └── srv/
│           └── QueryObjects.srv
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

## Development

See [DESIGN.md](DESIGN.md) for architecture notes.

### Branch Guidelines

- **`main`** — stable releases only. PRs from `develop`, squash merged.
- **`develop`** — integration branch. PRs from feature branches, merge committed to preserve history.
- **feature branches** — branch off `develop`, open a PR back into `develop` when ready.

## License

tbd
