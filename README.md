# juppiter: Real-Time Edge-Computed Object Pose Tracking

A perception system for mobile robots that localizes in dynamic environments, builds semantically labeled maps, and tracks 6D object poses in real-time. Targeting a Raspberry Pi 5 with an Intel RealSense D435i.

## Status

Early development. Currently scaffolding the ROS 2 workspace and Docker environment.

## Stack

- **Middleware:** ROS 2 Kilted Kaiju
- **SLAM:** ORB-SLAM3 (stereo-inertial)
- **Detection:** YOLOv8n via ONNX Runtime C++
- **6D Pose:** PnP + ICP + EKF
- **Containerization:** Docker (`ros:kilted` base)

## Project Structure

```
juppiter/
├── docker/
│   ├── Dockerfile.dev          # Dev container (placeholder)
│   └── docker-compose.yml      # Compose config (placeholder)
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

tbd — Docker build and workspace setup instructions will be added at a later date.

## Development

See [DESIGN.md](DESIGN.md) for architecture notes.

### Branch Guidelines

- **`main`** — stable releases only. PRs from `develop`, squash merged.
- **`develop`** — integration branch. PRs from feature branches, merge committed to preserve history.
- **feature branches** — branch off `develop`, open a PR back into `develop` when ready.

## License

tbd
