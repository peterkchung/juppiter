# juppiter: Real-Time Edge-Computed Object Pose Tracking

A perception system for mobile robots that localizes in dynamic environments, builds semantically labeled maps, and tracks 6D object poses in real-time. Targeting a Raspberry Pi 5 with an Intel RealSense D435i.

## Status

Early development. Currently scaffolding the ROS 2 workspace and Docker environment.

## Stack

- **Middleware:** ROS 2 Humble
- **SLAM:** ORB-SLAM3 (stereo-inertial)
- **Detection:** YOLOv8n via ONNX Runtime C++
- **6D Pose:** PnP + ICP + EKF
- **Containerization:** Docker (`ros:humble` base)

## Getting Started

tbd — Docker build and workspace setup instructions will be added at a later date.

## Development

See [DESIGN.md](DESIGN.md) for architecture notes.

## License

tbd
