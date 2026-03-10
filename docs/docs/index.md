# juppiter

**Extreme-Conditions Lunar Perception Pipeline**

A modular, multi-sensor perception stack for robotic systems operating in extreme environments. Built for lunar robotics applications targeting GNSS-denied, communication-delayed environments with redundant sensing (LiDAR + stereo + IMU) and graceful degradation under dust, thermal cycling, and harsh illumination.

---

## What is juppiter?

juppiter is a ROS 2-based perception pipeline designed for lunar rover navigation. It combines multiple estimators (LiDAR-Inertial Odometry, Visual-Inertial Odometry, and Kinematic odometry) with a confidence-weighted fusion system to provide robust state estimation even when sensors fail or degrade.

## Key Features

- **Multi-sensor fusion** with confidence-weighted redundancy
- **Real-time hazard detection** for terrain safety
- **Graceful degradation** when sensors fail or degrade
- **Deterministic time synchronization** across heterogeneous sensors
- **Edge-compute optimization** for constrained hardware (Raspberry Pi 5 / Jetson Orin Nano)
- **Plugin-based sensor architecture** for easy extensibility

## Architecture Overview

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
```

## Quick Start

Ready to get started? Follow our [installation guide](getting-started/installation.md) to set up your development environment, then run your first [simulation](getting-started/running-simulation.md).

## Development Status

**Current Phase:** Active development toward PRD v1.0

- [x] Modular sensor architecture with plugin-based providers
- [x] Time synchronization framework
- [x] Health monitoring and mode management
- [x] Multi-tier compute architecture (dev/edge/low_power profiles)
- [x] Fusion core with confidence-weighted multi-estimator fusion
- [x] Gazebo Harmonic simulation environment
- [x] FDIIR test scenarios for automated validation
- [ ] FAST-LIO2 and DLIO LIO estimator implementations (in progress)
- [ ] ORB-SLAM3 and OpenVINS VIO estimator implementations (in progress)

## Get Involved

- [GitHub Repository](https://github.com/opencode/juppiter)
- [Contributing Guidelines](development/contributing.md)
- [Architecture Documentation](architecture/overview.md)

---

**License:** Apache 2.0
