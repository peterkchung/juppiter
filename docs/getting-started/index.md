# Getting Started with juppiter

Welcome! This section will guide you through setting up juppiter and running your first simulation.

## Prerequisites

Before you begin, ensure you have:

- Docker and Docker Compose installed
- (Windows) WSL2 with WSLg for GUI forwarding
- 8GB+ RAM available for the container
- 10GB+ free disk space

## What's in This Section

- [Installation](installation.md) - Set up the development environment
- [Running Simulation](running-simulation.md) - Launch Gazebo and see juppiter in action

## System Requirements

### Minimum (Low Power Profile)

- Raspberry Pi 4B 4GB
- DLIO estimator only
- Suitable for basic navigation tasks

### Recommended (Edge Profile)

- Raspberry Pi 5 8GB or Jetson Orin Nano
- DLIO + OpenVINS estimators
- Full perception capabilities

### Development (Dev Profile)

- Workstation-class hardware
- FAST-LIO2 + ORB-SLAM3 estimators
- Best for algorithm development

## Quick Reference

```bash
# Build the container
docker compose -f docker/docker-compose.yml build

# Start the container
docker compose -f docker/docker-compose.yml up -d

# Enter the container
docker compose -f docker/docker-compose.yml exec dev bash

# Inside container - build juppiter
colcon build --symlink-install

# Run simulation
ros2 launch gazebo_lunar_sim simulation.launch.py compute_profile:=dev
```

Ready? Let's [install juppiter](installation.md).
