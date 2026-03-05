# Changelog

All notable changes to juppiter will be documented in this file.

Format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added

#### Modular Sensor Architecture (PRD Milestone 1)
- **sensor_interfaces** package — Abstract sensor driver contracts
  - `SensorDriver` interface for all sensor providers
  - `SensorCapability` enum for capability discovery
  - `HealthStatus` and `TimeSyncStatus` data structures
  - `SensorCalibration` with versioning support
  
- **sensor_core** package — Orchestration and monitoring
  - `SensorBridgeNode` — Main orchestrator loading providers via pluginlib
  - `TimeSynchronizer` — Inter-sensor timestamp skew monitoring
  - `HealthMonitor` — Health score aggregation and mode determination
  - `/perception/health` topic publishing
  - `/perception/mode` topic publishing (nominal/degraded/safe_stop)

- **euroc_provider** package — EuRoC dataset driver
  - `EurocDriver` implementing `SensorDriver` interface
  - Plugin registration via `pluginlib`
  - Reused `EurocReader` and `StereoDepth` from old sensor_bridge
  - Topics: `/stereo/left/image_raw`, `/stereo/right/image_raw`, `/imu/data`

### Changed

- **BREAKING:** Removed monolithic `sensor_bridge` package
  - Replaced with modular `sensor_core` + `euroc_provider` architecture
  - Topic names updated per PRD:
    - `/camera/color/image_raw` → `/stereo/left/image_raw`
    - `/camera/imu` → `/imu/data`
  - Launch file updated to use new package structure

- Updated launch file `replay.launch.py`
  - Now loads `sensor_core` package
  - Configures providers list with `euroc_provider::EurocDriver`
  - Adds calibration version and health monitoring parameters

### Deprecated

- Old sensor topic names (see Changed section for migration)
- Direct EuRoC CSV parsing in ROS nodes (now wrapped in provider)

## [0.1.0] - 2026-03-05

### Added

#### Initial Project Scaffolding
- Project structure: README, DESIGN, CHANGELOG, .gitignore, .dockerignore
- Docker dev environment: Dockerfile.dev, docker-compose.yml, entrypoint.sh
- ROS 2 Kilted Kaiju base environment

#### Packages
- **common_msgs** — Custom ROS 2 messages
  - `Detection2D.msg` — 2D bounding box detection
  - `Detection2DArray.msg` — Array of detections
  - `ObjectResult.msg` — Object with pose and confidence
  - `QueryObjects.srv` — Spatial query service

- **sensor_bridge** (DEPRECATED: removed in next version)
  - EuRoC MAV dataset replay node
  - Stereo depth computation via StereoSGBM
  - 200Hz wall timer for IMU interleaving
  - Topics: `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/camera/imu`

#### Infrastructure
- CI workflow: colcon build + test on PRs to develop and main
- Auto-merge CI workflow for PRs to develop
- Apache-2.0 license file
- EuRoC download script (`scripts/download_euroc.sh`)
- Camera configuration (`config/euroc_cam.yaml`)
- Replay launch file (`launch/replay.launch.py`)

### Fixed
- EuRoC download script: use ETH Research Collection bundle URLs
- Handle two-level zip extraction in download script
- Trailing carriage return in EuRoC CSV filenames causing imread failures
- Missing `calib3d` include for StereoSGBM
- Incomplete type errors with forward-declared unique_ptr members
