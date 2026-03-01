# Changelog

All notable changes to juppiter will be documented in this file.

Format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- Project scaffolding: README, DESIGN, CHANGELOG, .gitignore, .dockerignore
- Docker dev environment: Dockerfile.dev, docker-compose.yml, entrypoint.sh (ROS 2 Kilted)
- `common_msgs` package: Detection2D, Detection2DArray, ObjectResult messages and QueryObjects service
- CI workflow: colcon build + test on PRs to develop and main
- Auto-merge CI workflow for PRs to develop
- Apache-2.0 license file
- `sensor_bridge` package: EuRoC MAV dataset replay with stereo depth computation
- EuRoC download script and camera config
- Replay launch file

### Fixed
- EuRoC download script: use ETH Research Collection bundle URLs, handle two-level zip extraction
- Trailing carriage return in EuRoC CSV filenames causing imread failures
- Missing `calib3d` include for StereoSGBM
- Incomplete type errors with forward-declared unique_ptr members
