# Changelog

All notable changes to juppiter will be documented in this file.

Format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- Project scaffolding: README, DESIGN, CHANGELOG, .gitignore, .dockerignore
- Docker dev environment: Dockerfile.dev, docker-compose.yml, entrypoint.sh (ROS 2 Kilted)
- `common_msgs` package: Detection2D, Detection2DArray, ObjectResult messages and QueryObjects service
- CI workflow: colcon build + test on PRs to develop and main
