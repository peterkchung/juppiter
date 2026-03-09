# Reference

Complete API and configuration reference for juppiter.

## Available References

- [ROS Topics](ros-topics.md) - Complete topic catalog with types and rates
- [Configuration Reference](configuration-reference.md) - All YAML options documented
- [Package Overview](package-overview.md) - Source package structure and APIs

## Quick Reference

### Topic Quick Lookup

| Topic | Type | Purpose |
|-------|------|---------|
| `/perception/odom` | `nav_msgs/Odometry` | Fused pose estimate |
| `/perception/health` | `std_msgs/String` (JSON) | System health |
| `/perception/mode` | `std_msgs/String` | Operational mode |
| `/lidar/points` | `sensor_msgs/PointCloud2` | LiDAR data |
| `/imu/data` | `sensor_msgs/Imu` | IMU measurements |

### Common Commands

```bash
# Launch simulation
ros2 launch gazebo_lunar_sim simulation.launch.py

# Monitor health
ros2 topic echo /perception/health

# Check build status
colcon build --symlink-install && colcon test-result

# Enter container
docker compose -f docker/docker-compose.yml exec dev bash
```

### Configuration Quick Lookup

All YAML configs in `config/`:
- `compute_profiles/*.yaml` - Hardware profiles
- `sensors/*.yaml` - Sensor calibration
- `fusion/*.yaml` - Fusion algorithm settings

## Getting Started

New to juppiter? Start with:
1. [Installation Guide](../getting-started/installation.md)
2. [Running Simulation](../getting-started/running-simulation.md)
3. [Architecture Overview](../architecture/overview.md)
