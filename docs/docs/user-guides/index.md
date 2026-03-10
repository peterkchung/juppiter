# User Guides

Practical guides for configuring, extending, and testing juppiter.

## Available Guides

- [Configuration](configuration.md) - Understanding and modifying YAML configs
- [Sensor Integration](sensor-integration.md) - Adding custom sensors
- [FDIIR Testing](fdiir-testing.md) - Running fault injection scenarios

## Quick Reference

### Common Tasks

**Change Compute Profile:**
```bash
ros2 launch gazebo_lunar_sim simulation.launch.py compute_profile:=edge
```

**Monitor System Health:**
```bash
ros2 topic echo /perception/health
```

**Add a New Sensor:**
1. Create plugin package
2. Implement `SensorDriver` interface
3. Add to configuration
4. Test in simulation

**Run FDIIR Tests:**
```bash
ros2 launch gazebo_lunar_sim simulation.launch.py scenario:=lidar_dropout
```

## Getting Help

- Architecture questions → [Architecture section](../architecture/index.md)
- Development workflow → [Development section](../development/index.md)
- API reference → [Reference section](../reference/index.md)
