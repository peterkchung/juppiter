# Running Your First Simulation

This guide walks you through launching the Gazebo lunar simulation and verifying juppiter is working correctly.

## Launch the Simulation

From inside the container (your prompt shows `root@juppiter-dev:/ws#`):

```bash
ros2 launch gazebo_lunar_sim simulation.launch.py compute_profile:=dev
```

This command:
- Starts Gazebo Harmonic with lunar lighting environment
- Spawns a simulated lunar rover with differential drive
- Activates all sensors: LiDAR, stereo cameras, and IMU
- Launches the kinetic estimator (wheel odometry + IMU EKF)

## What You Should See

### Gazebo Simulator

A window opens showing:
- A flat lunar terrain surface
- A rover model with:
  - Livox Mid-360 LiDAR (cylindrical sensor on top)
  - Stereo camera pair (front-facing)
  - IMU (internal to chassis)
- Shadows and lighting simulating lunar conditions

### Terminal Output

You should see initialization messages like:
```
[ignition-1] [INFO] Loading world lunar_flat.sdf
[gazebo_lunar_sim-2] [INFO] Spawning rover model
[kinetic_estimator-3] [INFO] Kinetic estimator initialized
[sensor_core-4] [INFO] Health monitoring active
```

## Verify Sensor Topics

In a new terminal (outside the container), enter the container again:

```bash
docker compose -f docker/docker-compose.yml exec dev bash
```

### Check Published Topics

```bash
ros2 topic list
```

Expected output includes:
```
/lidar/points
/stereo/left/image_raw
/stereo/right/image_raw
/stereo/left/camera_info
/stereo/right/camera_info
/imu/data
/wheel/odometry
/perception/odom
/perception/mode
/perception/health
```

### Verify Sensor Rates

Check that sensors are publishing at expected frequencies:

```bash
# LiDAR: ~10 Hz
ros2 topic hz /lidar/points

# Cameras: ~20 Hz
ros2 topic hz /stereo/left/image_raw

# IMU: ~200 Hz
ros2 topic hz /imu/data

# Wheel odometry: ~50 Hz
ros2 topic hz /wheel/odometry
```

## Monitor System Health

The perception pipeline publishes health and mode information:

```bash
# View current system health (JSON format)
ros2 topic echo /perception/health
```

Example output:
```json
{
  "data": "{\"overall_health\": 0.95, \"mode\": \"nominal\", \"active_sensors\": 4, \"faults\": []}"
}
```

```bash
# View current mode
ros2 topic echo /perception/mode
```

Modes include:
- `nominal` - All sensors healthy
- `degraded_lio` - LiDAR issues, relying more on VIO
- `degraded_vio` - Camera issues, relying more on LIO
- `safe_stop` - Critical failure, stopping recommended

```bash
# View estimator fusion weights
ros2 topic echo /perception/estimator_weights
```

Shows current confidence distribution across LIO, VIO, and Kinematic estimators.

## Visualize with RViz

Open RViz2 with the pre-configured display:

```bash
rviz2 -d /ws/src/gazebo_lunar_sim/config/rviz.rviz
```

RViz displays:
- Point cloud from LiDAR
- Camera images (left/right)
- Robot model and TF frames
- Odometry path
- Sensor health status

## Drive the Rover

The simulated rover accepts velocity commands:

```bash
# Forward motion
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'

# Rotation
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}'

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'
```

## Common Issues

### Gazebo Doesn't Open

**Symptom:** Command runs but no window appears

**Solutions:**
- **Linux:** `export DISPLAY=:1` then retry
- **Windows WSL:** Ensure WSLg is enabled in Docker Desktop settings
- **macOS:** Use `xhost +` to allow X11 connections

### Low Frame Rate

**Symptom:** Gazebo runs slowly

**Solution:** Reduce compute profile complexity:
```bash
ros2 launch gazebo_lunar_sim simulation.launch.py compute_profile:=low_power
```

### Topics Not Publishing

**Symptom:** `ros2 topic hz` shows 0 Hz

**Solution:** Check if simulation is paused (press space in Gazebo to unpause)

## Next Steps

Now that you have the simulation running:

- [Learn about the architecture](../architecture/overview.md)
- [Configure compute profiles](../user-guides/configuration.md)
- [Run FDIIR test scenarios](../user-guides/fdiir-testing.md)

## Quick Reference

```bash
# Launch simulation with different profiles
ros2 launch gazebo_lunar_sim simulation.launch.py compute_profile:=dev     # Full
ros2 launch gazebo_lunar_sim simulation.launch.py compute_profile:=edge    # Optimized
ros2 launch gazebo_lunar_sim simulation.launch.py compute_profile:=low_power  # Minimal

# Monitor health
ros2 topic echo /perception/health

# View odometry
ros2 topic echo /perception/odom
```
