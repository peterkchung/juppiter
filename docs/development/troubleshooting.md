# Troubleshooting

Common issues and solutions for juppiter development.

## Quick Diagnostics

```bash
# Check system health
ros2 topic echo /perception/health

# List all nodes
ros2 node list

# Check node info
ros2 node info /sensor_bridge_node

# Monitor topics
ros2 topic list
ros2 topic hz /perception/odom
```

## Common Issues

### Build Failures

#### Missing Dependencies

**Symptom:**
```
CMake Error at CMakeLists.txt:10 (find_package):
  Could not find a package configuration file provided by
  "sensor_interfaces" with any of the following names:
```

**Solution:**
```bash
# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -y

# Or manually install
apt-get install ros-kilted-sensor-msgs
```

#### Circular Dependencies

**Symptom:**
```
ERROR: Circular dependency detected
```

**Solution:**
Check `package.xml` files for circular `depend` tags. Common issue:
- Package A depends on Package B
- Package B depends on Package A

Fix by refactoring or moving shared code to a third package.

#### Compiler Errors

**Symptom:**
```
error: 'MyClass' has not been declared
```

**Solution:**
- Check include paths
- Verify header guards are correct
- Ensure class is in correct namespace

```bash
# Clean build
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Container Issues

#### Cannot Enter Container

**Symptom:**
```
Error response from daemon: Container is not running
```

**Solution:**
```bash
# Start container
docker compose -f docker/docker-compose.yml up -d

# Or rebuild if corrupted
docker compose -f docker/docker-compose.yml down -v
docker compose -f docker/docker-compose.yml build --no-cache
docker compose -f docker/docker-compose.yml up -d
```

#### GUI Applications Fail

**Symptom:**
Gazebo or RViz won't open

**Solutions:**

**Linux:**
```bash
# Allow X11 connections
xhost +local:docker

# Or specify display
export DISPLAY=:1
```

**Windows (WSL2):**
```bash
# Verify WSLg
wsl.exe --version

# Check WSLg is installed
ls /mnt/wslg/

# If not, install WSLg from Microsoft Store
```

**macOS:**
```bash
# Install XQuartz
brew install --cask xquartz

# Enable network connections
# XQuartz → Preferences → Security → Allow connections from network clients

# Start XQuartz
open -a XQuartz
```

### Runtime Issues

#### No Odometry Published

**Symptom:**
```bash
ros2 topic echo /perception/odom
# (no output)
```

**Diagnosis:**
```bash
# Check if simulation is running
ros2 node list | grep gazebo

# Check sensor topics
ros2 topic list | grep lidar
ros2 topic hz /lidar/points

# Check health
ros2 topic echo /perception/health
```

**Solutions:**
1. Simulation not started:
   ```bash
   ros2 launch gazebo_lunar_sim simulation.launch.py
   ```

2. Gazebo paused:
   - Press spacebar in Gazebo to unpause

3. Topics not publishing:
   ```bash
   # Check for errors in logs
   ros2 node info /gazebo_lunar_sim
   ```

#### Health Score Too Low

**Symptom:**
```
{
  "overall_health": 0.45,
  "mode": "safe_stop"
}
```

**Causes:**
1. Sensor not publishing (check `ros2 topic hz`)
2. Time sync issues (check `/perception/health` details)
3. Sensor physically disconnected

**Solution:**
```bash
# Check individual sensor health
ros2 topic echo /perception/health

# Look for flags like "STALE", "SYNC_VIOLATION"

# Restart simulation
# If hardware, check connections
```

#### Mode Stuck in Degraded

**Symptom:**
Mode won't return to `nominal` after fault clears

**Solutions:**
```bash
# Check if fault is actually cleared
ros2 topic echo /perception/faults

# Health may be recovering slowly
# Wait 10-20 seconds for full recovery

# Force reset (if needed)
ros2 service call /sensor_bridge/reset std_srvs/srv/Trigger
```

### Sensor Issues

#### Plugin Not Loading

**Symptom:**
```
[ERROR] Failed to load my_sensor_provider::MySensorDriver
```

**Diagnosis:**
```bash
# List available plugins
ros2 pkg plugins --type sensor_interfaces::SensorDriver

# Verify plugin file exists
ls install/my_sensor_provider/share/my_sensor_provider/

# Check for missing dependencies
ldd install/my_sensor_provider/lib/libmy_sensor_provider.so
```

**Solutions:**
1. Package not built:
   ```bash
   colcon build --packages-select my_sensor_provider
   ```

2. Plugin XML not found:
   - Verify `pluginlib_export_plugin_description_file` in CMakeLists.txt
   - Check file path in XML

3. Symbol not found:
   - Ensure class is properly exported
   - Check constructor is public

#### Wrong Topic Names

**Symptom:**
Expected topic not found

**Solution:**
```bash
# List all topics
ros2 topic list

# Remap if needed
ros2 run my_package my_node --ros-args --remap old_topic:=new_topic

# Or in launch file:
Node(
    package='my_package',
    executable='my_node',
    remappings=[('old_topic', 'new_topic')]
)
```

### Performance Issues

#### High CPU Usage

**Symptom:**
System slow, fans loud, high `top` output

**Diagnosis:**
```bash
# Check which process
htop

# Or ROS-specific
ros2 run rqt_top rqt_top
```

**Solutions:**
1. Switch to lower compute profile:
   ```bash
   ros2 launch gazebo_lunar_sim simulation.launch.py compute_profile:=low_power
   ```

2. Reduce sensor rates in config

3. Check for infinite loops in custom code

#### High Memory Usage

**Symptom:**
Container or system runs out of memory

**Diagnosis:**
```bash
# Check memory
docker stats
free -h
```

**Solutions:**
1. Limit container memory:
   ```yaml
   # docker-compose.yml
   services:
     dev:
       mem_limit: 4g
   ```

2. Reduce point cloud density in config

3. Restart container periodically for long runs

### Test Failures

#### FDIIR Tests Failing

**Symptom:**
```
[FDIIR-TEST] ASSERTION FAILED: mode != degraded_lio
```

**Diagnosis:**
```bash
# Run with verbose output
ros2 launch gazebo_lunar_sim simulation.launch.py \
  scenario:=lidar_dropout \
  verbose:=true

# Check logs
tail -f log/latest_build/gazebo_lunar_sim/stdout.log
```

**Common Causes:**
1. Detection too slow - adjust thresholds
2. Recovery not happening - check provider restart logic
3. Position drift too high - tune fusion weights

#### Unit Tests Failing

**Symptom:**
```
[colcon-test] test_health_monitor: 2 tests, 1 failure
```

**Solution:**
```bash
# Run specific test with details
colcon test --packages-select sensor_core \
  --ctest-args -R test_health_monitor -V

# Debug with GDB
gdb -ex run --args build/sensor_core/test_health_monitor
```

### Git Issues

#### Merge Conflicts

**Symptom:**
```
CONFLICT (content): Merge conflict in src/sensor_core/src/health_monitor.cpp
```

**Solution:**
```bash
# See conflicts
git status

# Edit files to resolve
# Look for <<<<<<<, =======, >>>>>>> markers

# Mark as resolved
git add <resolved-file>
git commit
```

#### Large File Issues

**Symptom:**
```
remote: error: File data/dataset.bag is 250.00 MB; this exceeds GitHub's file size limit of 100.00 MB
```

**Solution:**
```bash
# Use Git LFS for large files
git lfs track "*.bag"
git lfs track "*.pcd"
git add .gitattributes

# Or add to .gitignore
echo "*.bag" >> .gitignore
```

### Docker Issues

#### Image Build Fails

**Symptom:**
```
ERROR: failed to solve: rpc error: code = Unknown desc = failed to solve
```

**Solutions:**
```bash
# Clean Docker cache
docker system prune -a

# Rebuild without cache
docker compose -f docker/docker-compose.yml build --no-cache

# Check Dockerfile syntax
docker build -f docker/Dockerfile.dev .
```

#### Permission Denied

**Symptom:**
```
permission denied while trying to connect to the Docker daemon
```

**Solution:**
```bash
# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker

# Or use sudo (not recommended for regular use)
sudo docker compose -f docker/docker-compose.yml up -d
```

## Debug Mode

Enable verbose logging:

```bash
# ROS 2 verbose
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"

# Launch with debug logging
ros2 launch gazebo_lunar_sim simulation.launch.py --log-level debug
```

## Getting Help

If issue persists:

1. **Check existing issues**: https://github.com/opencode/juppiter/issues
2. **Create minimal reproduction**: 
   - Small code snippet showing issue
   - Exact commands to reproduce
   - Expected vs actual behavior
3. **Include diagnostics**:
   ```bash
   # System info
   uname -a
   docker --version
   
   # ROS info
   ros2 doctor
   
   # Logs
   tail -n 100 log/latest_build/ERROR.log
   ```

## Next Topics

- [Testing](testing.md) - Debugging test failures
- [Contributing](contributing.md) - Getting help with PRs
