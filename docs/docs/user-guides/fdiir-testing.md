# FDIIR Testing Guide

The FDIIR (Fault Detection, Isolation, Identification, Recovery) framework is validated through automated test scenarios. This guide explains how to run and interpret these tests.

## What is FDIIR Testing?

FDIIR tests inject faults into the simulation to verify the system responds correctly:

```
Test Scenario → Fault Injection → System Response → Validation

Example:
  LiDAR Dropout → Stop publishing → Switch to VIO → Verify continuity
```

## Available Test Scenarios

Located in `test/fdiir_scenarios/`:

| Scenario | Fault Injected | Expected Response | Success Criteria |
|----------|---------------|-------------------|------------------|
| **lidar_dropout** | LiDAR stops publishing | VIO becomes primary | No position drift > 0.5m |
| **camera_degradation** | Camera rate drops to 5Hz | LIO weight increases | Mode = degraded_vio |
| **wheel_slip** | Wheel odometry error | IMU fusion compensates | Position error < 0.3m |
| **cascading_fault** | Multiple sensor failures | Safe_stop mode | Autonomous stop within 2s |

## Running Tests

### Basic Test Execution

```bash
# Test LiDAR dropout handling
ros2 launch gazebo_lunar_sim simulation.launch.py scenario:=lidar_dropout

# Test camera degradation
ros2 launch gazebo_lunar_sim simulation.launch.py scenario:=camera_degradation

# Test wheel slip
ros2 launch gazebo_lunar_sim simulation.launch.py scenario:=wheel_slip

# Test cascading failures
ros2 launch gazebo_lunar_sim simulation.launch.py scenario:=cascading_fault
```

### Test Parameters

All scenarios accept parameters:

```bash
ros2 launch gazebo_lunar_sim simulation.launch.py \
  scenario:=lidar_dropout \
  compute_profile:=edge \
  duration:=60.0 \
  record:=true
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `compute_profile` | `dev` | Hardware profile to use |
| `duration` | 60.0 | Test duration in seconds |
| `record` | false | Record rosbag for analysis |
| `verbose` | false | Detailed logging |

## Test Scenario Details

### lidar_dropout.yaml

```yaml
scenario:
  name: "lidar_dropout"
  description: "Test handling of LiDAR communication loss"
  
  # Fault injection schedule
  faults:
    - time: 10.0  # seconds
      action: "disable_publishing"
      target: "lidar"
      duration: 20.0  # seconds
      
    - time: 35.0
      action: "enable_publishing"
      target: "lidar"
  
  # Expected behavior assertions
  assertions:
    # Mode transitions
    - time: 10.5
      check: "mode"
      expected: "degraded_lio"
      tolerance: 1.0  # seconds
      
    - time: 35.5
      check: "mode"
      expected: "nominal"
      tolerance: 1.0
      
    # Health scores
    - time: 15.0
      check: "health"
      min_value: 0.55
      max_value: 0.75
      
    - time: 40.0
      check: "health"
      min_value: 0.75
      
    # Position continuity (no big jumps)
    - time_range: [10.0, 30.0]
      check: "position_drift"
      max_drift_m: 0.5
      
    # Fusion weights
    - time: 15.0
      check: "lio_weight"
      expected: 0.0
      tolerance: 0.05
      
    - time: 15.0
      check: "vio_weight"
      min_value: 0.70
```

### camera_degradation.yaml

```yaml
scenario:
  name: "camera_degradation"
  description: "Test handling of reduced camera frame rate"
  
  faults:
    - time: 5.0
      action: "reduce_rate"
      target: "cameras"
      new_rate_hz: 5.0  # Down from 20Hz
      duration: 30.0
      
    - time: 40.0
      action: "restore_rate"
      target: "cameras"
      
  assertions:
    - time: 10.0
      check: "mode"
      expected: "degraded_vio"
      
    - time: 10.0
      check: "vio_covariance"
      max_value: 2.0  # Higher uncertainty expected
      
    - time: 45.0
      check: "mode"
      expected: "nominal"
```

### wheel_slip.yaml

```yaml
scenario:
  name: "wheel_slip"
  description: "Test handling of wheel odometry errors (regolith simulation)"
  
  faults:
    - time: 5.0
      action: "add_noise"
      target: "wheel_odometry"
      noise_stddev_m: 0.5  # High slip
      duration: 20.0
      
  assertions:
    # Kinetic estimator health should drop
    - time: 10.0
      check: "kinetic_health"
      max_value: 0.6
      
    # But overall position should remain reasonable
    - time_range: [5.0, 25.0]
      check: "position_error"
      max_error_m: 0.3
      
    # Fusion should reduce kinetic weight
    - time: 15.0
      check: "kinetic_weight"
      max_value: 0.10
```

### cascading_fault.yaml

```yaml
scenario:
  name: "cascading_fault"
  description: "Test safe_stop with multiple concurrent failures"
  
  faults:
    # LiDAR fails first
    - time: 5.0
      action: "disable_publishing"
      target: "lidar"
      duration: 60.0  # Doesn't recover
      
    # Camera fails shortly after
    - time: 10.0
      action: "disable_publishing"
      target: "cameras"
      duration: 60.0
      
    # Wheel encoder error
    - time: 15.0
      action: "add_noise"
      target: "wheel_odometry"
      noise_stddev_m: 1.0
      duration: 60.0
      
  assertions:
    # Health degrades progressively
    - time: 8.0
      check: "health"
      min_value: 0.55
      max_value: 0.75
      
    - time: 18.0
      check: "health"
      max_value: 0.55
      
    - time: 20.0
      check: "mode"
      expected: "safe_stop"
      
    # Safe stop initiated
    - time: 22.0
      check: "safe_stop_triggered"
      expected: true
```

## Test Output

### Console Output

```
[FDIIR-TEST] Starting scenario: lidar_dropout
[FDIIR-TEST] T+0.0s: Simulation running, all sensors nominal
[FDIIR-TEST] T+10.0s: Injecting fault - disabling LiDAR publishing
[FDIIR-TEST] T+10.2s: ✓ Detected LiDAR dropout
[FDIIR-TEST] T+10.5s: ✓ Mode transitioned to degraded_lio
[FDIIR-TEST] T+10.5s: ✓ LIO weight reduced to 0.0
[FDIIR-TEST] T+10.5s: ✓ VIO weight increased to 0.85
[FDIIR-TEST] T+15.0s: ✓ Health score 0.68 (within expected range)
[FDIIR-TEST] T+25.0s: ✓ Position drift 0.23m (within 0.5m threshold)
[FDIIR-TEST] T+35.0s: Restoring LiDAR publishing
[FDIIR-TEST] T+35.5s: ✓ LiDAR recovered
[FDIIR-TEST] T+35.5s: ✓ Mode transitioned to nominal
[FDIIR-TEST] T+35.5s: ✓ Weights restored
[FDIIR-TEST] T+40.0s: ✓ Health score 0.91 (recovered)
[FDIIR-TEST] 
[FDIIR-TEST] =====================================
[FDIIR-TEST] Test Summary: lidar_dropout
[FDIIR-TEST] =====================================
[FDIIR-TEST] Total assertions: 8
[FDIIR-TEST] Passed: 8
[FDIIR-TEST] Failed: 0
[FDIIR-TEST] Duration: 40.5s
[FDIIR-TEST] 
[FDIIR-TEST] ✓ TEST PASSED
[FDIIR-TEST] =====================================
```

### Rosbag Recording

When `record:=true`:

```bash
# Recorded topics
/perception/odom
/perception/health
/perception/mode
/perception/faults
/perception/estimator_weights
/lidar/points
/vio/odom
/kinematic/odom
/cmd_vel

# Analyze later
ros2 bag play rosbag2_2024_01_15/
```

## Interpreting Results

### Pass Criteria

A test passes if all assertions succeed:
- Mode transitions occur within tolerance windows
- Health scores stay within bounds
- Position drift remains acceptable
- Fusion weights adjust correctly

### Fail Criteria

Tests fail if:
- Position drift exceeds thresholds
- Mode transitions don't occur
- Recovery doesn't happen
- System crashes or hangs

### Common Failures

| Failure | Cause | Solution |
|---------|-------|----------|
| Position drift too high | Fusion not compensating | Check fusion weights |
| Mode transition too slow | Detection delay | Tune fault detection |
| Test timeout | System hang | Check for blocking calls |
| Recovery failed | Sensor didn't restart | Check provider restart logic |

## Creating Custom Scenarios

### 1. Create YAML File

```bash
cp test/fdiir_scenarios/lidar_dropout.yaml \
   test/fdiir_scenarios/my_scenario.yaml
```

### 2. Define Faults

Available fault actions:
- `disable_publishing` - Stop sensor from publishing
- `enable_publishing` - Resume publishing
- `reduce_rate` - Lower publication rate
- `restore_rate` - Return to original rate
- `add_noise` - Inject noise into data
- `delay_messages` - Add latency
- `drop_packets` - Random message loss

### 3. Define Assertions

Available checks:
- `mode` - Current system mode
- `health` - Overall health score
- `sensor_health` - Individual sensor health
- `position_drift` - Position change over time
- `position_error` - Error vs ground truth
- `lio_weight` - LIO fusion weight
- `vio_weight` - VIO fusion weight
- `kinetic_weight` - Kinetic fusion weight

### 4. Run Your Scenario

```bash
ros2 launch gazebo_lunar_sim simulation.launch.py scenario:=my_scenario
```

## Integration with CI

Tests run automatically in GitHub Actions:

```yaml
# .github/workflows/fdiir-tests.yml
name: FDIIR Tests
on:
  push:
    branches: [develop, main]
  pull_request:
    branches: [develop]

jobs:
  fdiir-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Run all FDIIR scenarios
        run: |
          docker compose -f docker/docker-compose.yml up -d
          docker compose -f docker/docker-compose.yml exec dev bash -c "
            colcon build
            ros2 launch gazebo_lunar_sim fdiir_test_runner.launch.py
          "
```

## Manual Validation

### Step-by-Step Verification

```bash
# 1. Start simulation
ros2 launch gazebo_lunar_sim simulation.launch.py

# 2. In another terminal, monitor health
ros2 topic echo /perception/health

# 3. Stop LiDAR publishing (simulate failure)
ros2 topic pub /lidar/points sensor_msgs/msg/PointCloud2 '{}'
# This overrides the real publisher with empty messages

# 4. Watch mode transition
ros2 topic echo /perception/mode
# Should show: "degraded_lio"

# 5. Check fusion weights
ros2 topic echo /perception/estimator_weights
# Should show reduced LIO weight

# 6. Resume LiDAR
# Stop the override publisher

# 7. Verify recovery
# Mode should return to "nominal"
```

## Metrics and Analysis

### Key Metrics

| Metric | Target | Measured |
|--------|--------|----------|
| Detection latency | < 1s | `ros2 topic echo /perception/faults` |
| Mode transition time | < 2s | Check mode topic timestamps |
| Position continuity | < 0.5m | Calculate from /perception/odom |
| Recovery time | < 3s | Time to return to nominal |

### Generating Reports

```bash
# Run all scenarios and generate report
ros2 launch gazebo_lunar_sim fdiir_test_runner.launch.py \
  output_dir:=./fdiir_reports

# View HTML report
firefox ./fdiir_reports/index.html
```

## Next Topics

- [Configuration Guide](configuration.md) - Tune fault detection thresholds
- [Architecture/Sensor Core](../architecture/sensor-core.md) - FDIIR framework details
- [Development/Testing](../development/testing.md) - Unit and integration tests
