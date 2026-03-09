# Configuration Guide

juppiter is configured entirely via YAML files, allowing behavior to be changed without recompiling. This guide explains the configuration system and common modifications.

## Configuration Hierarchy

```
config/
├── compute_profiles/      # Hardware-specific settings
│   ├── dev.yaml
│   ├── edge.yaml
│   └── low_power.yaml
├── sensors/              # Sensor calibration
│   ├── simulated_livox.yaml
│   ├── simulated_stereo.yaml
│   └── simulated_imu.yaml
└── fusion/               # Fusion algorithm settings
    ├── weights_nominal.yaml
    └── health_thresholds.yaml
```

## Compute Profile Configuration

The compute profile determines which estimators run and their resource usage.

### dev.yaml (Workstation)

```yaml
compute_profile:
  name: "dev"
  hardware_target: "workstation"
  
  # Sensor providers to load
  providers:
    - gazebo_lunar_sim::GazeboLidarProvider
    - gazebo_lunar_sim::GazeboCameraProvider
    - gazebo_lunar_sim::GazeboImuProvider
  
  # Estimator selection
  estimators:
    lio:
      implementation: "fast_lio2"
      enabled: true
      # CPU limit prevents one estimator from starving others
      max_cpu_percent: 40.0
      
    vio:
      implementation: "orb_slam3"
      enabled: true
      max_cpu_percent: 30.0
      
    kinetic:
      enabled: true
      max_cpu_percent: 5.0
      # robot_localization EKF configuration
      ekf_config:
        frequency: 50.0
        odom0: "/wheel/odometry"
        odom0_config: [true, true, false,  # x, y, z
                       false, false, false,  # roll, pitch, yaw
                       false, false, false,  # vx, vy, vz
                       false, false, false,  # vroll, vpitch, vyaw
                       false, false, false]  # ax, ay, az
        imu0: "/imu/data"
        imu0_config: [false, false, false,
                      true, true, true,     # roll, pitch, yaw
                      false, false, false,
                      true, true, true,     # vroll, vpitch, vyaw
                      true, true, true]     # ax, ay, az
  
  # Sensor health thresholds
  health_thresholds:
    nominal: 0.75      # Health ≥ 0.75 = nominal mode
    degraded: 0.55     # 0.55 ≤ Health < 0.75 = degraded
    safe_stop: 0.0     # Health < 0.55 = safe_stop
    
  # Monitoring rate
  monitoring_rate_hz: 10.0
```

### edge.yaml (Raspberry Pi 5 / Jetson)

```yaml
compute_profile:
  name: "edge"
  hardware_target: "raspberry_pi_5"
  
  providers:
    - gazebo_lunar_sim::GazeboLidarProvider
    - gazebo_lunar_sim::GazeboCameraProvider
    - gazebo_lunar_sim::GazeboImuProvider
  
  estimators:
    lio:
      implementation: "dlio"  # Faster than FAST-LIO2
      enabled: true
      max_cpu_percent: 35.0
      # Downsample point clouds to reduce compute
      preprocessing:
        voxel_filter:
          enabled: true
          leaf_size: 0.1  # 10cm voxels
      
    vio:
      implementation: "openvins"  # Optimized for ARM
      enabled: true
      max_cpu_percent: 25.0
      # Limit features to reduce computation
      max_features: 150  # vs 300 in dev
      
    kinetic:
      enabled: true
      max_cpu_percent: 5.0
      ekf_config:
        frequency: 30.0  # Reduced from 50Hz
```

### low_power.yaml (Raspberry Pi 4B)

```yaml
compute_profile:
  name: "low_power"
  hardware_target: "raspberry_pi_4b"
  
  providers:
    - gazebo_lunar_sim::GazeboLidarProvider
    - gazebo_lunar_sim::GazeboImuProvider
  
  estimators:
    lio:
      implementation: "dlio"
      enabled: true
      max_cpu_percent: 45.0
      preprocessing:
        voxel_filter:
          enabled: true
          leaf_size: 0.15  # More aggressive downsampling
      # Limit update rate
      max_update_rate_hz: 10.0
      
    vio:
      enabled: false  # Disabled entirely
      
    kinetic:
      enabled: true
      max_cpu_percent: 10.0
      ekf_config:
        frequency: 20.0  # Further reduced
  
  monitoring_rate_hz: 5.0  # Less frequent monitoring
```

## Fusion Configuration

### weights_nominal.yaml

```yaml
fusion_core:
  # Nominal fusion weights (must sum to 1.0)
  weights:
    lio: 0.50        # Primary estimator
    vio: 0.35        # Secondary estimator
    kinetic: 0.15    # Baseline
  
  # Weight adjustment limits
  min_weight: 0.05   # Never below 5%
  max_weight: 0.90   # Never above 90%
  
  # Health-to-weight mapping
  # When health drops, weight is scaled linearly
  health_weight_mapping:
    health_threshold: 0.5    # Below this, start reducing
    min_scale: 0.2         # At health=0, scale weight to 20%
  
  # Consistency checking
  consistency:
    enabled: true
    mahalanobis_threshold: 3.0  # Standard deviations
    outlier_rejection: true
    
  # Covariance scaling based on weight
  covariance_scale_factor: 2.0  # Lower weight → higher covariance
  
  # EKF process noise
  process_noise:
    position: 0.01      # m²/s
    orientation: 0.001  # rad²/s
    velocity: 0.1       # (m/s)²/s
    angular_velocity: 0.01  # (rad/s)²/s
```

### health_thresholds.yaml

```yaml
health_monitoring:
  # Monitoring rate
  rate_hz: 10.0
  
  # System mode thresholds
  thresholds:
    nominal_min: 0.75
    degraded_min: 0.55
  
  # Time synchronization
  time_sync:
    max_mean_skew_ms: 5.0     # Mean offset
    max_p95_skew_ms: 10.0     # 95th percentile
    max_sync_skew_ms: 20.0    # Any single offset
    
  # Stale data detection
  stale_data:
    # Multiplier on expected period
    # If sensor publishes at 10Hz (100ms period),
    # stale after 2 * 100ms = 200ms
    timeout_multiplier: 2.0
    
  # Rate monitoring
  rate_monitoring:
    # Alert if rate drops below 80% of expected
    min_rate_percentage: 0.8
    
  # Covariance bounds
  covariance:
    max_position_std_m: 1.0
    max_orientation_std_deg: 10.0
    
  # Individual sensor health weights
  # How much each sensor contributes to overall health
  sensor_weights:
    lidar: 0.35
    left_camera: 0.20
    right_camera: 0.20
    imu: 0.25
```

## Sensor Configuration

### simulated_livox.yaml

```yaml
livox_lidar:
  name: "livox_mid360"
  frame_id: "livox_frame"
  
  # Simulation parameters
  simulation:
    range_min: 0.1
    range_max: 100.0
    horizontal_fov: 360.0
    vertical_fov: 77.2
    vertical_resolution: 0.5
    
  # Publish rate
  publish_rate_hz: 10.0
  
  # Topic configuration
  topic: "/lidar/points"
  
  # Add noise to simulation
  noise:
    enabled: true
    stddev_range: 0.01  # 1cm
    
  # Time synchronization
  time_sync:
    reference_clock: "ros"  # Use ROS time
    offset_s: 0.0
```

### simulated_stereo.yaml

```yaml
stereo_camera:
  left:
    name: "left_camera"
    frame_id: "left_camera_optical_frame"
    topic: "/stereo/left/image_raw"
    info_topic: "/stereo/left/camera_info"
    
  right:
    name: "right_camera"
    frame_id: "right_camera_optical_frame"
    topic: "/stereo/right/image_raw"
    info_topic: "/stereo/right/camera_info"
  
  # Common settings
  resolution:
    width: 1280
    height: 720
  
  publish_rate_hz: 20.0
  
  # Stereo configuration
  baseline_m: 0.12  # 12cm baseline
  
  # Intrinsics (can be calibrated)
  calibration:
    left:
      K: [640.0, 0.0, 640.0,   # fx, 0, cx
          0.0, 640.0, 360.0,   # 0, fy, cy
          0.0, 0.0, 1.0]       # 0, 0, 1
      D: [0.0, 0.0, 0.0, 0.0, 0.0]  # k1, k2, p1, p2, k3
    right:
      K: [640.0, 0.0, 640.0,
          0.0, 640.0, 360.0,
          0.0, 0.0, 1.0]
      D: [0.0, 0.0, 0.0, 0.0, 0.0]
  
  # Time synchronization
  time_sync:
    reference_clock: "ros"
    offset_s: 0.0
```

### simulated_imu.yaml

```yaml
imu:
  name: "imu_9axis"
  frame_id: "imu_frame"
  
  topic: "/imu/data"
  publish_rate_hz: 200.0  # High rate for good integration
  
  # Noise characteristics
  noise:
    # Accelerometer
    accel:
      noise_stddev: 0.01  # m/s²
      bias_stddev: 0.005
      
    # Gyroscope
    gyro:
      noise_stddev: 0.001  # rad/s
      bias_stddev: 0.0005
      
    # Magnetometer (if available)
    mag:
      noise_stddev: 0.001  # Gauss
      
  # Time synchronization (reference clock)
  time_sync:
    reference_clock: "imu"  # IMU is often the reference
    is_master: true
```

## Modifying Configuration

### Changing Compute Profile at Runtime

```bash
# Edit the launch file or pass as argument
ros2 launch gazebo_lunar_sim simulation.launch.py compute_profile:=edge
```

### Creating Custom Profiles

1. Copy existing profile:
```bash
cp config/compute_profiles/edge.yaml config/compute_profiles/custom.yaml
```

2. Edit settings in `custom.yaml`

3. Launch with custom profile:
```bash
ros2 launch gazebo_lunar_sim simulation.launch.py compute_profile:=custom
```

### Configuration Reload

Configuration is loaded at startup. To apply changes:

```bash
# Stop simulation
Ctrl+C

# Edit config file
nano config/fusion/weights_nominal.yaml

# Restart
ros2 launch gazebo_lunar_sim simulation.launch.py compute_profile:=dev
```

## Configuration Validation

The system validates configuration on load:

```bash
# Check for errors in logs
ros2 launch gazebo_lunar_sim simulation.launch.py 2>&1 | grep -i error

# Expected validations:
# - Weights sum to 1.0
# - Required fields present
# - File paths exist
# - Numeric ranges valid
```

## Environment Variables

Override configuration with environment variables:

```bash
export JUPPITER_CONFIG_PATH=/custom/path/to/config
export JUPPITER_LOG_LEVEL=debug
export JUPPITER_COMPUTE_PROFILE=edge
```

## Best Practices

1. **Version Control**: Track config changes in git
2. **Testing**: Test profile changes in simulation before hardware
3. **Documentation**: Comment non-obvious settings
4. **Gradual Changes**: Change one parameter at a time
5. **Backup**: Keep working configs as backups

## Troubleshooting

### Config Not Loading

```bash
# Check file paths
ls -la config/compute_profiles/dev.yaml

# Verify YAML syntax
python3 -c "import yaml; yaml.safe_load(open('config/compute_profiles/dev.yaml'))"
```

### Weights Don't Sum to 1.0

```bash
# Check error in logs
grep "weights" log/latest_build/*.log

# Verify in config
python3 -c "import yaml; d=yaml.safe_load(open('config/fusion/weights_nominal.yaml')); print(sum(d['fusion_core']['weights'].values()))"
```

## Next Topics

- [Sensor Integration](sensor-integration.md) - Add custom sensors
- [FDIIR Testing](fdiir-testing.md) - Test configurations
