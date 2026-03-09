# Configuration Reference

Complete reference for all YAML configuration files in juppiter.

## Configuration Files

```
config/
├── compute_profiles/           # Hardware-specific profiles
│   ├── dev.yaml               # Workstation
│   ├── edge.yaml              # Raspberry Pi 5 / Jetson
│   └── low_power.yaml         # Raspberry Pi 4B
├── sensors/                    # Sensor calibration
│   ├── simulated_livox.yaml
│   ├── simulated_stereo.yaml
│   └── simulated_imu.yaml
└── fusion/                     # Fusion algorithm settings
    ├── weights_nominal.yaml
    └── health_thresholds.yaml
```

## Compute Profile Schema

### Root Structure

```yaml
compute_profile:
  name: string                # Profile name (dev, edge, low_power)
  hardware_target: string     # Target hardware description
  providers: list             # List of SensorDriver plugins to load
  estimators: object          # Estimator configurations
  health_thresholds: object   # Health score thresholds
  monitoring_rate_hz: float   # Health monitoring frequency
```

### Provider Configuration

```yaml
providers:
  - string  # Fully qualified class name
```

**Examples:**
```yaml
providers:
  - gazebo_lunar_sim::GazeboLidarProvider
  - gazebo_lunar_sim::GazeboCameraProvider
  - gazebo_lunar_sim::GazeboImuProvider
  - my_package::MyCustomProvider
```

### Estimator Configuration

```yaml
estimators:
  lio:
    implementation: string      # "fast_lio2", "dlio"
    enabled: bool             # Enable/disable
    max_cpu_percent: float      # CPU usage limit
    preprocessing: object      # Point cloud preprocessing
    
  vio:
    implementation: string      # "orb_slam3", "openvins"
    enabled: bool
    max_cpu_percent: float
    max_features: int          # Feature limit
    
  kinetic:
    enabled: bool
    max_cpu_percent: float
    ekf_config: object         # robot_localization config
```

### LIO Estimator Schema

```yaml
lio:
  implementation: string        # Required
    # Options: "fast_lio2", "dlio"
    
  enabled: bool               # Required
    # Default: true
    
  max_cpu_percent: float      # Optional
    # Default: 40.0
    # Range: 5.0 - 80.0
    
  preprocessing:              # Optional
    voxel_filter:             # Point cloud downsampling
      enabled: bool           # Default: false (dev), true (edge/low_power)
      leaf_size: float        # Default: 0.1 (meters)
        # Range: 0.05 - 0.5
        
    rate_limit_hz: float      # Optional
      # Default: 50.0
      # Range: 1.0 - 100.0
```

### VIO Estimator Schema

```yaml
vio:
  implementation: string        # Required
    # Options: "orb_slam3", "openvins"
    
  enabled: bool               # Required
    # Default: true
    
  max_cpu_percent: float      # Optional
    # Default: 30.0
    # Range: 5.0 - 60.0
    
  max_features: int           # Optional
    # Default: 300 (dev), 150 (edge), 100 (low_power)
    # Range: 50 - 500
    
  feature_quality_threshold: float  # Optional
    # Default: 0.7
    # Range: 0.0 - 1.0
```

### Kinetic Estimator Schema

```yaml
kinetic:
  enabled: bool               # Required
    # Default: true
    
  max_cpu_percent: float      # Optional
    # Default: 5.0
    # Range: 1.0 - 20.0
    
  ekf_config:                 # Required if enabled
    frequency: float          # EKF update rate
      # Default: 50.0
      # Range: 10.0 - 100.0
      
    odom0: string             # Wheel odometry topic
      # Default: "/wheel/odometry"
      
    odom0_config: list        # 15-element boolean list
      # Order: x, y, z, roll, pitch, yaw, vx, vy, vz, 
      #        v_roll, v_pitch, v_yaw, ax, ay, az
      # true = fuse this variable
      # Example: [true, true, false, ...] = fuse x, y but not z
      
    imu0: string              # IMU topic
      # Default: "/imu/data"
      
    imu0_config: list         # 15-element boolean list
      # Same order as odom0_config
```

### Health Thresholds Schema

```yaml
health_thresholds:
  nominal: float              # Minimum health for nominal mode
    # Default: 0.75
    # Range: 0.5 - 0.9
    
  degraded: float             # Minimum health for degraded mode
    # Default: 0.55
    # Range: 0.3 - 0.7
    # Must be < nominal threshold
```

### Complete Example (dev.yaml)

```yaml
compute_profile:
  name: "dev"
  hardware_target: "workstation"
  
  providers:
    - gazebo_lunar_sim::GazeboLidarProvider
    - gazebo_lunar_sim::GazeboCameraProvider
    - gazebo_lunar_sim::GazeboImuProvider
  
  estimators:
    lio:
      implementation: "fast_lio2"
      enabled: true
      max_cpu_percent: 40.0
      
    vio:
      implementation: "orb_slam3"
      enabled: true
      max_cpu_percent: 30.0
      max_features: 300
      
    kinetic:
      enabled: true
      max_cpu_percent: 5.0
      ekf_config:
        frequency: 50.0
        odom0: "/wheel/odometry"
        odom0_config: [true, true, false,
                       false, false, false,
                       false, false, false,
                       false, false, false,
                       false, false, false]
        imu0: "/imu/data"
        imu0_config: [false, false, false,
                      true, true, true,
                      false, false, false,
                      true, true, true,
                      true, true, true]
  
  health_thresholds:
    nominal: 0.75
    degraded: 0.55
    
  monitoring_rate_hz: 10.0
```

## Fusion Configuration Schema

### weights_nominal.yaml

```yaml
fusion_core:
  weights:                      # Required
    lio: float                  # LIO fusion weight
      # Default: 0.50
      # Range: 0.0 - 1.0
      # Note: Weights should sum to 1.0
      
    vio: float                  # VIO fusion weight
      # Default: 0.35
      # Range: 0.0 - 1.0
      
    kinetic: float              # Kinetic fusion weight
      # Default: 0.15
      # Range: 0.0 - 1.0
  
  min_weight: float             # Minimum allowed weight
    # Default: 0.05
    # Range: 0.0 - 0.3
    
  max_weight: float             # Maximum allowed weight
    # Default: 0.90
    # Range: 0.5 - 1.0
  
  health_weight_mapping:        # Dynamic adjustment
    health_threshold: float     # Below this, reduce weight
      # Default: 0.5
    min_scale: float            # Scale factor at health=0
      # Default: 0.2
      # Range: 0.0 - 0.5
  
  consistency:                  # Outlier rejection
    enabled: bool               # Default: true
    mahalanobis_threshold: float  # Default: 3.0
      # Standard deviations
      # Range: 1.0 - 10.0
    outlier_rejection: bool     # Default: true
  
  covariance_scale_factor: float  # Default: 2.0
    # Higher weight → lower covariance scaling
    # Range: 0.5 - 5.0
  
  process_noise:                # EKF process noise
    position: float             # Default: 0.01 (m²/s)
    orientation: float          # Default: 0.001 (rad²/s)
    velocity: float             # Default: 0.1 ((m/s)²/s)
    angular_velocity: float     # Default: 0.01 ((rad/s)²/s)
```

### health_thresholds.yaml

```yaml
health_monitoring:
  rate_hz: float                # Monitoring frequency
    # Default: 10.0
    # Range: 1.0 - 50.0
  
  thresholds:
    nominal_min: float          # Minimum for nominal mode
      # Default: 0.75
    degraded_min: float         # Minimum for degraded mode
      # Default: 0.55
  
  time_sync:                    # Time sync monitoring
    max_mean_skew_ms: float     # Mean offset threshold
      # Default: 5.0
      # Range: 1.0 - 20.0
      
    max_p95_skew_ms: float      # 95th percentile threshold
      # Default: 10.0
      # Range: 2.0 - 50.0
      
    max_sync_skew_ms: float     # Maximum single offset
      # Default: 20.0
      # Range: 5.0 - 100.0
  
  stale_data:                   # Stale data detection
    timeout_multiplier: float   # Multiplier on expected period
      # Default: 2.0
      # Range: 1.5 - 5.0
      
  rate_monitoring:              # Publication rate monitoring
    min_rate_percentage: float  # Minimum acceptable rate
      # Default: 0.8 (80% of expected)
      # Range: 0.5 - 0.95
  
  covariance:                   # Covariance bounds
    max_position_std_m: float     # Position std dev limit
      # Default: 1.0
    max_orientation_std_deg: float  # Orientation std dev limit
      # Default: 10.0
  
  sensor_weights:               # Individual sensor weights
    lidar: float                # Default: 0.35
    left_camera: float          # Default: 0.20
    right_camera: float         # Default: 0.20
    imu: float                  # Default: 0.25
```

## Sensor Configuration Schema

### simulated_livox.yaml

```yaml
livox_lidar:
  name: string                  # Sensor name
    # Default: "livox_mid360"
    
  frame_id: string              # TF frame ID
    # Default: "livox_frame"
  
  simulation:                   # Simulation parameters
    range_min: float            # Minimum range (m)
      # Default: 0.1
    range_max: float            # Maximum range (m)
      # Default: 100.0
    horizontal_fov: float       # Horizontal field of view (deg)
      # Default: 360.0
    vertical_fov: float         # Vertical field of view (deg)
      # Default: 77.2
    vertical_resolution: float  # Angular resolution (deg)
      # Default: 0.5
  
  publish_rate_hz: float        # Publication rate
    # Default: 10.0
  
  topic: string                 # ROS topic
    # Default: "/lidar/points"
  
  noise:                        # Simulation noise
    enabled: bool               # Default: true
    stddev_range: float         # Range noise std dev (m)
      # Default: 0.01
  
  time_sync:                    # Time synchronization
    reference_clock: string     # "ros" or "sensor"
      # Default: "ros"
    offset_s: float             # Time offset
      # Default: 0.0
```

### simulated_stereo.yaml

```yaml
stereo_camera:
  left:
    name: string
    frame_id: string
    topic: string
    info_topic: string
    
  right:
    name: string
    frame_id: string
    topic: string
    info_topic: string
  
  resolution:
    width: int                  # Default: 1280
    height: int                 # Default: 720
  
  publish_rate_hz: float        # Default: 20.0
  
  baseline_m: float             # Stereo baseline (m)
    # Default: 0.12
  
  calibration:                  # Camera calibration
    left:
      K: list                   # 9-element camera matrix [fx, 0, cx, 0, fy, cy, 0, 0, 1]
      D: list                   # 5-element distortion [k1, k2, p1, p2, k3]
    right:
      K: list
      D: list
  
  time_sync:
    reference_clock: string
    offset_s: float
```

### simulated_imu.yaml

```yaml
imu:
  name: string                  # Default: "imu_9axis"
  frame_id: string              # Default: "imu_frame"
  topic: string                 # Default: "/imu/data"
  publish_rate_hz: float        # Default: 200.0
  
  noise:                        # Noise characteristics
    accel:                      # Accelerometer
      noise_stddev: float       # Default: 0.01 (m/s²)
      bias_stddev: float        # Default: 0.005
    gyro:                       # Gyroscope
      noise_stddev: float       # Default: 0.001 (rad/s)
      bias_stddev: float        # Default: 0.0005
    mag:                        # Magnetometer
      noise_stddev: float       # Default: 0.001 (Gauss)
  
  time_sync:
    reference_clock: string     # Default: "imu"
    is_master: bool             # This sensor is sync master
      # Default: true
```

## Validation

### YAML Syntax Check

```bash
# Install yamllint
pip install yamllint

# Check all files
yamllint config/

# Check specific file
yamllint config/compute_profiles/dev.yaml
```

### Python Validation

```bash
# Validate in Python
python3 << 'EOF'
import yaml
with open('config/compute_profiles/dev.yaml') as f:
    config = yaml.safe_load(f)
    print("Valid YAML!")
    # Add custom validation here
EOF
```

## Environment Variables

Override configuration at runtime:

| Variable | Description | Example |
|----------|-------------|---------|
| `JUPPITER_CONFIG_PATH` | Custom config directory | `/custom/path/to/config` |
| `JUPPITER_LOG_LEVEL` | Logging verbosity | `debug`, `info`, `warn`, `error` |
| `JUPPITER_COMPUTE_PROFILE` | Default compute profile | `dev`, `edge`, `low_power` |

Example:
```bash
export JUPPITER_CONFIG_PATH=/workspace/custom_config
export JUPPITER_LOG_LEVEL=debug
ros2 launch gazebo_lunar_sim simulation.launch.py
```

## Configuration Inheritance

Profiles can inherit from base configurations:

```yaml
# base.yaml
compute_profile:
  estimators:
    kinetic:
      enabled: true

# dev.yaml
compute_profile:
  name: "dev"
  extends: "base"              # Inherit from base.yaml
  estimators:
    lio:
      enabled: true
    # kinetic inherited from base
```

## Best Practices

### Do:
- ✓ Comment non-obvious settings
- ✓ Use consistent indentation (2 spaces)
- ✓ Validate YAML syntax before committing
- ✓ Document custom profiles
- ✓ Keep sensitive data out of configs

### Don't:
- ✗ Use tabs for indentation
- ✗ Commit untested config changes
- ✗ Hardcode absolute paths
- ✗ Use floating point for integer values
- ✗ Skip validation

## Next Topics

- [Package Overview](package-overview.md) - Package structure
- [User Guides/Configuration](../user-guides/configuration.md) - Using configs
- [Architecture/Compute Profiles](../architecture/compute-profiles.md) - Profile concepts
