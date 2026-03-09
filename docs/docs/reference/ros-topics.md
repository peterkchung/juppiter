# ROS Topics Reference

Complete catalog of all ROS topics used in juppiter.

## Perception Pipeline Topics

### Output Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/perception/odom` | `nav_msgs/Odometry` | 50 Hz | Fused pose estimate from all estimators |
| `/perception/mode` | `std_msgs/String` | 10 Hz | Current operational mode: `nominal`, `degraded_lio`, `degraded_vio`, `safe_stop` |
| `/perception/health` | `std_msgs/String` | 10 Hz | System health metrics (JSON format) |
| `/perception/faults` | `std_msgs/String` | Event | Active faults and their classification |
| `/perception/estimator_weights` | `std_msgs/String` | 10 Hz | Current fusion weights by source (JSON) |
| `/perception/sync_status` | `std_msgs/String` | 10 Hz | Time synchronization status |

### Estimator Output Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/lio/odom` | `nav_msgs/Odometry` | 10-50 Hz | LIO pose estimate (FAST-LIO2 or DLIO) |
| `/vio/odom` | `nav_msgs/Odometry` | 10-30 Hz | VIO pose estimate (ORB-SLAM3 or OpenVINS) |
| `/kinematic/odom` | `nav_msgs/Odometry` | 50 Hz | Wheel odometry + IMU EKF output |
| `/lio/health` | `std_msgs/Float32` | 10 Hz | LIO estimator health score |
| `/vio/health` | `std_msgs/Float32` | 10 Hz | VIO estimator health score |
| `/kinematic/health` | `std_msgs/Float32` | 10 Hz | Kinetic estimator health score |

## Simulation Sensor Topics (Gazebo)

### LiDAR

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/lidar/points` | `sensor_msgs/PointCloud2` | 10 Hz | Livox Mid-360 point cloud |
| `/lidar/scan` | `sensor_msgs/LaserScan` | 10 Hz | 2D laser scan (if enabled) |

### Cameras

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/stereo/left/image_raw` | `sensor_msgs/Image` | 20 Hz | Left camera (1280x720) |
| `/stereo/right/image_raw` | `sensor_msgs/Image` | 20 Hz | Right camera (1280x720) |
| `/stereo/left/camera_info` | `sensor_msgs/CameraInfo` | 20 Hz | Left camera intrinsics |
| `/stereo/right/camera_info` | `sensor_msgs/CameraInfo` | 20 Hz | Right camera intrinsics |
| `/stereo/depth` | `sensor_msgs/Image` | 20 Hz | Computed depth from stereo |

### IMU

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | 200 Hz | 9-axis IMU data |
| `/imu/data_raw` | `sensor_msgs/Imu` | 200 Hz | Raw IMU (before bias correction) |
| `/imu/mag` | `sensor_msgs/MagneticField` | 50 Hz | Magnetometer data |
| `/imu/temperature` | `sensor_msgs/Temperature` | 10 Hz | IMU temperature |

### Wheel Odometry

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/wheel/odometry` | `nav_msgs/Odometry` | 50 Hz | Differential drive odometry |
| `/wheel/joint_states` | `sensor_msgs/JointState` | 50 Hz | Individual wheel encoders |
| `/cmd_vel` | `geometry_msgs/Twist` | Sub | Velocity commands to rover |

## Legacy Topics (EuRoC Dataset)

Used for testing with EuRoC dataset replay:

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/stereo/left/image_raw` | `sensor_msgs/Image` | 20 Hz | Left grayscale (EuRoC) |
| `/stereo/right/image_raw` | `sensor_msgs/Image` | 20 Hz | Right grayscale (EuRoC) |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 20 Hz | Camera intrinsics |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 20 Hz | Stereo depth (computed) |
| `/imu/data` | `sensor_msgs/Imu` | 200 Hz | IMU measurements |

## Diagnostic Topics

### TF (Transform Tree)

Published by robot_state_publisher:

| Frame | Parent | Description |
|-------|--------|-------------|
| `base_link` | `odom` | Robot base frame |
| `livox_frame` | `base_link` | LiDAR mounting frame |
| `left_camera_optical_frame` | `base_link` | Left camera frame |
| `right_camera_optical_frame` | `base_link` | Right camera frame |
| `imu_frame` | `base_link` | IMU mounting frame |
| `wheel_left` | `base_link` | Left wheel frame |
| `wheel_right` | `base_link` | Right wheel frame |

### Diagnostic Messages

| Topic | Type | Description |
|-------|------|-------------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Aggregated diagnostics |
| `/diagnostics_agg` | `diagnostic_msgs/DiagnosticArray` | Aggregated by node |

### ROS 2 System Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/parameter_events` | `rcl_interfaces/ParameterEvent` | Parameter changes |
| `/rosout` | `rcl_interfaces/Log` | System logs |
| `/clock` | `rosgraph_msgs/Clock` | Simulation time (Gazebo) |

## Topic Message Examples

### /perception/odom

```yaml
header:
  stamp: sec: 1234567890, nanosec: 0
  frame_id: "odom"
child_frame_id: "base_link"
pose:
  pose:
    position: {x: 1.5, y: 2.3, z: 0.1}
    orientation: {x: 0.0, y: 0.0, z: 0.1, w: 0.995}
  covariance: [0.01, 0.0, ..., 0.001]  # 6x6 matrix
 twist:
  twist:
    linear: {x: 0.5, y: 0.0, z: 0.0}
    angular: {x: 0.0, y: 0.0, z: 0.1}
  covariance: [0.01, 0.0, ..., 0.001]
```

### /perception/health (JSON)

```json
{
  "timestamp": "2024-01-15T10:30:45.123Z",
  "overall_health": 0.92,
  "mode": "nominal",
  "active_sensors": 4,
  "faults": [],
  "sensor_health": {
    "lidar": 0.98,
    "left_camera": 0.95,
    "right_camera": 0.95,
    "imu": 0.99,
    "wheels": 1.0
  },
  "estimator_health": {
    "lio": 0.96,
    "vio": 0.88,
    "kinetic": 1.0
  }
}
```

### /perception/estimator_weights (JSON)

```json
{
  "timestamp": "2024-01-15T10:30:45.123Z",
  "weights": {
    "lio": 0.50,
    "vio": 0.35,
    "kinetic": 0.15
  },
  "mode": "nominal",
  "adjusted_by_health": false
}
```

## Working with Topics

### Echo with Filtering

```bash
# Echo specific field
ros2 topic echo /perception/health --field data

# Echo with JSON parsing
ros2 topic echo /perception/health | jq '.data | fromjson'

# Echo once
ros2 topic echo /perception/mode --once
```

### Topic Statistics

```bash
# Measure rate
ros2 topic hz /perception/odom

# Show bandwidth
ros2 topic bw /lidar/points

# Show type info
ros2 topic info /perception/odom
```

### Recording and Playback

```bash
# Record all perception topics
ros2 bag record -o perception_run \
  /perception/odom \
  /perception/health \
  /perception/mode \
  /lidar/points \
  /imu/data

# Playback
ros2 bag play perception_run/

# Playback with rate limit
ros2 bag play perception_run/ --rate 0.5
```

### Remapping

```bash
# Remap topics at runtime
ros2 run sensor_core sensor_bridge_node \
  --ros-args --remap /perception/odom:=/my_odom

# Or in launch file
Node(
    package='sensor_core',
    executable='sensor_bridge_node',
    remappings=[('/perception/odom', '/my_odom')]
)
```

## Topic QoS (Quality of Service)

Default QoS profiles:

| Topic Category | Reliability | Durability | History | Depth |
|----------------|-------------|------------|---------|-------|
| Sensor data | Best effort | Volatile | Keep last | 5 |
| Odometry | Reliable | Volatile | Keep last | 10 |
| Health/Mode | Reliable | Transient local | Keep last | 1 |
| Commands | Reliable | Volatile | Keep last | 1 |

Override in code:

```cpp
// Subscribe with custom QoS
rclcpp::QoS qos(10);
qos.reliable().transient_local();

auto sub = node->create_subscription<...>(
    "topic", qos, callback);
```

## Next Topics

- [Configuration Reference](configuration-reference.md) - YAML options
- [Package Overview](package-overview.md) - Package details
- [User Guides/Configuration](../user-guides/configuration.md) - Using configs
