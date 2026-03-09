# Sensor Core (FDIIR Framework)

The Sensor Core implements the FDIIR (Fault Detection, Isolation, Identification, Recovery) framework, providing robust fault management for the perception pipeline.

## What is FDIIR?

**FDIIR** stands for:
- **Fault Detection** - Identifying when something is wrong
- **Fault Isolation** - Determining which component failed
- **Fault Identification** - Classifying the type of fault
- **Recovery Management** - Taking corrective action

```
┌─────────────────────────────────────────────────────────────┐
│                    FDIIR Framework                           │
│                                                                │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐    │
│  │   Fault      │───▶│   Fault      │───▶│   Recovery   │    │
│  │  Detection   │    │  Isolation   │    │   Manager    │    │
│  └──────────────┘    └──────────────┘    └──────────────┘    │
│         │                                            │       │
│         ▼                                            ▼       │
│  /perception/health                          Mode transition │
│  /perception/faults                                          │
└─────────────────────────────────────────────────────────────┘
```

## Components

### 1. Fault Detector

Monitors all sensors and estimators for anomalies:

**Detection Methods:**

| Check | Description | Threshold |
|-------|-------------|-----------|
| Stale Data | No message received in expected window | 2x expected period |
| Rate Drop | Publication rate below minimum | Configurable per sensor |
| Sync Violation | Inter-sensor timestamp skew too high | > 10ms p95 |
| Covariance Exceeded | Estimated uncertainty too high | > 1.0 m position |
| Correlation Loss | Expected correlation between sensors broken | Statistical test |

**Implementation:**

The `FaultDetector` class runs at a configurable rate (default 10Hz) and checks:

```cpp
class FaultDetector {
    // Check for stale data
    bool checkTimestampFreshness(const rclcpp::Time& last_msg_time);
    
    // Check publication rate
    bool checkPublicationRate(const std::string& topic, double expected_hz);
    
    // Check timestamp synchronization
    bool checkSyncSkew(const std::vector<TimeSyncStatus>& statuses);
    
    // Check covariance bounds
    bool checkCovarianceBounds(const nav_msgs::msg::Odometry& odom);
};
```

### 2. Fault Isolator

Identifies which sensor or estimator is faulty:

**Fault Mapping:**

```
Fault Signature → Isolated Component

Stale LiDAR data + Good cameras → LiDAR provider fault
Stale VIO + Good LIO → VIO estimator fault  
High sync skew across all → Time synchronizer fault
High covariance on one only → That estimator fault
```

**Mode Determination:**

Based on isolated faults, the system enters one of four modes:

```cpp
enum class SystemMode {
    NOMINAL,           // Health ≥ 0.75, all good
    DEGRADED_LIO,      // LiDAR issues, VIO primary
    DEGRADED_VIO,      // Camera issues, LIO primary  
    SAFE_STOP          // Health < 0.55, stop required
};
```

### 3. Recovery Manager

Takes action based on fault classification:

**Recovery Actions:**

| Fault Type | Action | Priority |
|------------|--------|----------|
| Sensor timeout | Switch to redundant sensor | Immediate |
| Sync drift | Trigger resynchronization | Immediate |
| Rate degradation | Reduce fusion weight | Gradual |
| Estimator crash | Zero weight, alert operator | Immediate |
| Cascading failures | Enter safe_stop mode | Emergency |

**Graceful Degradation:**

```
NOMINAL → DEGRADED_LIO → SAFE_STOP
   │            │            │
   │            │            └── All navigation stops
   │            └── LIO offline, VIO primary
   └── Full redundancy active
```

## Health Monitoring

### Health Score Calculation

The overall health score is computed as:

```
H_total = Σ(w_i × H_i) / Σ(w_i)

where:
  H_i = individual sensor/estimator health (0.0 - 1.0)
  w_i = weight based on importance in current mode
```

### Health Components

Each sensor provides a `HealthStatus`:

```cpp
struct HealthStatus {
    double score;           // 0.0 - 1.0
    FaultFlags flags;     // Bitmask of active faults
    rclcpp::Time timestamp; // When measured
    std::string details;    // Human-readable info
};

enum class FaultFlags : uint32_t {
    NONE = 0,
    STALE = 1 << 0,              // No data received
    SYNC_VIOLATION = 1 << 1,      // Time sync issue
    RATE_DROP = 1 << 2,           // Low publication rate
    COVARIANCE_EXCEEDED = 1 << 3, // High uncertainty
    CORRELATION_LOSS = 1 << 4,     // Unexpected data
    CALIBRATION_INVALID = 1 << 5   // Bad calibration data
};
```

### Health Topics

The Sensor Core publishes:

```bash
# Overall health status (JSON)
ros2 topic echo /perception/health

# Example output:
{
  "overall_health": 0.92,
  "mode": "nominal",
  "active_sensors": 4,
  "faults": [],
  "timestamp": "2024-01-15T10:30:45.123Z"
}

# Current mode
ros2 topic echo /perception/mode
# Output: "nominal"

# Detailed fault log
ros2 topic echo /perception/faults
```

## Time Synchronization

### Importance

Multi-sensor fusion requires temporally aligned data. A 10ms timing error can cause significant position drift when moving at 1 m/s.

### Implementation

The `TimeSynchronizer` maintains:

```cpp
struct TimeSyncStatus {
    std::string sensor_name;
    rclcpp::Time last_timestamp;
    double skew_mean_ms;    // Average offset from reference
    double skew_p95_ms;     // 95th percentile offset
    double skew_max_ms;     // Maximum observed offset
    bool is_synced;         // Within acceptable bounds?
};
```

### Targets (from PRD)

| Metric | Target | Measured |
|--------|--------|----------|
| Mean skew | ≤ 5ms | Monitor with `ros2 topic echo /perception/sync_status` |
| p95 skew | ≤ 10ms | Checked every monitoring cycle |
| Max skew | < 20ms | Triggers warning |

### Synchronization Strategy

1. **Reference Clock** - Use the most stable sensor (typically IMU at 200Hz)
2. **Timestamp Recording** - Log arrival time vs message timestamp
3. **Skew Calculation** - Moving window average over 100 samples
4. **Bound Enforcement** - Alert if thresholds exceeded

```
Sensor A (LiDAR):  timestamp=1000.0, received_at=1000.005 → skew=5ms
Sensor B (Camera): timestamp=1000.0, received_at=1000.012 → skew=12ms ⚠️
Sensor C (IMU):   timestamp=1000.0, received_at=1000.001 → skew=1ms ✓ (reference)
```

## Configuration

Fault detection thresholds are configurable in `config/fusion/health_thresholds.yaml`:

```yaml
health_monitoring:
  rate_hz: 10.0
  
  thresholds:
    nominal_min: 0.75
    degraded_min: 0.55
    
  time_sync:
    max_mean_skew_ms: 5.0
    max_p95_skew_ms: 10.0
    max_sync_skew_ms: 20.0
    
  stale_data:
    timeout_multiplier: 2.0  # 2x expected period
    
  rate_monitoring:
    min_rate_tolerance: 0.8  # 80% of expected rate
    
  covariance:
    max_position_std_m: 1.0
    max_orientation_std_deg: 10.0
```

## Example Scenarios

### Scenario 1: LiDAR Dropout

```
T+0s:  LiDAR publishing normally at 10Hz, Health=1.0
T+0.5s: LiDAR message delayed
T+0.6s: FaultDetector detects stale data (no msg in 200ms)
T+0.6s: FaultIsolator identifies LiDAR provider
T+0.6s: RecoveryManager:
        - Sets LIO weight to 0.0
        - Increases VIO weight to 0.85
        - Increases Kinematic to 0.15
        - Publishes mode=DEGRADED_LIO
T+0.6s: Health drops to 0.72 (degraded but acceptable)
```

### Scenario 2: Camera Degradation

```
T+0s:  Cameras running at 20Hz, good lighting
T+10s: Rover enters shadow, image quality degrades
T+12s: VIO covariance increases to 2.0m
T+12.1s: FaultDetector flags COVARIANCE_EXCEEDED
T+12.1s: Fusion Core reduces VIO weight to 0.15
T+12.1s: Mode remains NOMINAL (LIO still good)
```

### Scenario 3: Cascading Failure

```
T+0s:  All sensors healthy, Health=0.95
T+5s:  LiDAR fails (hardware disconnect)
T+5.1s: Mode=DEGRADED_LIO, Health=0.72
T+7s:  Camera lens fogs up
T+7.2s: VIO covariance spikes
T+7.3s: Health=0.58, still DEGRADED_LIO
T+10s: IMU bias drift detected
T+10.1s: Health=0.51
T+10.1s: Mode transitions to SAFE_STOP
T+10.1s: RecoveryManager sends stop command
```

## Monitoring and Debugging

### Health Monitoring Tools

```bash
# Watch health in real-time
ros2 topic echo /perception/health

# Monitor mode transitions
ros2 topic echo /perception/mode

# See detailed fault log
ros2 topic echo /perception/faults

# Check sync status
ros2 topic hz /perception/health
```

### Diagnostics

The Sensor Core provides diagnostic information:

```bash
# View node info
ros2 node info /sensor_bridge_node

# List all topics
ros2 topic list | grep perception

# Echo with rate stats
ros2 topic hz /perception/odom
```

### FDIIR Test Scenarios

Run automated tests to validate FDIIR behavior:

```bash
# Test LiDAR failure handling
ros2 launch gazebo_lunar_sim simulation.launch.py scenario:=lidar_dropout

# Test camera degradation
ros2 launch gazebo_lunar_sim simulation.launch.py scenario:=camera_degradation

# Test cascading failures
ros2 launch gazebo_lunar_sim simulation.launch.py scenario:=cascading_fault
```

## Next Topics

- [Fusion Core](fusion-core.md) - How health affects fusion weights
- [FDIIR Testing](../user-guides/fdiir-testing.md) - Running validation scenarios
- [Interfaces](interfaces.md) - Plugin architecture
