# Compute Profiles

juppiter supports three compute profiles optimized for different hardware tiers. Each profile configures the system to use appropriate estimators and parameters for the target hardware.

## Profile Overview

| Profile | Target Hardware | Estimators | Use Case |
|---------|-----------------|------------|----------|
| **dev** | Workstation/Server | FAST-LIO2 + ORB-SLAM3 | Algorithm development, maximum accuracy |
| **edge** | Raspberry Pi 5 / Jetson Orin Nano | DLIO + OpenVINS | Field deployment, balanced performance |
| **low_power** | Raspberry Pi 4B 4GB | DLIO only | Minimal power/resource usage |

## Profile Configuration Files

Located in `config/compute_profiles/`:

```
config/compute_profiles/
├── dev.yaml          # Full estimators
├── edge.yaml         # Optimized estimators
└── low_power.yaml    # Minimal estimators
```

## Dev Profile

**Hardware Requirements:**
- 8+ CPU cores
- 16GB+ RAM
- Dedicated GPU (optional but recommended for ORB-SLAM3)

**Configuration:** `config/compute_profiles/dev.yaml`

```yaml
compute_profile:
  name: "dev"
  hardware_target: "workstation"
  
  estimators:
    lio:
      implementation: "fast_lio2"
      enabled: true
      max_cpu_percent: 40.0
      
    vio:
      implementation: "orb_slam3"
      enabled: true
      max_cpu_percent: 30.0
      
    kinetic:
      enabled: true
      max_cpu_percent: 5.0
  
  fusion:
    weights:
      lio: 0.50
      vio: 0.35
      kinetic: 0.15
    
  monitoring:
    rate_hz: 10.0
    health_threshold_nominal: 0.75
    health_threshold_degraded: 0.55
```

**Characteristics:**
- **Accuracy:** Highest (FAST-LIO2 is more accurate than DLIO)
- **Resource Usage:** High (40% CPU for LIO, 30% for VIO)
- **Latency:** Lower (more processing power available)
- **Use Cases:**
  - Algorithm development and tuning
  - Ground truth generation for validation
  - High-performance robots with powerful computers

## Edge Profile

**Hardware Requirements:**
- Raspberry Pi 5 8GB or Jetson Orin Nano
- 4+ CPU cores
- 8GB RAM

**Configuration:** `config/compute_profiles/edge.yaml`

```yaml
compute_profile:
  name: "edge"
  hardware_target: "raspberry_pi_5"
  
  estimators:
    lio:
      implementation: "dlio"
      enabled: true
      max_cpu_percent: 35.0
      downsampling:
        voxel_size: 0.1  # m
        enabled: true
      
    vio:
      implementation: "openvins"
      enabled: true
      max_cpu_percent: 25.0
      features:
        max_features: 150  # Reduced from dev default
        
    kinetic:
      enabled: true
      max_cpu_percent: 5.0
  
  fusion:
    weights:
      lio: 0.50
      vio: 0.35
      kinetic: 0.15
    
  monitoring:
    rate_hz: 10.0
    health_threshold_nominal: 0.75
    health_threshold_degraded: 0.55
```

**Characteristics:**
- **Accuracy:** Good (DLIO is slightly less accurate than FAST-LIO2 but much faster)
- **Resource Usage:** Moderate (35% CPU for LIO, 25% for VIO)
- **Latency:** Acceptable for real-time operation
- **Use Cases:**
  - Field robots with moderate compute
  - Prototypes before final optimization
  - Balanced performance applications

**Optimizations:**
- LiDAR point cloud downsampled to 0.1m voxels
- VIO feature count limited to 150 (vs 300 in dev)
- Reduced iteration counts in optimization loops

## Low Power Profile

**Hardware Requirements:**
- Raspberry Pi 4B 4GB
- 4 CPU cores
- 4GB RAM (tight fit)

**Configuration:** `config/compute_profiles/low_power.yaml`

```yaml
compute_profile:
  name: "low_power"
  hardware_target: "raspberry_pi_4b"
  
  estimators:
    lio:
      implementation: "dlio"
      enabled: true
      max_cpu_percent: 45.0
      downsampling:
        voxel_size: 0.15  # More aggressive downsampling
        enabled: true
      rate_limit_hz: 10.0  # Limit to 10Hz max
      
    vio:
      enabled: false  # Disabled to save resources
      
    kinetic:
      enabled: true
      max_cpu_percent: 10.0
  
  fusion:
    weights:
      lio: 0.80  # Higher weight since no VIO
      vio: 0.0
      kinetic: 0.20
    
  monitoring:
    rate_hz: 5.0  # Reduced monitoring frequency
    health_threshold_nominal: 0.75
    health_threshold_degraded: 0.55
```

**Characteristics:**
- **Accuracy:** Acceptable (single estimator)
- **Resource Usage:** Low (45% CPU for LIO, VIO disabled)
- **Latency:** Higher but stable
- **Use Cases:**
  - Minimal power consumption scenarios
  - Long-duration missions
  - Cost-sensitive deployments

**Trade-offs:**
- VIO disabled entirely
- More aggressive LiDAR downsampling (0.15m vs 0.1m)
- Reduced monitoring frequency (5Hz vs 10Hz)
- Single point of failure (only LIO + Kinetic)

## Switching Profiles

### At Launch Time

```bash
# Dev profile (default if not specified)
ros2 launch gazebo_lunar_sim simulation.launch.py compute_profile:=dev

# Edge profile
ros2 launch gazebo_lunar_sim simulation.launch.py compute_profile:=edge

# Low power profile
ros2 launch gazebo_lunar_sim simulation.launch.py compute_profile:=low_power
```

### Runtime Profile Detection

The system detects which estimators are actually running and adjusts fusion weights accordingly. If you launch with `dev` profile but FAST-LIO2 crashes, the system automatically:

1. Detects missing `/lio/odom` topic
2. Reduces LIO weight to 0.0
3. Increases Kinematic weight to compensate
4. Switches to `degraded_lio` mode

## Creating Custom Profiles

You can create custom profiles by copying and modifying an existing one:

```bash
cp config/compute_profiles/edge.yaml config/compute_profiles/custom.yaml
```

Edit the file to adjust:
- CPU limits
- Feature counts
- Downsampling parameters
- Fusion weights

Then launch with your custom profile:
```bash
ros2 launch gazebo_lunar_sim simulation.launch.py compute_profile:=custom
```

## Performance Comparison

Based on simulation benchmarks (Gazebo lunar environment):

| Metric | Dev (FAST-LIO2) | Edge (DLIO) | Low Power (DLIO) |
|--------|-----------------|-------------|------------------|
| **Position RMSE** | 0.08m | 0.12m | 0.15m |
| **Orientation RMSE** | 1.2° | 1.8° | 2.1° |
| **CPU Usage** | ~70% | ~65% | ~55% |
| **RAM Usage** | ~4GB | ~3GB | ~2GB |
| **Latency (mean)** | 25ms | 35ms | 45ms |
| **Max update rate** | 50Hz | 40Hz | 10Hz |

*Note: Metrics vary based on hardware and environment dynamics*

## Profile Selection Guide

Choose based on your constraints:

**Use Dev Profile If:**
- Developing new algorithms
- Generating ground truth data
- Running on powerful hardware
- Accuracy is more important than power

**Use Edge Profile If:**
- Deploying to Raspberry Pi 5 or Jetson
- Need both LIO and VIO redundancy
- Balanced performance required
- Field testing prototypes

**Use Low Power Profile If:**
- Running on Raspberry Pi 4B
- Battery life is critical
- LiDAR-only operation acceptable
- Cost is a major constraint

## Hardware Selection

| Profile | Recommended Hardware | Price Range |
|---------|---------------------|-------------|
| Dev | Intel i7/i9, 32GB RAM, RTX GPU | $1500+ |
| Edge | Raspberry Pi 5 8GB or Jetson Orin Nano | $100-200 |
| Low Power | Raspberry Pi 4B 4GB | $55 |

## Next Steps

- Learn about [Sensor Core](sensor-core.md) health monitoring
- Read the [Configuration Guide](../user-guides/configuration.md)
- Explore [FDIIR Testing](../user-guides/fdiir-testing.md)
