# Architecture Overview

This section covers the high-level architecture and design principles of juppiter.

## Topics

- [System Overview](overview.md) - Complete system architecture
- [Compute Profiles](compute-profiles.md) - dev/edge/low_power tiers
- [Sensor Core](sensor-core.md) - FDIIR framework and health monitoring
- [Fusion Core](fusion-core.md) - Multi-estimator confidence fusion
- [Interfaces](interfaces.md) - C++ API and plugin architecture

## Design Philosophy

juppiter follows these core principles:

1. **Fault Tolerance First** - Every component has degradation paths
2. **Modularity** - Plugin-based architecture for easy extension
3. **Determinism** - Time synchronization and bounded latency
4. **Edge-Ready** - Runs on constrained hardware (Pi 4B -> Workstation)
5. **Simulation-First** - Validated in Gazebo before hardware deployment

## System Layers

```
┌─────────────────────────────────────────────────────────────────┐
│                    User Applications                            │
│         (Navigation, Planning, Control)                         │
└─────────────────────────────────────────────────────────────────┘
                                │
                    ┌───────────▼───────────┐
                    │  /perception/odom     │
                    │  /perception/mode     │
                    │  /perception/health   │
                    └───────────┬───────────┘
                                │
┌───────────────────────────────▼───────────────────────────────┐
│                     Perception Stack                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │ Fusion Core  │  │ Sensor Core  │  │ Estimators   │         │
│  │ (Confidence) │  │ (FDIIR)      │  │ (LIO/VIO)    │         │
│  └──────────────┘  └──────────────┘  └──────────────┘         │
└─────────────────────────────────────────────────────────────────┘
                                │
┌───────────────────────────────▼───────────────────────────────┐
│                     Sensor Layer                                │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌───────────┐ │
│  │   LiDAR     │ │   Camera    │ │     IMU     │ │  Wheels   │ │
│  │  Providers  │ │  Providers  │ │  Providers  │ │ Encoders  │ │
│  └─────────────┘ └─────────────┘ └─────────────┘ └───────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

## Data Flow

1. **Sensors** publish data via ROS topics
2. **Sensor Core** monitors health and synchronizes timestamps
3. **Estimators** (LIO/VIO/Kinematic) process sensor data independently
4. **Fusion Core** combines estimates with confidence weighting
5. **Output** is published as `/perception/odom` for downstream use

## Next Steps

- Read the [System Overview](overview.md) for detailed architecture
- Understand [Compute Profiles](compute-profiles.md) for hardware optimization
- Learn about the [FDIIR Framework](sensor-core.md)
