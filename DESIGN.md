# Design — juppiter

Architecture and design decisions for the Extreme-Conditions Lunar Perception Pipeline.

## High-Level Architecture

Multi-sensor perception stack with plugin-based provider architecture:

```
┌─────────────────────────────────────────────────────────────┐
│                    sensor_core (orchestrator)                │
│  ┌─────────────────┐  ┌─────────────────┐                  │
│  │ TimeSynchronizer│  │ HealthMonitor   │                  │
│  │ (skew detection)│  │ (mode manager)  │                  │
│  └─────────────────┘  └─────────────────┘                  │
│                                                              │
│  Topics: /perception/health, /perception/mode                │
└─────────────────────────────────────────────────────────────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
┌───────▼──────┐  ┌────────▼──────┐  ┌───────▼──────┐
│euroc_provider│  │lidar_provider │  │other_provider│
│ (EuRoC data) │  │ (real lidar) │  │  (future)    │
└──────────────┘  └──────────────┘  └──────────────┘
        │                  │
        ▼                  ▼
/stereo/left/       /lidar/points
/stereo/right/          │
/imu/data                │
    │                    │
    └────────┬───────────┘
             ▼
    ┌────────────────────┐
│  │  fusion_core       │
│  │  (LIO + VIO merge) │  [FUTURE]
│  └────────────────────┘
│           │
│           ▼
│  /perception/odometry
│
│  ┌────────────────────┐
│  │  mode_manager      │  [FUTURE]
│  │  (nominal/degraded/safe_stop)
│  └────────────────────┘
│
└─────────────────────────────────────────────────────────────
```

## Package Architecture

### sensor_interfaces (Interface Layer)

**Purpose:** Header-only package defining contracts for all sensor implementations.

**Key Components:**
- `SensorDriver` — Abstract base class for all sensor providers
- `SensorCapability` — Capability discovery flags (COLOR_IMAGE, LIDAR_POINTS, etc.)
- `HealthStatus` — Standardized health reporting structure
- `TimeSyncStatus` — Time synchronization monitoring data
- `SensorCalibration` — Calibration data structures with versioning

**Design Principles:**
- Pure interfaces, no implementation
- No ROS dependencies in base interface (only rclcpp::Node for initialization)
- Versioned calibration artifacts support

### sensor_core (Orchestration Layer)

**Purpose:** Central coordinator that loads providers, monitors health, and manages time sync.

**Key Components:**

**1. SensorBridgeNode**
- Main ROS 2 node
- Loads providers via `pluginlib::ClassLoader`
- Manages provider lifecycle (initialize → start → stop → shutdown)
- Runs monitoring loop at configurable rate (default: 10Hz)
- Publishes `/perception/health` and `/perception/mode`

**2. TimeSynchronizer**
- Registers all sensors for timestamp monitoring
- Computes inter-sensor skew (mean, p95, max)
- Reports sync status against PRD thresholds (5ms mean, 10ms p95)
- Thread-safe operations with mutex protection

**3. HealthMonitor**
- Aggregates health from all registered drivers
- Computes overall health score per PRD formula
- Determines system mode:
  - `nominal` (health ≥ 0.75)
  - `degraded_lidar` or `degraded_vision` (0.55 ≤ health < 0.75)
  - `safe_stop` (health < 0.55)

### euroc_provider (Provider Example)

**Purpose:** Concrete SensorDriver implementation for EuRoC dataset replay.

**Structure:**
- `EurocDriver` — Implements `SensorDriver` interface
- `EurocReader` — Pure C++ CSV parsing and image loading (no ROS)
- `StereoDepth` — Pure C++ stereo rectification and depth computation (no ROS)

**Topics Published:**
- `/stereo/left/image_raw` — Left camera (mono8)
- `/stereo/right/image_raw` — Right camera (mono8)
- `/camera/depth/image_rect_raw` — Stereo depth (32FC1)
- `/imu/data` — IMU measurements
- `/camera/color/camera_info` — Camera intrinsics

## Design Decisions

### 1. Plugin-Based Architecture

**Decision:** Use `pluginlib` for runtime provider loading.

**Rationale:**
- New sensors can be added without recompiling core
- Enables third-party provider packages
- Matches ROS 2 ecosystem conventions

**Trade-off:** Slightly more complex than static linking, but enables the modular vision.

### 2. Interface Separation

**Decision:** Separate `sensor_interfaces` (pure contracts) from `sensor_core` (orchestration).

**Rationale:**
- Prevents circular dependencies
- Providers only depend on interfaces, not implementation
- Enables testing with mock implementations

### 3. Time Sync Monitoring

**Decision:** Monitor timestamp skew in core, don't force hardware sync on providers.

**Rationale:**
- Providers can use software timestamps or hardware PPS
- Core reports violations but doesn't force sync method
- Allows gradual migration from software to hardware sync

**Thresholds (per PRD):**
- Target: ≤ 5ms mean skew
- Warning: ≤ 10ms p95 skew
- Alert: > 10ms p95 skew (triggers degraded mode)

### 4. Health Score Calculation

**Decision:** Start with simple weighted average, evolve to full PRD formula.

**Current Formula:**
```
raw = 0.45 * lidar_score + 0.35 * vision_score + 0.20 * imu_score
penalty = stale_penalty + sync_penalty + fault_penalty
overall = clamp(raw - penalty, 0.0, 1.0)
```

**Future additions:**
- Covariance penalty
- EMA smoothing (alpha=0.25)
- Confidence-weighted fusion

### 5. Provider Isolation

**Decision:** Each provider runs as separate ROS 2 node (via Node composition).

**Rationale:**
- Crash isolation — one provider failure doesn't kill others
- Independent lifecycle management
- Easier debugging and profiling

**Trade-off:** More resource overhead than single-process, but acceptable for safety-critical applications.

### 6. Pure C++ Components

**Decision:** Keep computation (stereo depth, parsing) in pure C++/OpenCV, separate from ROS.

**Rationale:**
- Testable without ROS dependencies
- Reusable in non-ROS contexts
- Easier unit testing with standard test frameworks

## CI/CD

**Active (Tier 1):**
- `colcon build` — compile all packages
- `colcon test` — ament_lint (copyright, cpplint, uncrustify, cppcheck, xmllint)

**Planned (Tier 2):**
- Unit tests for pure C++ components (EurocReader, StereoDepth)
- Integration tests with mock providers
- Plugin loading validation

**Planned (Tier 3):**
- Full pipeline integration tests with EuRoC dataset
- Performance benchmarks (latency, CPU usage)
- Health monitoring validation with injected faults

## Future Extensions

### Multi-Estimator Fusion (Milestone 4)

Add `fusion_core` package:
- EKF-based LIO + VIO fusion
- Covariance-weighted sensor arbitration
- Handoff discontinuity monitoring (≤ 0.30m, ≤ 5° yaw)

### Hazard Detection (Milestone 5)

Add `terrain_perception` package:
- Lidar-based geometry analysis
- Traversability scoring (slope, roughness, clearance)
- Hazard detection (rocks, crater edges, steep slopes)

### Calibration Management

Extend with calibration versioning:
```yaml
calibration:
  version: "1.0.0"
  sensors:
    - name: "camera_left"
      extrinsics: { parent: "base_link", ... }
    - name: "lidar"
      extrinsics: { parent: "base_link", ... }
```

On-boot validation and runtime drift monitoring.

## Branch Strategy

- **`main`** — Stable releases, merge commits from develop
- **`develop`** — Integration branch, merge commits from feature branches
- **feature branches** — Branch off develop, PR back to develop

Rationale: Preserve full history, avoid divergence issues from squash merging.
