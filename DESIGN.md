# Design — juppiter

Architecture and design decisions. More to come... 

## High-Level Architecture

Six ROS 2 (Kilted Kaiju) nodes running in a Docker container, communicating via DDS topics, TF2 transforms, and services:

```
RealSense D435i (RGB + Depth + IMU)
        │
  sensor_bridge ── remapping + time sync
        │
        ├──► slam_core ──────── ORB-SLAM3 → odometry + map + TF
        ├──► dynamic_filter ─── YOLOv8n → detections + masks
        ├──► pose_tracker ───── PnP + ICP + EKF → 6D object poses
        ├──► semantic_map ───── 3D labeled point cloud
        └──► query_node ─────── spatial queries over the map
```

## CI/CD

PRs to `develop` run automated checks and auto-merge on pass. PRs to `main` require manual sign-off.

**Active (Tier 1):**
- `colcon build` — compile all packages, validate message definitions
- `colcon test` — run unit tests declared in packages

**Planned (Tier 2) — add when C++ nodes exist:**
- `ament_lint` — ROS 2 style and convention checks
- `cppcheck` — static analysis for C++ (null derefs, memory leaks, undefined behavior)

**Planned (Tier 3) — add when nodes can be launched together:**
- Integration tests — launch node graph, replay bag, verify outputs
- Coverage thresholds

## Design Decisions

tbd
