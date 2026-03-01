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
- `colcon test` — ament_copyright, ament_cpplint, ament_uncrustify, cppcheck, xmllint

**Planned (Tier 3) — add when nodes can be launched together:**
- Integration tests — launch node graph, replay bag, verify outputs
- Coverage thresholds

## sensor_bridge

Three components separated by responsibility:

- **`sensor_bridge_node`** — ROS 2 node. Owns publishers, 200Hz wall timer, parameters. Orchestrates the replay loop with IMU interleaved before camera frames for VIO causal ordering.
- **`euroc_reader`** — Pure C++/OpenCV. Parses EuRoC CSV files, loads images on demand. No ROS dependency.
- **`stereo_depth`** — Pure C++/OpenCV. Rectifies stereo pair via `cv::stereoRectify`, runs StereoSGBM, converts disparity to depth. No ROS dependency.

Camera intrinsics and stereo extrinsics are loaded from `config/euroc_cam.yaml` as ROS parameters, not parsed from EuRoC sensor.yaml. This avoids a yaml-cpp dependency and keeps the node reusable for other datasets.

## Design Decisions

- **Incremental packages** — ROS 2 packages are created as needed, not scaffolded ahead of time.
- **Pure C++ components** — Computation (stereo depth, CSV parsing) is separated from ROS in pure C++/OpenCV classes for testability and reuse.
- **Merge commits** — `develop` → `main` uses merge commits to preserve full history and avoid divergence issues that squash merging causes.
