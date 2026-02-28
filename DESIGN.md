# Design — juppiter

Architecture and design decisions. More to come... 

## High-Level Architecture

Six ROS 2 nodes running in a Docker container, communicating via DDS topics, TF2 transforms, and services:

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

## Design Decisions

tbd
