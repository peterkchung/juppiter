# Installation

This guide walks you through setting up the juppiter development environment using Docker.

## Prerequisites

### Linux/macOS

- Docker Engine 20.10+ and Docker Compose 2.0+
- Git

### Windows

- Windows 10/11 with WSL2 enabled
- Docker Desktop with WSL2 backend
- WSLg (for GUI applications like Gazebo and RViz)

## Step-by-Step Installation

### 1. Clone the Repository

```bash
git clone https://github.com/opencode/juppiter.git
cd juppiter
```

### 2. Build the Development Container

```bash
docker compose -f docker/docker-compose.yml build
```

This builds a container based on `osrf/ros:kilted-desktop` with:
- ROS 2 Kilted Kaiju
- All juppiter dependencies pre-installed
- Gazebo Harmonic for simulation
- RViz2 for visualization

### 3. Start the Container

```bash
docker compose -f docker/docker-compose.yml up -d
```

The container runs in detached mode (`-d`), keeping it alive in the background.

### 4. Enter the Container

```bash
docker compose -f docker/docker-compose.yml exec dev bash
```

Your prompt should change to:
```
root@juppiter-dev:/ws#
```

This indicates you're inside the container at `/ws` (the workspace directory).

### 5. Build juppiter

Inside the container:

```bash
colcon build --symlink-install
```

The `--symlink-install` flag creates symlinks instead of copying files, allowing you to edit code outside the container and see changes immediately inside.

## Verify Installation

Check that all packages built successfully:

```bash
ros2 pkg list | grep juppiter
```

You should see packages like:
- `gazebo_lunar_sim`
- `sensor_core`
- `fusion_core`
- `sensor_interfaces`
- `kinetic_estimator`

## Container Management

### Stop the Container

```bash
docker compose -f docker/docker-compose.yml down
```

### Restart the Container

```bash
docker compose -f docker/docker-compose.yml up -d
```

### Rebuild After Dependency Changes

If you update dependencies (e.g., add packages to `package.xml`):

```bash
docker compose -f docker/docker-compose.yml down
docker compose -f docker/docker-compose.yml build --no-cache
docker compose -f docker/docker-compose.yml up -d
```

## Troubleshooting

### Container Won't Start

**Issue:** Port conflicts or volume issues

**Solution:**
```bash
# Check for running containers
docker ps

# Remove all juppiter containers
docker compose -f docker/docker-compose.yml down -v

# Rebuild and start fresh
docker compose -f docker/docker-compose.yml build --no-cache
docker compose -f docker/docker-compose.yml up -d
```

### Build Fails

**Issue:** Missing dependencies or stale build artifacts

**Solution:**
```bash
# Clean build artifacts
rm -rf build/ install/ log/

# Rebuild
colcon build --symlink-install
```

### GUI Applications Don't Work (Gazebo/RViz)

**Issue:** Display or graphics driver issues

**Solution:**
- **Linux:** Ensure X11 forwarding is enabled: `xhost +local:docker`
- **Windows:** Verify WSLg is installed and working
- **macOS:** Install XQuartz and enable "Allow connections from network clients"

## Next Steps

Your environment is now ready! Proceed to [Running Simulation](running-simulation.md) to launch Gazebo and see juppiter in action.
