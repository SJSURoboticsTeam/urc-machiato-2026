# Isaac ROS Integration & Docker Support

Complete integration guide for NVIDIA Isaac ROS (Visual SLAM, NvBlox, AprilTag) and Docker containerization for Isaac Sim.

## Overview

This implementation provides:

1. **Isaac ROS Integration** - GPU-accelerated perception on Jetson and Desktop
2. **Docker Support** - Containerized Isaac Sim with GUI and headless modes
3. **SLAM Backend Switching** - Seamlessly switch between Isaac SLAM, RTAB-Map, and SLAM Toolbox
4. **USD Scene Automation** - Automated Mars environment setup

## Quick Start

### 0. Test without Isaac ROS (optional)

To validate the package build and launch file without installing Isaac ROS:

```bash
# Build (workspace uses COLCON_IGNORE; use explicit base path)
source /opt/ros/jazzy/setup.bash   # or humble
colcon build --base-paths src/autonomy/isaac_ros_integration --symlink-install
source install/setup.bash

# Show launch arguments
ros2 launch isaac_ros_integration isaac_perception.launch.py --show-args

# Run with Isaac components disabled (status monitor only)
ros2 launch isaac_ros_integration isaac_perception.launch.py \
  slam_backend:=rtabmap enable_3d_reconstruction:=false enable_apriltag:=false
```

### 1. Isaac ROS Setup

```bash
# Automatic setup for your platform (detects Jetson/Desktop)
./src/autonomy/isaac_ros_integration/scripts/setup_isaac_ros.sh

# Or specify platform explicitly
./src/autonomy/isaac_ros_integration/scripts/setup_isaac_ros.sh jetson
./src/autonomy/isaac_ros_integration/scripts/setup_isaac_ros.sh desktop
```

### 2. Launch Isaac ROS Perception

```bash
# Source ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch with Isaac SLAM (default)
ros2 launch isaac_ros_integration isaac_perception.launch.py

# Launch with RTAB-Map instead
ros2 launch isaac_ros_integration isaac_perception.launch.py slam_backend:=rtabmap

# Launch with all features
ros2 launch isaac_ros_integration isaac_perception.launch.py \
    slam_backend:=isaac \
    enable_3d_reconstruction:=true \
    enable_apriltag:=true
```

### 3. Docker Isaac Sim

```bash
# GUI mode (requires X11)
./scripts/run_isaac_sim_docker.sh gui

# Headless mode with VNC access
./scripts/run_isaac_sim_docker.sh headless -d
# Access at: http://localhost:6080/vnc.html

# Development mode with full workspace
./scripts/run_isaac_sim_docker.sh dev

# Full stack (Isaac Sim + ROS2 + Isaac ROS + Dashboard)
./scripts/run_isaac_sim_docker.sh fullstack

# Test mode for CI/CD
./scripts/run_isaac_sim_docker.sh test
```

## Architecture

### Isaac ROS Components

```
Isaac ROS Integration
├── Visual SLAM (isaac_ros_visual_slam)
│   ├── GPU-accelerated feature detection
│   ├── Real-time pose estimation
│   └── Loop closure detection
│
├── NvBlox (isaac_ros_nvblox)
│   ├── 3D reconstruction
│   ├── 2D costmap generation
│   └── Mesh publishing
│
└── AprilTag (isaac_ros_apriltag)
    ├── GPU-accelerated detection
    ├── Pose estimation
    └── Tag database management
```

### Docker Services

```
Docker Compose Profiles:
├── gui: Full GUI with X11
├── headless: VNC + noVNC web interface
├── dev: Development environment
├── test: Automated testing
├── fullstack: Complete system
└── perception: Isaac ROS only
```

## Configuration

### SLAM Backend Switching

Switch between SLAM implementations dynamically:

```bash
# Switch to Isaac Visual SLAM
ros2 run isaac_ros_integration switch_slam_backend.py isaac

# Switch to RTAB-Map
ros2 run isaac_ros_integration switch_slam_backend.py rtabmap

# Switch to SLAM Toolbox (LiDAR-only)
ros2 run isaac_ros_integration switch_slam_backend.py slam_toolbox

# Show current backend
ros2 run isaac_ros_integration switch_slam_backend.py --current

# Show detailed status
ros2 run isaac_ros_integration switch_slam_backend.py --status
```

Configuration is stored in:
- `~/.bashrc` (environment variable)
- `config/isaac_ros.yaml` (YAML config)

### Isaac ROS Configuration File

Edit `src/autonomy/isaac_ros_integration/config/isaac_ros.yaml`:

```yaml
isaac_ros:
  slam:
    backend: "isaac"  # isaac, rtabmap, slam_toolbox
    isaac_visual_slam:
      enabled: true
      voxel_size: 0.05
      
  nvblox:
    enabled: true
    voxel_size: 0.05
    min_depth: 0.5
    max_depth: 10.0
    
  apriltag:
    enabled: true
    tag_family: "tag36h11"
    default_tag_size: 0.165
```

### AprilTag Configuration

Known tags are defined in:
`src/autonomy/isaac_ros_integration/config/apriltag_tags.yaml`

```yaml
apriltag_tags:
  competition:
    equipment_delivery:
      - id: 1
        size: 0.165
        name: "equipment_station_1"
        position: [10.0, 5.0, 0.0]
```

## Platform-Specific Notes

### Jetson Orin NX

**Prerequisites:**
- JetPack 6.0+
- CUDA 12.2+
- ROS2 Humble/Jazzy

**Optimizations:**
```bash
# Set MAXN power mode
sudo nvpmodel -m 0

# Enable jetson_clocks
sudo jetson_clocks --fan

# Monitor performance
sudo tegrastats
```

**Isaac ROS Installation:**
```bash
# Add NVIDIA repositories
curl -sSL https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/isaac-ros.list

# Install packages
sudo apt update
sudo apt install ros-humble-isaac-ros-visual-slam \
    ros-humble-isaac-ros-nvblox \
    ros-humble-isaac-ros-apriltag
```

### Desktop/Workstation

**Prerequisites:**
- NVIDIA GPU (RTX 4060 or better recommended)
- CUDA 11.8+ or 12.x
- Docker with nvidia-docker2
- ROS2 Humble/Jazzy

**GPU Setup:**
```bash
# Install nvidia-docker2
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
    sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt update
sudo apt install -y nvidia-docker2
sudo systemctl restart docker
```

## Docker Usage

### Build Images

```bash
# Build all images
docker-compose -f docker/docker-compose.isaac-sim.yml build

# Build specific target
docker build --target isaac-sim-gui -t urc2026/isaac-sim:gui .
docker build --target isaac-sim-headless -t urc2026/isaac-sim:headless .
```

### Run Containers

```bash
# GUI mode (interactive)
docker-compose -f docker/docker-compose.isaac-sim.yml --profile gui up

# Headless mode (background)
docker-compose -f docker/docker-compose.isaac-sim.yml --profile headless up -d

# Full stack
docker-compose -f docker/docker-compose.isaac-sim.yml --profile fullstack up

# Development (interactive shell)
docker-compose -f docker/docker-compose.isaac-sim.yml --profile dev up
```

### Environment Variables

```bash
# Required
export DISPLAY=:0  # For GUI mode

# Optional
export ISAAC_SIM_PATH=/path/to/isaac-sim-5.0.0
export ISAAC_SIM_VERSION=5.0.0
export ISAAC_ROS_SLAM_BACKEND=isaac
export ROS_DOMAIN_ID=42
export URC_ENV=simulation
```

## USD Scene Automation

Automated Mars environment setup script:

```bash
# Run from within Isaac Sim (Script Editor or Terminal)
python3 simulation/isaac/setup_mars_scene.py --output mars_scene.usd

# With rover
python3 simulation/isaac/setup_mars_scene.py \
    --with-rover \
    --rover-urdf robot_definition/rover2025/urdf/rover2025.urdf \
    --ros-bridge

# Headless (batch processing)
python3 simulation/isaac/setup_mars_scene.py --headless --output mars_scene.usd
```

**Features:**
- Mars gravity (3.71 m/s²)
- Procedural terrain with craters and rocks
- Sun lighting with reddish ambient
- Automatic rover import
- Camera and LiDAR sensors
- ROS2 bridge setup

## Launch Files Reference

### isaac_perception.launch.py
Main launch file with all components:
```bash
ros2 launch isaac_ros_integration isaac_perception.launch.py \
    slam_backend:=isaac \
    enable_3d_reconstruction:=true \
    enable_apriltag:=true \
    camera_namespace:=camera
```

### isaac_visual_slam.launch.py
Visual SLAM only:
```bash
ros2 launch isaac_ros_integration isaac_visual_slam.launch.py \
    use_isaac_slam:=true \
    image_source:=isaac_sim
```

### isaac_nvblox.launch.py
3D reconstruction only:
```bash
ros2 launch isaac_ros_integration isaac_nvblox.launch.py \
    use_nvblox:=true \
    voxel_size:=0.05
```

### isaac_apriltag.launch.py
AprilTag detection only:
```bash
ros2 launch isaac_ros_integration isaac_apriltag.launch.py \
    enable_apriltag:=true \
    tag_family:=tag36h11
```

## Testing

### Unit Tests
```bash
python -m pytest tests/unit/autonomy/isaac_ros/ -v
```

### Integration Tests
```bash
# With Isaac Sim running
python -m pytest tests/integration/simulation/test_ros2_simulation_integration.py -v

# Docker-based
./scripts/run_isaac_sim_docker.sh test
```

### Performance Tests
```bash
# SLAM performance
ros2 launch isaac_ros_integration isaac_visual_slam.launch.py &
ros2 topic hz /visual_slam/tracking/slam_pose

# NvBlox performance
ros2 topic hz /nvblox/costmap_2d
```

## Troubleshooting

### Common Issues

**Isaac ROS packages not found:**
```bash
# Re-run setup
./src/autonomy/isaac_ros_integration/scripts/setup_isaac_ros.sh

# Check installation
ros2 pkg list | grep isaac_ros
```

**Docker GPU not working:**
```bash
# Check nvidia-docker
nvidia-docker version

# Test GPU in container
docker run --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```

**X11 forwarding fails:**
```bash
# Allow local connections
xhost +local:docker

# Check DISPLAY
export DISPLAY=:0
```

**Isaac Sim crashes:**
```bash
# Check logs
docker logs urc2026-isaac-sim-gui

# Clear cache
rm -rf ~/.cache/isaac-sim
```

### Debug Mode

Enable debug logging:
```bash
export RCUTILS_LOGGING_SEVERITY=DEBUG
ros2 launch isaac_ros_integration isaac_perception.launch.py
```

## File Structure

```
urc-machiato-2026/
├── src/autonomy/isaac_ros_integration/
│   ├── launch/
│   │   ├── isaac_perception.launch.py
│   │   ├── isaac_visual_slam.launch.py
│   │   ├── isaac_nvblox.launch.py
│   │   └── isaac_apriltag.launch.py
│   ├── config/
│   │   ├── isaac_ros.yaml
│   │   └── apriltag_tags.yaml
│   ├── scripts/
│   │   ├── setup_isaac_ros.sh
│   │   └── switch_slam_backend.py
│   ├── package.xml
│   └── CMakeLists.txt
├── docker/
│   ├── Dockerfile.isaac-sim
│   └── docker-compose.isaac-sim.yml
├── simulation/isaac/
│   └── setup_mars_scene.py
└── scripts/
    └── run_isaac_sim_docker.sh
```

## Next Steps

1. **Test Installation:**
   ```bash
   ros2 launch isaac_ros_integration isaac_perception.launch.py
   ```

2. **Run Simulation:**
   ```bash
   ./scripts/run_isaac_sim_docker.sh gui
   ```

3. **Verify Topics:**
   ```bash
   ros2 topic list | grep -E "(visual_slam|nvblox|apriltag)"
   ```

4. **Monitor Performance:**
   ```bash
   ros2 topic hz /visual_slam/tracking/slam_pose
   ```

## References

- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/)
- [URC Isaac Sim Setup](ISAAC_SIM_SETUP.md)
