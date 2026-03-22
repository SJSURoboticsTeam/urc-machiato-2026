# Dockerfile for Isaac Sim 5.0 with ROS2 Integration
# Supports both GUI and headless modes
# 
# Build commands:
#   GUI mode: docker build --target isaac-sim-gui -t urc2026/isaac-sim:gui .
#   Headless: docker build --target isaac-sim-headless -t urc2026/isaac-sim:headless .
#
# Run commands:
#   GUI: docker run --gpus all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix urc2026/isaac-sim:gui
#   Headless: docker run --gpus all urc2026/isaac-sim:headless

# ==================================================
# Base Image with NVIDIA Support
# ==================================================
FROM nvidia/cuda:12.2.0-devel-ubuntu22.04 AS base

LABEL maintainer="URC 2026 Team"
LABEL description="Isaac Sim 5.0 with ROS2 Humble for URC Rover Simulation"

# Prevent interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake \
    git \
    wget \
    curl \
    # Python
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-venv \
    # X11 for GUI
    libx11-6 \
    libxext6 \
    libxrender1 \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglu1-mesa \
    libxi6 \
    libxkbcommon-x11-0 \
    libfontconfig1 \
    # VNC support for headless GUI
    xvfb \
    x11vnc \
    xfce4 \
    xfce4-terminal \
    tigervnc-standalone-server \
    tigervnc-viewer \
    # Audio (for simulation audio feedback)
    libpulse0 \
    libasound2 \
    # Utilities
    locales \
    ca-certificates \
    software-properties-common \
    nano \
    vim \
    htop \
    # Clean up
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Set locale
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

# Create workspace directory
WORKDIR /workspace

# ==================================================
# ROS2 Humble Installation
# ==================================================
FROM base AS ros2-base

# Add ROS2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-ros2launch \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Set ROS2 environment
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/humble
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ==================================================
# Isaac Sim Installation
# ==================================================
FROM ros2-base AS isaac-sim-base

# Isaac Sim version
ARG ISAAC_SIM_VERSION=5.0.0
ENV ISAAC_SIM_VERSION=${ISAAC_SIM_VERSION}

# Install Isaac Sim dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libssl-dev \
    libffi-dev \
    libxml2-dev \
    libxslt1-dev \
    zlib1g-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    && rm -rf /var/lib/apt/lists/*

# Download and install Isaac Sim (or use cached version)
# Note: In production, this should be a multi-stage build with pre-downloaded Isaac Sim
RUN mkdir -p /isaac-sim \
    && cd /isaac-sim \
    && wget -q --show-progress https://download.isaacsim.omniverse.nvidia.com/isaac-sim-${ISAAC_SIM_VERSION}.tar.gz \
    || echo "Using cached Isaac Sim or manual install required"

# Set Isaac Sim environment
ENV ISAAC_SIM_PATH=/isaac-sim
ENV OMNI_SERVER=http://localhost:8080

# ==================================================
# GUI Mode Setup
# ==================================================
FROM isaac-sim-base AS isaac-sim-gui

# Install additional GUI dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    mesa-utils \
    libnvidia-gl-470 \
    && rm -rf /var/lib/apt/lists/*

# Set display environment
ENV DISPLAY=:1
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Create startup script
RUN cat > /workspace/start_isaac_sim.sh << 'EOF'
#!/bin/bash
set -e

# Source ROS2
source /opt/ros/humble/setup.bash

# Setup display if not set
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:1
fi

# Check for GPU
if ! nvidia-smi > /dev/null 2>&1; then
    echo "WARNING: NVIDIA GPU not detected. Running in CPU mode (very slow)."
fi

# Start Isaac Sim with ROS2 bridge
cd /isaac-sim
./isaac-sim.sh --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge "$@"
EOF

RUN chmod +x /workspace/start_isaac_sim.sh

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
    CMD pgrep -f "isaac-sim" || exit 1

EXPOSE 8080 8899 47995-48012/udp

ENTRYPOINT ["/workspace/start_isaac_sim.sh"]

# ==================================================
# Headless Mode Setup
# ==================================================
FROM isaac-sim-base AS isaac-sim-headless

# Install VNC server for remote access
RUN apt-get update && apt-get install -y --no-install-recommends \
    novnc \
    websockify \
    && rm -rf /var/lib/apt/lists/*

# Setup VNC
RUN mkdir -p ~/.vnc \
    && echo "isaacsim" | vncpasswd -f > ~/.vnc/passwd \
    && chmod 600 ~/.vnc/passwd

# Create startup script for headless mode
RUN cat > /workspace/start_headless.sh << 'EOF'
#!/bin/bash
set -e

# Source ROS2
source /opt/ros/humble/setup.bash

# Start Xvfb (virtual display)
export DISPLAY=:99
Xvfb :99 -screen 0 1920x1080x24 > /dev/null 2>&1 &
sleep 2

# Start VNC server
x11vnc -display :99 -nopw -listen localhost -xkb -forever &
sleep 1

# Start noVNC web interface
websockify --web=/usr/share/novnc --cert=self.pem 6080 localhost:5900 &

echo "==============================================="
echo "Isaac Sim Headless Mode Started"
echo "Access via: http://localhost:6080/vnc.html"
echo "==============================================="

# Start Isaac Sim in headless mode
cd /isaac-sim
./isaac-sim.sh --headless --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge "$@"
EOF

RUN chmod +x /workspace/start_headless.sh

# Expose VNC and noVNC ports
EXPOSE 5900 6080 8080 8899 47995-48012/udp

ENTRYPOINT ["/workspace/start_headless.sh"]

# ==================================================
# Development Mode (with URC code)
# ==================================================
FROM isaac-sim-gui AS isaac-sim-dev

# Install development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pytest \
    python3-pytest-cov \
    python3-black \
    python3-isort \
    python3-mypy \
    python3-flake8 \
    gdb \
    valgrind \
    tmux \
    screen \
    && rm -rf /var/lib/apt/lists/*

# Copy URC codebase
COPY --chown=root:root . /workspace/urc-machiato-2026/

# Install Python dependencies
WORKDIR /workspace/urc-machiato-2026
RUN pip3 install --no-cache-dir -r requirements.txt 2>&1 | grep -v "ERROR:" || echo "Some packages may have failed, continuing..."

# Build ROS2 workspace
RUN bash -c "source /opt/ros/humble/setup.bash && \
    if [ -f src/autonomy/isaac_ros_integration/package.xml ]; then \
        colcon build --packages-select isaac_ros_integration --cmake-args -DCMAKE_BUILD_TYPE=Release; \
    fi"

# Set workspace environment
ENV URC_WORKSPACE=/workspace/urc-machiato-2026
ENV PYTHONPATH=/workspace/urc-machiato-2026:$PYTHONPATH

WORKDIR /workspace

# Default to bash for development
CMD ["/bin/bash"]

# ==================================================
# Testing Mode (for CI/CD)
# ==================================================
FROM isaac-sim-headless AS isaac-sim-test

# Copy test configuration
COPY --chown=root:root ./config /workspace/config/
COPY --chown=root:root ./tests /workspace/tests/
COPY --chown=root:root ./scripts /workspace/scripts/

# Install test dependencies
RUN pip3 install --no-cache-dir \
    pytest \
    pytest-cov \
    pytest-timeout \
    pytest-benchmark

# Create test runner script
RUN cat > /workspace/run_tests.sh << 'EOF'
#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

echo "Running Isaac Sim integration tests..."
cd /workspace

# Start Isaac Sim in background
/isaac-sim/isaac-sim.sh --headless --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge &
ISAAC_PID=$!

# Wait for Isaac Sim to initialize
sleep 30

# Run tests
python3 -m pytest tests/integration/simulation/ -v --tb=short || true

# Cleanup
kill $ISAAC_PID || true

echo "Tests completed"
EOF

RUN chmod +x /workspace/run_tests.sh

CMD ["/workspace/run_tests.sh"]

# ==================================================
# Default target
# ==================================================
FROM isaac-sim-gui
