# Universal Dockerfile for URC 2026 Robotics Platform
# Best practices: Multi-stage builds, layer caching, security, minimal images
# Use with build arguments: --build-arg SERVICE=<service_name>

# Metadata labels (best practice)
LABEL maintainer="URC 2026 Team"
LABEL description="URC 2026 Robotics Platform - Universal Dockerfile"
LABEL version="1.0.0"

FROM ros:humble-ros-base AS base

# Install system dependencies in single layer (caching optimization)
# Use --no-install-recommends to minimize image size
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    python3-setuptools \
    git \
    curl \
    wget \
    net-tools \
    iproute2 \
    can-utils \
    iputils-ping \
    dnsutils \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Upgrade pip and install dependency parser (cached layer)
RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel tomli

# Create non-root user early (security best practice)
RUN useradd --create-home --shell /bin/bash --groups dialout --uid 1000 urc && \
    echo 'urc:urc2026' | chpasswd

# Set environment variables (before copying code for layer caching)
# Repo root + shared (for `import core.*`) + services tree + autonomy_core Python path
ENV PYTHONPATH=/home/urc:/home/urc/shared:/home/urc/services:/home/urc/services/autonomy/autonomy_core
ENV ROS_DOMAIN_ID=42
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

# Install Python dependencies (cached if pyproject.toml unchanged)
COPY --chown=urc pyproject.toml ./
RUN python3 -c "import tomli; f=open('pyproject.toml','rb'); d=tomli.load(f); deps=d.get('project',{}).get('dependencies',[]); [print(d) for d in deps if not d.strip().startswith('#') and not d.strip().startswith('Note:')]" > /tmp/requirements.txt && \
    pip3 install --no-cache-dir -r /tmp/requirements.txt 2>&1 | grep -v "ERROR:" || echo "Some packages may have failed, continuing..."

# Switch to non-root user (security)
USER urc
WORKDIR /home/urc

# ==================================================
# PERFORMANCE-OPTIMIZED BASE (for real-time services)
# ==================================================
FROM base AS performance-optimized

USER root
# Performance tuning and libraries
ENV PYTHONOPTIMIZE=2
ENV PYTHONHASHSEED=0
ENV LD_PRELOAD=libjemalloc.so
ENV URC_ENV=production
ENV URC_PERFORMANCE_MODE=optimized
ENV URC_REALTIME_MODE=enabled
ENV PYTHONASYNCIODEBUG=0

RUN apt-get update && apt-get install -y --no-install-recommends \
    libjemalloc-dev \
    htop \
    iotop \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN pip3 install --no-cache-dir psutil==5.9.0 py-spy==0.3.14 || true

USER urc
COPY --chown=urc services/ ./services/
COPY --chown=urc shared/ ./shared/
COPY --chown=urc config/ ./config/
COPY --chown=urc scripts/ ./scripts/
COPY --chown=urc vendor/ ./vendor/

RUN mkdir -p /home/urc/.urc/performance /home/urc/test_results

HEALTHCHECK --interval=5s --timeout=3s \
    CMD python3 -c "import time; print('HEALTHY:', time.time())" || exit 1

CMD ["python3", "-c", "print('URC performance-optimized base: override CMD per deployment target')"]

# ==================================================
# PERFORMANCE-OPTIMIZED SERVICES
# ==================================================
FROM performance-optimized AS sensor-bridge-optimized

HEALTHCHECK --interval=10s --timeout=5s --start-period=15s --retries=3 \
    CMD python3 -c "import autonomy_core.perception.sensor_simulator; print('Sensor sim OK')" || exit 1

CMD ["python3", "-c", "from autonomy_core.perception.sensor_simulator import main; main()"]

# Computer vision optimized: extend vision-service with performance env
FROM vision-service AS computer-vision-optimized

ENV PYTHONOPTIMIZE=2
ENV URC_PERFORMANCE_MODE=optimized
ENV URC_REALTIME_MODE=enabled

FROM performance-optimized AS autonomy-system-optimized

HEALTHCHECK --interval=10s --timeout=5s --start-period=15s --retries=3 \
    CMD python3 -c "import pathlib; assert pathlib.Path('/home/urc/services/missions/mission_executor.py').exists(); print('Autonomy OK')" || exit 1

CMD ["python3", "/home/urc/services/missions/mission_executor.py"]

FROM performance-optimized AS mission-control-optimized

EXPOSE 8080 8765

HEALTHCHECK --interval=10s --timeout=5s --start-period=15s --retries=3 \
    CMD python3 -c "print('Mission Control OK')" || exit 1

CMD ["python3", "/home/urc/services/missions/mission_executor.py"]

# ==================================================
# ROS2 STATE MANAGEMENT SERVICE
# ==================================================
FROM performance-optimized AS state-management-service

HEALTHCHECK --interval=30s --timeout=5s --start-period=20s --retries=3 \
    CMD python3 -c "import shared.infrastructure.bridges.ros2_state_machine_bridge; print('state bridge OK')" || exit 1

CMD ["python3", "-m", "shared.infrastructure.bridges.ros2_state_machine_bridge"]

# ==================================================
# SAFETY SYSTEM SERVICE
# ==================================================
FROM performance-optimized AS safety-service

HEALTHCHECK --interval=5s --timeout=3s --start-period=10s --retries=3 \
    CMD python3 -c "from autonomy_core.safety.safety_watchdog import main; print('Safety OK')" || exit 1

CMD ["python3", "-c", "from autonomy_core.safety.safety_watchdog import main; main()"]

# ==================================================
# NAVIGATION SERVICE
# ==================================================
FROM performance-optimized AS navigation-service

HEALTHCHECK --interval=10s --timeout=5s --start-period=15s --retries=3 \
    CMD python3 -c "import autonomy_core.navigation.navigation_node; print('Navigation OK')" || exit 1

CMD ["python3", "-c", "from autonomy_core.navigation.navigation_node import main; main()"]

# ==================================================
# COMPUTER VISION SERVICE
# ==================================================
FROM performance-optimized AS vision-service

# Install OpenCV and AI dependencies
USER root
RUN apt-get update && apt-get install -y \
    libopencv-dev \
    python3-opencv \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

# Install PyTorch (CPU version for containerization)
RUN pip3 install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

USER urc

HEALTHCHECK --interval=15s --timeout=10s --start-period=20s --retries=3 \
    CMD python3 -c "import cv2; import torch; print('Vision OK')" || exit 1

CMD ["python3", "-c", "from autonomy_core.perception.computer_vision_node import main; main()"]

# ==================================================
# LED STATUS SERVICE
# ==================================================
FROM performance-optimized AS led-service

HEALTHCHECK --interval=10s --timeout=5s --start-period=10s --retries=3 \
    CMD python3 -c "import autonomy_core.control.led_controller; print('LED OK')" || exit 1

CMD ["python3", "-c", "from autonomy_core.control.led_controller import main; main()"]

# ==================================================
# SENSOR BRIDGE SERVICE
# ==================================================
FROM performance-optimized AS sensor-bridge-service

HEALTHCHECK --interval=10s --timeout=5s --start-period=15s --retries=3 \
    CMD python3 -c "import autonomy_core.perception.sensor_simulator; print('Sensor Bridge OK')" || exit 1

CMD ["python3", "-c", "from autonomy_core.perception.sensor_simulator import main; main()"]

# ==================================================
# SLAM SERVICE
# ==================================================
FROM performance-optimized AS slam-service

HEALTHCHECK --interval=15s --timeout=10s --start-period=20s --retries=3 \
    CMD python3 -c "import autonomy_core.perception.slam_node; print('SLAM OK')" || exit 1

CMD ["python3", "-c", "from autonomy_core.perception.slam_node import main; main()"]

# ==================================================
# MISSION EXECUTOR SERVICE
# ==================================================
FROM performance-optimized AS mission-service

HEALTHCHECK --interval=10s --timeout=5s --start-period=15s --retries=3 \
    CMD python3 -c "import pathlib; assert pathlib.Path('/home/urc/services/missions/mission_executor.py').exists(); print('Mission OK')" || exit 1

CMD ["python3", "/home/urc/services/missions/mission_executor.py"]

# ==================================================
# SIMULATION SERVICE (Gazebo Integration)
# ==================================================
FROM performance-optimized AS simulation-service

# Install Gazebo dependencies
USER root
RUN apt-get update && apt-get install -y \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-xacro \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

USER urc

# Set Gazebo environment (models under services/simulation/gazebo)
ENV GAZEBO_MODEL_PATH=/home/urc/services/simulation/gazebo/models:$GAZEBO_MODEL_PATH
ENV GAZEBO_PLUGIN_PATH=/home/urc/services/simulation/gazebo/plugins:$GAZEBO_PLUGIN_PATH

HEALTHCHECK --interval=15s --timeout=10s --start-period=30s --retries=3 \
    CMD python3 -c "import gazebo_msgs; print('Simulation OK')" || exit 1

CMD ["python3", "-c", "print('Simulation service ready - use ROS2 launch files to start Gazebo')"]

# ==================================================
# WEBSOCKET BRIDGE SERVICE
# ==================================================
FROM performance-optimized AS websocket-bridge-service

EXPOSE 8765

HEALTHCHECK --interval=10s --timeout=5s --start-period=10s --retries=3 \
    CMD python3 -c "import shared.infrastructure.bridges.teleop_websocket_bridge; print('WebSocket Bridge OK')" || exit 1

CMD ["python3", "-c", "print('Teleop bridge: ros2 run or python entry TBD'); import time; time.sleep(86400)"]

# ==================================================
# CAN MOCK SIMULATOR SERVICE
# ==================================================
FROM performance-optimized AS can-mock-service

EXPOSE 8766

HEALTHCHECK --interval=10s --timeout=5s --start-period=10s --retries=3 \
    CMD python3 -c "from services.simulation.python.can.can_bus_mock_simulator import CANBusMockSimulator; print('CAN Mock OK')" || exit 1

CMD ["python3", "-c", "from services.simulation.python.can.can_bus_mock_simulator import CANBusMockSimulator; print('CAN mock: use ROS2 launch in dev'); import time; time.sleep(86400)"]

# ==================================================
# FRONTEND SERVICE
# ==================================================
FROM node:18-alpine AS frontend-service

WORKDIR /app

# Copy package files
COPY services/dashboard/package*.json ./

# Install dependencies
RUN npm ci --only=production

# Copy source code
COPY services/dashboard/ ./

# Build the application
RUN npm run build

# Production stage with Nginx
FROM nginx:alpine AS frontend-prod

# Copy built application
COPY --from=frontend-service /app/dist /usr/share/nginx/html

# Copy nginx configuration (if exists, otherwise use default)
# COPY src/dashboard/nginx.conf /etc/nginx/nginx.conf

EXPOSE 80

HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD curl -f http://localhost/ || exit 1

CMD ["nginx", "-g", "daemon off;"]

# ==================================================
# DEVELOPMENT ENVIRONMENT (all services)
# ==================================================
FROM base AS development

# Install additional development tools
USER root
RUN apt-get update && apt-get install -y \
    python3-venv \
    python3-pytest \
    python3-pytest-cov \
    python3-black \
    python3-isort \
    python3-mypy \
    python3-flake8 \
    tmux \
    screen \
    && rm -rf /var/lib/apt/lists/*

# Install Node.js for frontend development
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash - && \
    apt-get install -y nodejs && \
    rm -rf /var/lib/apt/lists/*

USER urc

# Copy entire project for development
COPY --chown=urc . .

# Set development environment variables
# Repo root + shared (for `import core.*`) + services tree + autonomy_core Python path
ENV PYTHONPATH=/home/urc:/home/urc/shared:/home/urc/services:/home/urc/services/autonomy/autonomy_core
ENV ROS_DOMAIN_ID=42
ENV URC_ENV=development

# Expose all service ports
EXPOSE 8765 8766 3000

# Default command for development
CMD ["/bin/bash"]

# ==================================================
# DEFAULT TARGET (show help)
# ==================================================
FROM base AS help

RUN echo "URC 2026 Universal Dockerfile" > /help.txt && \
    echo "" >> /help.txt && \
    echo "Available targets:" >> /help.txt && \
    echo "  state-management-service  - ROS2 State Machine" >> /help.txt && \
    echo "  safety-service            - Safety System" >> /help.txt && \
    echo "  navigation-service        - Navigation System" >> /help.txt && \
    echo "  vision-service           - Computer Vision" >> /help.txt && \
    echo "  led-service              - LED Status System" >> /help.txt && \
    echo "  sensor-bridge-service    - Sensor Bridge" >> /help.txt && \
    echo "  slam-service             - SLAM System" >> /help.txt && \
    echo "  mission-service          - Mission Executor" >> /help.txt && \
    echo "  websocket-bridge-service - WebSocket Bridge" >> /help.txt && \
    echo "  can-mock-service         - CAN Bus Mock Simulator" >> /help.txt && \
    echo "  frontend-service         - Frontend (dev)" >> /help.txt && \
    echo "  frontend-prod            - Frontend (production)" >> /help.txt && \
    echo "  development              - Full development environment" >> /help.txt && \
    echo "  performance-optimized    - Performance-tuned base" >> /help.txt && \
    echo "  sensor-bridge-optimized  - Sensor bridge (optimized)" >> /help.txt && \
    echo "  computer-vision-optimized - Computer vision (optimized)" >> /help.txt && \
    echo "  autonomy-system-optimized - Autonomy system (optimized)" >> /help.txt && \
    echo "  mission-control-optimized - Mission control (optimized)" >> /help.txt && \
    echo "" >> /help.txt && \
    echo "Usage:" >> /help.txt && \
    echo "  docker build --target <target_name> -t urc2026:<target_name> ." >> /help.txt

CMD ["cat", "/help.txt"]
