# Unified Docker System for URC 2026

This unified Docker system provides consistent testing, deployment, and simulation capabilities for the URC 2026 robotics platform. It consolidates multiple Docker configurations into a single, environment-aware system.

## 🚀 Quick Start

```bash
# Run the demo to see available options
./scripts/demo_unified_docker.sh

# Deploy development environment
./scripts/docker_deploy.sh deploy dev

# Run unit tests
./scripts/docker_deploy.sh test unit

# Start simulation
./scripts/docker_deploy.sh sim
```

## 📁 File Structure

```
docker/
├── docker-compose.unified.yml    # Main orchestration file
├── Dockerfile.unified           # Multi-stage build targets
└── ...

config/
├── docker_development.yaml      # Development configuration
├── docker_testing.yaml         # Testing configuration
├── docker_simulation.yaml       # Simulation configuration
└── ...

scripts/
├── docker_deploy.sh             # Main deployment script
├── demo_unified_docker.sh      # Quick demo script
└── ...

.env.unified                     # Environment variables
```

## 🎯 Environments

### Development (`dev`)
- Full development stack with hot reload
- Mock systems for hardware simulation
- Source code mounting
- Debug logging enabled
- Frontend dev server

### Testing (`test`)
- Optimized for CI/CD and local testing
- Parallel test execution
- Performance benchmarking
- Coverage reporting
- Headless simulation

### Simulation (`sim`)
- Gazebo with Mars environment
- Realistic physics and sensors
- GUI visualization
- Scenario testing
- Performance analysis

### Production (`prod`)
- Optimized for competition
- Real hardware integration
- Minimal logging
- Performance monitoring
- Safety systems active

## 🔧 Core Features

### Unified Orchestration
- **Single compose file** with environment profiles
- **Multi-stage builds** for different service targets
- **Environment-specific configurations** via YAML files
- **Consistent networking** across all environments

### Service Targets
The unified Dockerfile provides multiple build targets:

```dockerfile
# Services
config-service           # Configuration management
can-mock-service        # CAN bus simulation
websocket-bridge        # Real-time communication
navigation-service       # Autonomous navigation
vision-service         # Computer vision
safety-service         # Safety monitoring
simulation-service     # Gazebo simulation
sensor-simulator       # Sensor data generation

# Testing
test-runner           # Unified test execution
integration-test      # Integration testing
simulation-test       # Simulation testing

# Frontend
frontend-dev         # Development server
frontend-prod        # Production build

# Monitoring
monitoring-service    # Prometheus + Grafana
```

### Parallel Testing
```bash
# Run tests with 8 parallel workers
./scripts/docker_deploy.sh test unit 8

# Run performance tests
./scripts/docker_deploy.sh perf

# Run all test suites
./scripts/docker_deploy.sh test all
```

## 🐳 Docker Compose Profiles

### Development Profile
```yaml
profiles: [dev]
services:
  - ros2-core
  - config-service
  - can-mock-simulator
  - websocket-bridge
  - state-management
  - safety-system
  - navigation-system
  - computer-vision
  - frontend-dev
  - monitoring-stack
```

### Testing Profile
```yaml
profiles: [test]
services:
  - ros2-core
  - config-service
  - test-runner
  - performance-tester
  - sensor-simulator
```

### Simulation Profile
```yaml
profiles: [sim]
services:
  - ros2-core
  - config-service
  - simulation-service
  - sensor-simulator
  - navigation-system
  - computer-vision
  - monitoring-stack
```

## 🌍 Simulation Environment

### Mars World Configuration
- **Gravity**: 38% of Earth (3.71 m/s²)
- **Atmosphere**: Thin CO₂ (610 Pa)
- **Terrain**: Rocky, sandy, cratered
- **Temperature**: -63°C to 20°C
- **Wind**: Realistic wind simulation
- **Dust**: Particle effects

### Supported Scenarios
1. **Autonomous Navigation**: Waypoint following
2. **Sample Collection**: Precision positioning
3. **Equipment Delivery**: Task execution
4. **Obstacle Course**: Navigation challenge

### Robot Configuration
- **6-wheel drive** with swerve steering
- **RealSense D435** depth camera
- **SICK LMS111** LiDAR
- **MPU9250** IMU
- **U-blox NEO-M8P** GPS
- **6-DOF arm** for manipulation

## 📊 Monitoring & Observability

### Prometheus Metrics
- ROS2 topic rates and latency
- Service health and performance
- Resource utilization
- Error rates and recovery times

### Grafana Dashboards
- System overview
- Performance metrics
- Safety system status
- Simulation telemetry

### Health Checks
```bash
# Check environment health
./scripts/docker_deploy.sh health dev

# View container status
./scripts/docker_deploy.sh status dev

# Show live logs
./scripts/docker_deploy.sh logs dev
```

## 🔒 Security & Safety

### Container Security
- **Non-root user** execution
- **Minimal base images**
- **Security scanning** in CI/CD
- **Resource limits** enforced

### Safety Systems
- **Multi-layer redundancy** (primary/secondary/tertiary)
- **Circuit breaker patterns**
- **Automatic recovery** mechanisms
- **Emergency stop** procedures

## 🛠️ Development Workflow

### 1. Local Development
```bash
# Deploy dev environment
./scripts/docker_deploy.sh deploy dev

# Work with hot reload
# Source code changes automatically reload
# Frontend dev server on port 3000
# WebSocket bridge on port 8765
```

### 2. Testing
```bash
# Run tests in parallel
./scripts/docker_deploy.sh test unit 8

# Run performance benchmarks
./scripts/docker_deploy.sh perf

# Full test suite
./scripts/docker_deploy.sh test all
```

### 3. Simulation
```bash
# Start Mars simulation
./scripts/docker_deploy.sh sim

# Custom world and robot
./scripts/docker_deploy.sh sim mars_world advanced_rover
```

### 4. Production Deployment
```bash
# Deploy production stack
./scripts/docker_deploy.sh deploy prod

# Production monitoring
./scripts/docker_deploy.sh status prod
```

## 📋 Environment Variables

Create `.env.unified` to customize:

```bash
# Global
URC_ENV=development
ROS_DISTRO=humble
ROS_DOMAIN_ID=42

# Networking
WEBSOCKET_PORT=8765
API_PORT=8080
GAZEBO_GUI_PORT=11345

# Testing
TEST_PARALLEL_WORKERS=4
TEST_MEMORY_LIMIT=4G

# Simulation
SIMULATION_WORLD=urc_mars_world
SIMULATION_ROBOT=urc_rover
```

## 🔄 CI/CD Integration

### GitHub Actions
```yaml
- name: Run Tests
  run: ./scripts/docker_deploy.sh test all

- name: Performance Tests
  run: ./scripts/docker_deploy.sh perf

- name: Build Production
  run: ./scripts/docker_deploy.sh build prod
```

### Pipeline Features
- **Parallel test execution**
- **Docker layer caching**
- **Artifact collection**
- **Performance regression detection**
- **Coverage reporting**

## 🐛 Troubleshooting

### Common Issues

1. **Permission Denied**
   ```bash
   chmod +x scripts/docker_deploy.sh
   ```

2. **Docker Daemon Not Running**
   ```bash
   sudo systemctl start docker
   sudo usermod -aG docker $USER
   ```

3. **Port Conflicts**
   ```bash
   # Check ports
   netstat -tulpn | grep :8765
   
   # Modify in .env.unified
   WEBSOCKET_PORT=8766
   ```

4. **Container Resource Limits**
   ```bash
   # Increase test limits
   export TEST_MEMORY_LIMIT=6G
   export TEST_CPU_LIMIT=4.0
   ```

### Debug Mode
```bash
# Enable debug output
DEBUG=true ./scripts/docker_deploy.sh status dev

# Open shell in container
./scripts/docker_deploy.sh shell dev config

# View container logs
./scripts/docker_deploy.sh logs dev
```

## 📈 Performance Optimization

### Build Caching
- **Layer caching** optimized with pyproject.toml
- **Multi-stage builds** reduce final image size
- **BuildKit** enabled for parallel builds

### Runtime Optimization
- **Resource limits** prevent system overload
- **Health checks** ensure service availability
- **Parallel execution** for faster testing

### Simulation Performance
- **GPU acceleration** when available
- **Multi-threading** for physics simulation
- **Real-time factor** control

## 🎯 Best Practices

1. **Always use the deployment script** for consistency
2. **Check container health** before connecting services
3. **Use parallel testing** for faster feedback
4. **Monitor resource usage** during long simulations
5. **Clean up containers** after testing to free resources
6. **Use environment-specific configs** for different scenarios

## 🤝 Contributing

When adding new services:

1. **Add service target** to `docker/Dockerfile.unified`
2. **Configure service** in `docker/docker-compose.unified.yml`
3. **Add environment config** in `config/docker_*.yaml`
4. **Update deployment script** if needed
5. **Add tests** for the new service
6. **Update documentation**

## 📞 Support

For issues with the unified Docker system:

1. Check the troubleshooting section
2. Run health checks to diagnose problems
3. Use debug mode for detailed logging
4. Check container logs for error messages
5. Verify environment variables are set correctly

---

This unified Docker system provides a solid foundation for development, testing, simulation, and production deployment of the URC 2026 robotics platform. It's designed for both developer productivity and competition reliability.