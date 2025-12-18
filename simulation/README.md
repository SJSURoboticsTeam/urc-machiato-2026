# Centralized URC 2026 Simulation Framework

## Overview

The **Centralized Simulation Framework** provides a unified, comprehensive simulation environment for URC 2026 rover development. All simulation components (sensors, networks, rover physics, environments) are consolidated into a single, consistent framework that ensures **simulation-based testing** before hardware validation.

**Key Features:**
- ✅ **Three-Tier Environment Testing** (PERFECT → REAL_LIFE → EXTREME)
- ✅ **Network Emulation** (WiFi, cellular, satellite, extreme conditions)
- ✅ **Comprehensive Sensor Simulation** (GPS, IMU, camera, LiDAR)
- ✅ **Realistic Rover Physics** (kinematics, dynamics, terrain interaction)
- ✅ **Unified Data Recording & Analysis**
- ✅ **Hardware Validation Preparation**

**Enhanced Features v2.0:**
- 🔍 **Structured Logging**: JSON-formatted logs with context and correlation IDs
- 📊 **Real-time Monitoring**: Performance metrics, system resources, alerts
- 🕵️ **Comprehensive Tracing**: Operation profiling, debugging, performance analysis
- 🤖 **RL Training Support**: Episode tracking, reward functions, curriculum learning
- 📈 **Advanced Analytics**: Performance reports, bottleneck identification, optimization insights

---

## Architecture

```
simulation/
├── core/                    # Core simulation engine
│   ├── simulation_manager.py   # Central orchestrator
│   └── time_manager.py         # Time management
├── sensors/                 # Sensor simulation
│   ├── base_sensor.py          # Abstract sensor interface
│   ├── gps_simulator.py        # GPS simulation
│   ├── imu_simulator.py        # IMU simulation
│   └── sensor_factory.py       # Sensor creation
├── network/                 # Network simulation
│   ├── network_emulator.py     # Network conditions
│   └── network_factory.py      # Network creation
├── rover/                   # Rover simulation
│   ├── base_rover.py           # Abstract rover interface
│   ├── urc_rover.py            # URC rover implementation
│   └── rover_factory.py        # Rover creation
├── environments/            # Environment simulation
│   ├── base_environment.py     # Abstract environment
│   ├── perfect_environment.py  # Ideal conditions
│   ├── real_life_environment.py # Field conditions
│   ├── extreme_environment.py  # Worst-case conditions
│   └── environment_factory.py  # Environment creation
├── scenarios/               # Test scenarios
├── tools/                   # Utilities
│   └── data_recorder.py        # Data recording
├── config/                  # Configuration files
├── examples/                # Usage examples
└── README.md               # This file
```

---

## Quick Start

### 1. Basic Simulation

```python
from simulation import SimulationManager

# Create simulation
manager = SimulationManager()

# Configure
config = {
    "environment": {"tier": "real_life"},
    "sensors": [
        {"name": "gps", "type": "gps", "update_rate": 10.0},
        {"name": "imu", "type": "imu", "update_rate": 100.0},
    ],
    "network": {"profile": "rural_wifi"},
    "rover": {"model": "urc_rover"},
}

# Initialize and run
manager.initialize(config)
manager.start()

# Step simulation
for i in range(1000):
    state = manager.step(0.01)  # 10ms steps
    print(f"Time: {state['timestamp']:.2f}")

manager.stop()
```

### 2. Run Complete Demo

```bash
cd /path/to/urc-machiato-2026
python3 simulation/examples/complete_simulation_demo.py
```

This runs a 30-second simulation across all three tiers and generates comprehensive reports.

### 3. Custom Scenario

```python
from simulation import SimulationFactory

# Create components
environment = SimulationFactory.create_environment("real_life")
network = SimulationFactory.create_network("rural_wifi")
rover = SimulationFactory.create_rover("urc_rover")
gps = SimulationFactory.create_sensor("gps", {"update_rate": 10.0})

# Custom simulation loop
for i in range(1000):
    # Update environment
    env_state = environment.step(0.01)

    # Generate sensor data
    gps_data = gps.step(0.01, env_state)

    # Simulate network delay
    network.send_message(gps_data)

    # Update rover
    rover.step(0.01, {"gps": gps_data})
```

---

## Three-Tier Environment Testing

### PERFECT Environment
**Purpose:** Establish baseline performance with no environmental effects.

```python
config = {
    "environment": {"tier": "perfect"},
    # No noise, perfect sensors, ideal conditions
}
```

**Characteristics:**
- ✅ Zero sensor noise/bias
- ✅ Perfect network (0ms latency)
- ✅ Ideal terrain (flat, perfect traction)
- ✅ 100% sensor reliability
- ✅ Expected: 100% success rate

### REAL_LIFE Environment
**Purpose:** Test typical URC field conditions.

```python
config = {
    "environment": {"tier": "real_life"},
    # Realistic desert conditions
}
```

**Characteristics:**
- 🟡 Moderate sensor degradation (20% accuracy loss)
- 🟡 Rural WiFi network (85ms latency, 2% packet loss)
- 🟡 Desert terrain (35°C, moderate dust, variable traction)
- 🟡 95% sensor reliability
- 🟡 Expected: 90% success rate

### EXTREME Environment
**Purpose:** Test worst-case survival conditions.

```python
config = {
    "environment": {"tier": "extreme"},
    # Severe desert storm conditions
}
```

**Characteristics:**
- 🔴 Severe sensor degradation (70% accuracy loss)
- 🔴 Extreme network (1500ms latency, 15% packet loss)
- 🔴 Severe terrain (50°C, dust storm, poor traction)
- 🔴 70% sensor reliability
- 🔴 Expected: 60% success rate (survival mode)

---

## Network Emulation

### Available Profiles

| Profile | Latency | Packet Loss | Bandwidth | Use Case |
|---------|---------|-------------|-----------|----------|
| `perfect` | 0ms | 0% | Unlimited | Baseline |
| `rural_wifi` | 85ms | 2% | 25 Mbps | Typical field |
| `cellular_4g` | 125ms | 3% | 15 Mbps | Backup |
| `satellite` | 900ms | 1% | 5 Mbps | Remote |
| `extreme` | 1500ms | 15% | 1 Mbps | Worst-case |

### Usage

```python
from simulation import SimulationFactory

# Create network emulator
network = SimulationFactory.create_network("rural_wifi")

# Send message with network effects
success = network.send_message({"type": "sensor_data", "value": 42})
if success:
    # Message sent (may be delayed/lost)
    pass

# Get network statistics
stats = network.get_statistics()
print(f"Latency: {stats['average_latency_ms']:.1f}ms")
```

---

## Sensor Simulation

### Available Sensors

- **GPS**: Position, velocity, satellite data with accuracy degradation
- **IMU**: Angular velocity, linear acceleration with noise/bias
- **Camera**: Image data with visibility/dust effects (future)
- **LiDAR**: Point cloud data with dust attenuation (future)

### Sensor Configuration

```python
sensor_config = {
    "name": "primary_gps",
    "type": "gps",
    "update_rate": 10.0,  # Hz
    "position_noise_std": 2.5,  # meters
    "failure_rate": 0.001,  # 0.1% per update
}
```

### Environmental Effects

Sensors automatically apply environmental degradation:

```python
# Perfect environment
gps_data = gps.step(dt, env_state)  # No degradation

# Real-life environment
gps_data = gps.step(dt, env_state)  # 20% accuracy loss

# Extreme environment
gps_data = gps.step(dt, env_state)  # 70% accuracy loss
```

---

## Rover Physics Simulation

### Available Models

- **URC Rover**: 6-wheel differential drive with realistic physics

### Physics Features

- **Kinematics**: Differential drive with steering
- **Dynamics**: Mass, friction, motor characteristics
- **Terrain**: Traction, rolling resistance, slope effects
- **Environmental**: Wind resistance, thermal effects

### Configuration

```python
rover_config = {
    "model": "urc_rover",
    "mass": 75.0,  # kg
    "wheel_count": 6,
    "wheelbase": 0.8,  # meters
    "max_velocity": 1.5,  # m/s
    "max_acceleration": 0.8,  # m/s²
}
```

### Control Interface

```python
# Set velocity commands
rover.set_control_inputs(
    linear_velocity=1.0,    # m/s forward
    angular_velocity=0.5,   # rad/s turning
    steering_angle=0.0      # radians
)

# Emergency stop
rover.emergency_stop()
```

---

## Data Recording & Analysis

### Automatic Recording

```python
# Configure recording
config = {
    "recording": {
        "max_records": 10000,
        "record_interval": 0.1,  # 10Hz
        "compress_data": True,
    }
}

manager.initialize(config)
# Recording starts automatically
```

### Manual Recording

```python
from simulation.tools.data_recorder import DataRecorder

recorder = DataRecorder()
recorder.initialize({"max_records": 5000})

# Record simulation state
recorder.record(simulation_state)

# Save to file
recorder.save_to_file("simulation_data.json")
```

### Playback

```python
# Load recorded data
recorder.load_from_file("simulation_data.json")

# Playback
def analyze_frame(frame):
    print(f"Time: {frame['timestamp']}")

recorder.playback(analyze_frame)
```

---

## Advanced Usage

### Custom Environment

```python
from simulation.environments.base_environment import BaseEnvironment

class MarsEnvironment(BaseEnvironment):
    def __init__(self, config):
        super().__init__(config)
        self.temperature = -60  # Mars temperature
        self.atmosphere_density = 0.01  # Thin atmosphere

    def _initialize_conditions(self):
        # Mars-specific initialization
        pass

    def _update_weather(self, dt):
        # Mars weather patterns
        pass

    def _update_terrain(self, dt):
        # Mars terrain changes
        pass

# Register custom environment
from simulation.environments.environment_factory import EnvironmentFactory
EnvironmentFactory.register_environment_type("mars", MarsEnvironment)
```

### Custom Sensor

```python
from simulation.sensors.base_sensor import BaseSensor

class RadiationSensor(BaseSensor):
    def _generate_raw_data(self, environment_state):
        # Generate radiation data based on environment
        cosmic_radiation = 0.5  # mSv/hr base
        solar_flare = environment_state.get("solar_activity", 0)
        return {
            "radiation_level": cosmic_radiation + solar_flare,
            "particle_count": int(cosmic_radiation * 1000),
        }

# Register custom sensor
from simulation.sensors.sensor_factory import SensorFactory
SensorFactory.register_sensor_type("radiation", RadiationSensor)
```

### Multi-Rover Simulation

```python
# Create multiple rovers
rovers = []
for i in range(3):
    rover_config = {
        "model": "urc_rover",
        "name": f"rover_{i}",
        # Different starting positions
        "initial_position": [i * 10, 0, 0],
    }
    rover = SimulationFactory.create_rover(rover_config)
    rovers.append(rover)

# Simulate swarm behavior
for rover in rovers:
    # Coordinate movement
    pass
```

---

## Performance Benchmarks

### Typical Performance

| Component | Update Rate | Memory Usage | CPU Usage |
|-----------|-------------|--------------|-----------|
| Environment | 100Hz | ~1MB | <1% |
| GPS Sensor | 10Hz | ~500KB | <0.5% |
| IMU Sensor | 100Hz | ~1MB | <1% |
| Network | 1000Hz | ~2MB | <2% |
| Rover Physics | 100Hz | ~1MB | <1% |
| Data Recording | 10Hz | ~5MB/hour | <0.5% |

### Scaling

- **10 sensors**: ~10MB RAM, <5% CPU
- **100Hz simulation**: Real-time on modern hardware
- **1-hour recording**: ~300MB data
- **Multi-rover**: Linear scaling with rover count

---

## Testing & Validation

### Unit Tests

```bash
# Run all simulation tests
python3 -m pytest tests/comprehensive_integration_suite.py -v

# Run specific component tests
python3 -m pytest tests/integration/test_full_system_integration.py -v
python3 -m pytest tests/integration/test_navigation_comprehensive.py -v
```

### Integration Tests

```bash
# Run comprehensive integration suite
python3 tests/comprehensive_integration_suite.py
```

### Performance Tests

```bash
# Run performance benchmarks
python3 tests/performance/test_performance_load.py
```

### Validation Scripts

```bash
# Validate simulation accuracy
python3 simulation/tools/validate_simulation.py

# Compare simulation vs real data
python3 simulation/tools/compare_real_world.py
```

---

## Configuration Files

### Simulation Config

```yaml
# config/simulation_config.yaml
simulation:
  time_step: 0.01
  real_time_factor: 1.0

sensors:
  gps:
    update_rate: 10.0
    noise_std: 2.5
    failure_rate: 0.001

  imu:
    update_rate: 100.0
    gyro_noise_std: 0.01
    accel_noise_std: 0.1

network:
  default_profile: "rural_wifi"
  profiles:
    rural_wifi:
      latency_ms: 85
      jitter_ms: 25
      packet_loss: 0.02

rover:
  model: "urc_rover"
  mass: 75.0
  max_velocity: 1.5

recording:
  max_records: 10000
  compress_data: true
```

### Environment Presets

```yaml
# config/environment_presets.yaml
perfect:
  temperature: 25.0
  visibility: 1.0
  dust_density: 0.0
  terrain_difficulty: 0.0

real_life:
  temperature: 35.0
  visibility: 0.8
  dust_density: 0.3
  terrain_difficulty: 0.4

extreme:
  temperature: 50.0
  visibility: 0.2
  dust_density: 0.9
  terrain_difficulty: 0.9
```

---

## Troubleshooting

### Common Issues

**High CPU Usage**
```python
# Reduce update rates
config["sensors"][0]["update_rate"] = 5.0  # Instead of 10.0

# Increase time step
config["time"]["step_size"] = 0.02  # Instead of 0.01
```

**Memory Issues**
```python
# Reduce recording
config["recording"]["max_records"] = 1000  # Instead of 10000
config["recording"]["record_interval"] = 0.5  # Instead of 0.1
```

**Network Delays**
```python
# Use perfect network for debugging
config["network"]["profile"] = "perfect"
```

**Sensor Failures**
```python
# Disable failures for testing
sensor_config["failure_rate"] = 0.0
```

### Debug Mode

```python
import logging
logging.basicConfig(level=logging.DEBUG)

# Enable detailed simulation logging
manager.logger.setLevel(logging.DEBUG)
```

### Performance Profiling

```python
import cProfile
pr = cProfile.Profile()
pr.enable()

# Run simulation
# ... simulation code ...

pr.disable()
pr.print_stats(sort='time')
```

---

## Contributing

### Adding New Components

1. **Follow the factory pattern** for new components
2. **Inherit from base classes** for consistency
3. **Add comprehensive tests** with all three tiers
4. **Update documentation** and examples
5. **Follow pre-commit hooks** for code quality

### Code Standards

- **Type hints** for all function parameters/returns
- **Docstrings** for all classes and methods
- **Logging** for debugging and monitoring
- **Error handling** with appropriate exceptions
- **Configuration-driven** behavior

### Testing Requirements

- **Unit tests** for all new components
- **Integration tests** with existing components
- **Performance tests** for resource usage
- **Three-tier validation** (PERFECT/REAL_LIFE/EXTREME)

---

## Migration from Scattered Components

### Before (Scattered)
```
autonomy/simulation/     # Gazebo worlds
tests/                   # Consolidated test framework
tests/fixtures/         # Mock sensors
bridges/                # Network simulation
```

### After (Centralized)
```
simulation/             # Single unified framework
├── core/              # Central engine
├── sensors/           # All sensor simulation
├── network/           # All network simulation
├── rover/             # All rover simulation
├── environments/      # All environment simulation
└── tools/             # Unified utilities
```

### Migration Benefits

- **Single API** for all simulation needs
- **Consistent interfaces** across components
- **Integrated testing** with comprehensive coverage
- **Better documentation** and examples
- **Easier maintenance** and extension

---

## Support

### Documentation

- **API Reference**: Inline docstrings and type hints
- **Examples**: `simulation/examples/` directory
- **Configuration**: `simulation/config/` templates
- **Troubleshooting**: This README and issue templates

### Getting Help

1. Check existing examples in `simulation/examples/`
2. Review test files in `tests/` directory
3. Check configuration templates in `simulation/config/`
4. File an issue with detailed reproduction steps

### Performance Issues

If you encounter performance issues:
1. Reduce update rates and recording frequency
2. Use perfect environment for debugging
3. Profile with `cProfile` to identify bottlenecks
4. Consider hardware acceleration for compute-intensive simulations

---

## Future Enhancements

- **Hardware-in-the-Loop**: Real sensor integration
- **Multi-Rover Simulation**: Swarm coordination
- **Advanced Physics**: Gazebo integration
- **Machine Learning**: RL training environments
- **Visualization**: Real-time simulation dashboard
- **Cloud Simulation**: Distributed computing support

---

## Summary

The **Centralized Simulation Framework** provides:

- ✅ **Unified API** for all simulation components
- ✅ **Three-tier testing** (PERFECT/REAL_LIFE/EXTREME)
- ✅ **Comprehensive validation** before hardware testing
- ✅ **Production-ready** with proper error handling
- ✅ **Extensible architecture** for future enhancements
- ✅ **Performance optimized** for real-time simulation

**Use this framework to validate your algorithms and identify issues before touching hardware!**

---

## Enhanced Features v2.0

### Structured Logging System
```python
from simulation import setup_simulation_logging, get_simulation_logger

# Setup structured logging
setup_simulation_logging(
    log_level="INFO",
    log_file="simulation.log",
    enable_structured=True,
    enable_json=True
)

# Get contextual logger
logger = get_simulation_logger(__name__, "sensor_simulation")
logger.info("Sensor data updated", sensor_id="gps_1", accuracy=2.5)
```

### Real-time Monitoring Dashboard
```python
from simulation import SimulationMonitor

# Create monitor
monitor = SimulationMonitor(simulation_manager)
monitor.start_monitoring(interval=1.0)

# Get dashboard data
dashboard = monitor.get_dashboard_data()
print(f"Memory usage: {dashboard['current_metrics']['system']['memory_usage_percent']:.1f}%")

# Export monitoring data
monitor.export_metrics("monitoring_data.json")
```

### Comprehensive Tracing
```python
from simulation import SimulationTracer

# Create tracer
tracer = SimulationTracer()
tracer.enable()

# Trace operations
with tracer.trace_context("sensor_update", sensor_type="gps"):
    # Your code here
    pass

# Get performance report
report = tracer.get_performance_report()
print(f"Slowest operation: {report['slowest_operations'][0]}")
```

### RL Training Support
```python
from simulation import RLDataRecorder

# Create RL recorder
rl_recorder = RLDataRecorder()

# Start episode
episode_id = rl_recorder.start_episode()

# Record RL steps
rl_recorder.record_rl_step(
    state=state,
    action=action,
    next_state=next_state,
    done=False,
    info={"goal_distance": 10.5}
)

# End episode
rl_recorder.end_episode("goal_reached", success=True)

# Save RL dataset
rl_recorder.save_rl_dataset("rl_training_data.json")
```

### Configuration Examples

#### Basic Monitoring Setup
```python
config = {
    "logging": {"enabled": True, "structured": True},
    "monitoring": {"enabled": True, "interval": 1.0},
    "tracing": {"enabled": True, "auto_profile_threshold": 0.1},
    # ... rest of simulation config
}
```

#### RL Training Configuration
```python
rl_config = {
    "recording": {
        "format": "rl_dataset",
        "include_rewards": True,
        "record_interval": 0.1
    },
    "rl": {
        "reward_function": "navigation_efficiency",
        "episode_timeout": 300,
        "max_episodes": 100
    },
    # ... standard simulation config
}
```

### Example Scripts

Run the enhanced examples:
```bash
# RL training with monitoring
python3 simulation/examples/rl_training_example.py

# Real-time monitoring dashboard
python3 simulation/examples/monitoring_dashboard_demo.py
```

---

**Last Updated**: 2025-12-12
**Version**: 2.0.0
**Status**: ✅ Enhanced with Logging, Monitoring & RL Support
