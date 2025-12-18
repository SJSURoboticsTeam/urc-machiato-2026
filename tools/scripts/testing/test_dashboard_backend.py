#!/usr/bin/env python3
"""
Test Dashboard Backend Service

Provides real-time test execution data, communication metrics,
and system state monitoring for the Integrated Testing Dashboard.

Features:
- WebSocket server for real-time data streaming
- ROS2 bridge integration for topic monitoring
- CAN bus simulator integration
- Test execution engine
- Performance metrics collection
"""

import asyncio
import json
import logging
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from enum import Enum
from typing import Any, Dict, List, Optional

import websockets

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import BatteryState, Imu, NavSatFix
    from std_msgs.msg import Float32, String

    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    logging.warning("ROS2 not available - running in standalone mode")

# Import simulation components
try:
    # Add current directory to path for simulation imports
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).parent.parent))

    from simulation.can.can_bus_mock_simulator import (
        CANBusMockSimulator,
        CANMessagePriority,
        CANMessageType,
    )
    from simulation.rover.rover_factory import RoverFactory
    from simulation.sensors.sensor_factory import SensorFactory

    HAS_SIMULATION = True
    logging.info("Simulation components loaded successfully")
except ImportError as e:
    HAS_SIMULATION = False
    logging.warning(
        f"Simulation components not available - using basic simulation: {e}"
    )


# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class ROS2BridgeNode(Node if HAS_ROS2 else object):
    """ROS2 Node for real-time data bridging"""

    def __init__(self, backend=None):
        self.backend = backend  # Store backend reference first

        if HAS_ROS2:
            super().__init__("test_dashboard_bridge")

            # Publishers for simulated sensor data
            self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
            self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
            self.battery_pub = self.create_publisher(BatteryState, "/battery/state", 10)
            self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

            # Subscribers for monitoring
            self.state_sub = self.create_subscription(
                String, "/state_machine/current_state", self.state_callback, 10
            )
            self.cmd_vel_sub = self.create_subscription(
                Twist, "/cmd_vel", self.cmd_vel_callback, 10
            )

            # Data storage
            self.current_state = "BOOT"
            self.last_cmd_vel = Twist()

            # Timer for periodic publishing
            self.create_timer(0.1, self.publish_sensor_data)  # 10Hz publishing

            self.get_logger().info("ROS2 Bridge Node initialized")
            logger.info("ROS2 Bridge Node initialized with backend reference")
        else:
            logger.warning("ROS2 not available - bridge node disabled")

    def state_callback(self, msg):
        """Handle state machine updates"""
        self.current_state = msg.data
        logger.info(f"State updated: {self.current_state}")

    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        self.last_cmd_vel = msg
        logger.info(
            f"Command velocity: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}"
        )

    def publish_sensor_data(self):
        """Publish simulated sensor data using simple time-based simulation"""
        if not HAS_ROS2:
            return

        current_time = self.get_clock().now().to_msg()
        sim_time = time.time()

        # Generate simple simulation data directly in ROS2 node
        # This is independent of the backend simulation
        imu_data = {
            "linear_acceleration": {
                "x": 0.1 + (sim_time % 2) * 0.1,
                "y": -0.05 + (sim_time % 2) * 0.05,
                "z": 9.81,
            },
            "angular_velocity": {
                "x": (sim_time % 3) * 0.05,
                "y": (sim_time % 3) * 0.03,
                "z": (sim_time % 3) * 0.02,
            },
        }

        gps_data = {
            "latitude": 38.406 + (sim_time % 100) * 0.001,
            "longitude": -110.792 + (sim_time % 100) * 0.001,
            "altitude": 1500.0,
        }

        battery_data = {
            "voltage": 24.0 + (sim_time % 10) * 0.1 - 0.5,
            "current": 2.0 + (sim_time % 5) * 0.2 - 0.5,
            "percentage": 85.0 + (sim_time % 10) * 2 - 5,
        }

        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = "imu"
        imu_msg.linear_acceleration.x = imu_data["linear_acceleration"]["x"]
        imu_msg.linear_acceleration.y = imu_data["linear_acceleration"]["y"]
        imu_msg.linear_acceleration.z = imu_data["linear_acceleration"]["z"]
        imu_msg.angular_velocity.x = imu_data["angular_velocity"]["x"]
        imu_msg.angular_velocity.y = imu_data["angular_velocity"]["y"]
        imu_msg.angular_velocity.z = imu_data["angular_velocity"]["z"]
        self.imu_pub.publish(imu_msg)

        # Publish GPS data
        gps_msg = NavSatFix()
        gps_msg.header.stamp = current_time
        gps_msg.header.frame_id = "gps"
        gps_msg.latitude = gps_data["latitude"]
        gps_msg.longitude = gps_data["longitude"]
        gps_msg.altitude = gps_data["altitude"]
        self.gps_pub.publish(gps_msg)

        # Publish battery data
        battery_msg = BatteryState()
        battery_msg.header.stamp = current_time
        battery_msg.voltage = battery_data["voltage"]
        battery_msg.current = battery_data["current"]
        battery_msg.percentage = battery_data["percentage"]
        self.battery_pub.publish(battery_msg)

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.x = (sim_time % 100) * 0.1
        odom_msg.pose.pose.position.y = (sim_time % 50) * 0.05
        odom_msg.twist.twist.linear.x = 0.5 + (sim_time % 5) * 0.1
        odom_msg.twist.twist = self.last_cmd_vel
        self.odom_pub.publish(odom_msg)

        # Debug print every 5 seconds
        if int(sim_time) % 5 == 0 and int(sim_time * 10) % 10 == 0:
            print(
                f"ROS2 PUBLISHING: IMU x={imu_data['linear_acceleration']['x']:.3f}, GPS lat={gps_data['latitude']:.6f}"
            )


class TestStatus(Enum):
    """Test execution status"""

    PENDING = "pending"
    RUNNING = "running"
    PASSED = "passed"
    FAILED = "failed"
    STOPPED = "stopped"


class TestPriority(Enum):
    """Test priority levels"""

    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"


@dataclass
class TestDefinition:
    """Definition of a test"""

    id: str
    name: str
    category: str
    priority: TestPriority
    duration_estimate: float  # seconds
    description: str


@dataclass
class TestResult:
    """Result of a test execution"""

    test_id: str
    status: TestStatus
    start_time: float
    end_time: Optional[float] = None
    message: str = ""
    metrics: Dict[str, Any] = None


@dataclass
class CommunicationMetrics:
    """Metrics for a communication channel"""

    message_count: int = 0
    message_rate: float = 0.0
    latency: float = 0.0
    errors: int = 0
    last_activity: Optional[float] = None


class TestExecutionEngine:
    """
    Executes tests and collects results.

    This is a simplified implementation - in production, this would
    interface with pytest, ROS2 test infrastructure, etc.
    """

    def __init__(self):
        self.active_tests: Dict[str, TestResult] = {}
        self.test_history: List[TestResult] = []

        # Define available tests
        self.test_definitions = {
            # Communication tests
            "comm-ws-ros2": TestDefinition(
                id="comm-ws-ros2",
                name="WebSocket ↔ ROS2",
                category="communication",
                priority=TestPriority.HIGH,
                duration_estimate=2.0,
                description="Verify message routing between WebSocket and ROS2",
            ),
            "comm-ros2-can": TestDefinition(
                id="comm-ros2-can",
                name="ROS2 ↔ CAN",
                category="communication",
                priority=TestPriority.HIGH,
                duration_estimate=2.0,
                description="Verify message routing between ROS2 and CAN bus",
            ),
            "comm-bidirectional": TestDefinition(
                id="comm-bidirectional",
                name="Bidirectional Flow",
                category="communication",
                priority=TestPriority.HIGH,
                duration_estimate=3.0,
                description="Verify bidirectional message flow across all systems",
            ),
            "comm-latency": TestDefinition(
                id="comm-latency",
                name="Latency Check",
                category="communication",
                priority=TestPriority.HIGH,
                duration_estimate=5.0,
                description="Measure end-to-end latency across communication channels",
            ),
            # State machine tests
            "state-transitions": TestDefinition(
                id="state-transitions",
                name="State Transitions",
                category="state_machine",
                priority=TestPriority.HIGH,
                duration_estimate=4.0,
                description="Test all valid state transitions",
            ),
            "state-recovery": TestDefinition(
                id="state-recovery",
                name="Error Recovery",
                category="state_machine",
                priority=TestPriority.HIGH,
                duration_estimate=3.0,
                description="Test error state recovery mechanisms",
            ),
            "state-timing": TestDefinition(
                id="state-timing",
                name="Transition Timing",
                category="state_machine",
                priority=TestPriority.MEDIUM,
                duration_estimate=6.0,
                description="Measure state transition timing and delays",
            ),
            # Mock data tests
            "mock-can-stream": TestDefinition(
                id="mock-can-stream",
                name="CAN Bus Stream",
                category="mock_data",
                priority=TestPriority.MEDIUM,
                duration_estimate=10.0,
                description="Continuously stream mock CAN bus data",
            ),
            "mock-sensor-data": TestDefinition(
                id="mock-sensor-data",
                name="Sensor Data",
                category="mock_data",
                priority=TestPriority.MEDIUM,
                duration_estimate=5.0,
                description="Generate and validate mock sensor data",
            ),
            "mock-fault-injection": TestDefinition(
                id="mock-fault-injection",
                name="Fault Injection",
                category="mock_data",
                priority=TestPriority.LOW,
                duration_estimate=8.0,
                description="Inject faults into mock data streams",
            ),
            # Simulation tests
            "sim-drive": TestDefinition(
                id="sim-drive",
                name="Drive System",
                category="simulation",
                priority=TestPriority.LOW,
                duration_estimate=15.0,
                description="Run drive system simulation",
            ),
            "sim-arm": TestDefinition(
                id="sim-arm",
                name="Arm System",
                category="simulation",
                priority=TestPriority.LOW,
                duration_estimate=12.0,
                description="Run robotic arm simulation",
            ),
            "sim-science": TestDefinition(
                id="sim-science",
                name="Science Payload",
                category="simulation",
                priority=TestPriority.LOW,
                duration_estimate=10.0,
                description="Run science payload simulation",
            ),
        }

    async def start_test(self, test_id: str) -> TestResult:
        """Start a test execution"""
        if test_id not in self.test_definitions:
            raise ValueError(f"Unknown test: {test_id}")

        if test_id in self.active_tests:
            return self.active_tests[test_id]

        result = TestResult(
            test_id=test_id,
            status=TestStatus.RUNNING,
            start_time=time.time(),
            message="Test started",
        )

        self.active_tests[test_id] = result

        # Start test execution in background
        asyncio.create_task(self._execute_test(test_id))

        return result

    async def _execute_test(self, test_id: str):
        """Execute a test (simulated)"""
        test_def = self.test_definitions[test_id]
        result = self.active_tests[test_id]

        try:
            # Simulate test execution
            await asyncio.sleep(test_def.duration_estimate)

            # 90% success rate for demo purposes
            success = True  # In production, this would be actual test result

            result.status = TestStatus.PASSED if success else TestStatus.FAILED
            result.end_time = time.time()
            result.message = (
                "Test completed successfully" if success else "Test failed - check logs"
            )
            result.metrics = {
                "duration": result.end_time - result.start_time,
                "assertions_passed": 15 if success else 12,
                "assertions_total": 15,
            }

            logger.info(f"Test {test_id} completed: {result.status.value}")

        except asyncio.CancelledError:
            result.status = TestStatus.STOPPED
            result.end_time = time.time()
            result.message = "Test stopped by user"
            logger.info(f"Test {test_id} stopped")

        except Exception as e:
            result.status = TestStatus.FAILED
            result.end_time = time.time()
            result.message = f"Test error: {str(e)}"
            logger.error(f"Test {test_id} error: {e}", exc_info=True)

        finally:
            # Move to history
            self.test_history.append(result)
            if test_id in self.active_tests:
                del self.active_tests[test_id]

    def stop_test(self, test_id: str):
        """Stop a running test"""
        if test_id in self.active_tests:
            # Cancel the test task (would need task tracking in production)
            result = self.active_tests[test_id]
            result.status = TestStatus.STOPPED
            result.end_time = time.time()
            result.message = "Test stopped by user"

    def get_test_status(self, test_id: str) -> Optional[TestResult]:
        """Get current status of a test"""
        return self.active_tests.get(test_id)

    def get_all_test_statuses(self) -> Dict[str, TestResult]:
        """Get status of all active tests"""
        return self.active_tests.copy()


class TestDashboardBackend:
    """
    Backend service for the Integrated Testing Dashboard.

    Provides WebSocket server for real-time data streaming and
    integrates with ROS2 for actual system monitoring.
    """

    def __init__(self, host: str = "localhost", port: int = 8766):
        self.host = host
        self.port = port
        self.clients: set = set()
        self.test_engine = TestExecutionEngine()

        # Initialize simulation components
        self.simulation_data = {}
        self.can_simulator = None
        self.imu_sensor = None
        self.gps_sensor = None
        self.rover = None

        if HAS_SIMULATION:
            self._init_simulation()
        else:
            logger.warning(
                "Using basic simulation - install simulation package for full features"
            )

        # Initialize ROS2 bridge
        self.ros2_node = None
        self.ros2_thread = None
        if HAS_ROS2:
            try:
                logger.info("Creating ROS2 Bridge Node...")
                self.ros2_node = ROS2BridgeNode(self)
                logger.info("ROS2 Bridge Node created, starting thread...")
                self._init_ros2()  # Start ROS2 spinning in separate thread
                logger.info("ROS2 Bridge Node initialized and started successfully")
            except Exception as e:
                logger.error(f"Failed to initialize ROS2 bridge: {e}")
                import traceback

                logger.error(f"Traceback: {traceback.format_exc()}")
                self.ros2_node = None

        # Communication metrics
        self.metrics = {
            "websocket": CommunicationMetrics(),
            "ros2": CommunicationMetrics(),
            "can": CommunicationMetrics(last_activity=time.time()),
        }

        # State history
        self.state_history: List[Dict[str, Any]] = [
            {"state": "BOOT", "timestamp": time.time() - 60, "duration": 60000}
        ]

        # Active topics
        self.active_topics = [
            {
                "name": "/state_machine/current_state",
                "type": "std_msgs/String",
                "rate": 1.0,
                "active": True,
                "source": "ros2",
            },
            {
                "name": "/gps/fix",
                "type": "sensor_msgs/NavSatFix",
                "rate": 10.0,
                "active": True,
                "source": "can",
            },
            {
                "name": "/imu/data",
                "type": "sensor_msgs/Imu",
                "rate": 100.0,
                "active": True,
                "source": "can",
            },
            {
                "name": "/cmd_vel",
                "type": "geometry_msgs/Twist",
                "rate": 20.0,
                "active": True,
                "source": "ros2",
            },
        ]

        # Mock CAN data
        self.can_data = {
            "imu": {"x": 0.0, "y": 0.0, "z": 9.81},
            "gps": {"lat": 38.406, "lon": -110.792},
            "battery": {"voltage": 24.0, "current": 5.0},
            "motors": {"left": 0.0, "right": 0.0},
        }

    def _init_simulation(self):
        """Initialize simulation components"""
        try:
            # Initialize CAN bus simulator
            self.can_simulator = CANBusMockSimulator()

            # Initialize sensor factory and create sensors
            sensor_factory = SensorFactory()
            self.imu_sensor = sensor_factory.create(
                "imu",
                {
                    "name": "imu_sensor",
                    "type": "imu",
                    "gyro_bias_drift": 0.01,
                    "accel_bias_drift": 0.001,
                    "gyro_noise_std": 0.01,
                    "accel_noise_std": 0.1,
                },
            )

            self.gps_sensor = sensor_factory.create(
                "gps",
                {
                    "name": "gps_sensor",
                    "type": "gps",
                    "base_position": [38.406, -110.792, 1500.0],
                    "position_noise_std": 2.5,
                    "min_satellites": 4,
                    "max_satellites": 12,
                },
            )

            # Initialize rover
            rover_factory = RoverFactory()
            self.rover = rover_factory.create({"model": "urc_rover"})

            logger.info("Simulation components initialized")

        except Exception as e:
            logger.error(f"Failed to initialize simulation: {e}")
            HAS_SIMULATION = False

    def _init_ros2(self):
        """Initialize ROS2 node in separate thread"""
        if not HAS_ROS2:
            return

        def ros2_spin():
            try:
                rclpy.init()
                self.ros2_node = ROS2BridgeNode(backend=self)
                logger.info("ROS2 bridge node created successfully")

                rclpy.spin(self.ros2_node)
            except KeyboardInterrupt:
                logger.info("ROS2 spin interrupted")
            except Exception as e:
                logger.error(f"ROS2 spin error: {e}")
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
                logger.info("ROS2 shutdown complete")

        import threading

        self.ros2_thread = threading.Thread(target=ros2_spin, daemon=True)
        self.ros2_thread.start()
        logger.info("ROS2 bridge node started")

    async def handle_client(self, websocket):
        """Handle WebSocket client connection"""
        self.clients.add(websocket)
        logger.info(f"Client connected: {websocket.remote_address}")

        try:
            # Send initial state
            await self.send_full_state(websocket)

            # Keep connection open for broadcasting - handle incoming messages if any
            async for message in websocket:
                await self.handle_message(websocket, message)

        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Client disconnected: {websocket.remote_address}")

        except Exception as e:
            logger.error(f"Error handling client {websocket.remote_address}: {e}")

        finally:
            self.clients.remove(websocket)

    async def handle_message(self, websocket, message: str):
        """Handle incoming WebSocket message"""
        try:
            logger.info(f"Received message: {message[:100]}...")
            data = json.loads(message)
            msg_type = data.get("type")
            logger.info(f"Parsed message type: {msg_type}")

            if msg_type == "start_test":
                test_id = data.get("test_id")
                result = await self.test_engine.start_test(test_id)
                await self.broadcast_test_update(result)

            elif msg_type == "stop_test":
                test_id = data.get("test_id")
                self.test_engine.stop_test(test_id)
                await self.broadcast_test_update(
                    self.test_engine.get_test_status(test_id)
                )

            elif msg_type == "get_state":
                await self.send_full_state(websocket)

            else:
                logger.warning(f"Unknown message type: {msg_type}")

        except json.JSONDecodeError as e:
            logger.error(f"Invalid JSON message: {message} - Error: {e}")

        except Exception as e:
            logger.error(f"Error handling message: {e}", exc_info=True)
            # Don't let exceptions propagate to WebSocket layer
            pass

    async def send_full_state(self, websocket):
        """Send complete current state to client"""
        state = {
            "type": "full_state",
            "timestamp": time.time(),
            "metrics": {k: asdict(v) for k, v in self.metrics.items()},
            "state_history": self.state_history[-10:],  # Last 10 states
            "active_topics": self.active_topics,
            "test_statuses": {
                test_id: asdict(result)
                for test_id, result in self.test_engine.get_all_test_statuses().items()
            },
            "can_data": self.can_data,
        }

        await websocket.send(json.dumps(state))

    async def broadcast_test_update(self, result: TestResult):
        """Broadcast test update to all clients"""
        if not result:
            return

        update = {
            "type": "test_update",
            "timestamp": time.time(),
            "result": asdict(result),
        }

        # Convert enum to string
        if "status" in update["result"]:
            update["result"]["status"] = update["result"]["status"].value

        await self.broadcast(json.dumps(update))

    async def broadcast(self, message: str):
        """Broadcast message to all connected clients"""
        if self.clients:
            await asyncio.gather(
                *[client.send(message) for client in self.clients],
                return_exceptions=True,
            )

    async def update_metrics_loop(self):
        """Periodically update communication metrics and generate simulation data"""
        simulation_time = 0.0

        while True:
            await asyncio.sleep(0.1)  # 10 Hz updates
            simulation_time += 0.1

            # Generate simulation data
            simulation_data = self._generate_simulation_data(simulation_time)

            # Store latest data for broadcasting
            self.simulation_data = simulation_data
            logger.debug(
                f"Updated simulation_data with IMU accel_x: {simulation_data.get('imu', {}).get('linear_acceleration', {}).get('x', 'N/A')}"
            )

            # Update metrics
            self._update_communication_metrics(simulation_data)

            # Publish to ROS2 if available
            if HAS_ROS2 and self.ros2_node:
                self._publish_to_ros2(simulation_data)

            # Broadcast to WebSocket clients
            if self.clients:
                # Simplify metrics to avoid serialization issues
                simple_metrics = {}
                for k, v in self.metrics.items():
                    simple_metrics[k] = {
                        "message_count": getattr(v, "message_count", 0),
                        "message_rate": getattr(v, "message_rate", 0.0),
                        "latency": getattr(v, "latency", 0.0),
                        "last_activity": getattr(v, "last_activity", 0.0),
                        "connection_status": getattr(v, "connection_status", False),
                    }

                update = {
                    "type": "simulation_update",
                    "timestamp": time.time(),
                    "simulation_data": simulation_data,
                    "metrics": simple_metrics,
                }
                await self.broadcast(json.dumps(update))

    def _generate_simulation_data(self, simulation_time):
        """Generate comprehensive simulation data"""
        data = {
            "timestamp": time.time(),
            "simulation_time": simulation_time,
        }

        if HAS_SIMULATION and self.imu_sensor and self.gps_sensor:
            # Use real simulation sensors
            imu_raw = self.imu_sensor.step(0.1, {})  # dt=0.1s, empty environment
            gps_raw = self.gps_sensor.step(0.1, {})  # dt=0.1s, empty environment

            # Transform to ROS2-compatible format
            data["imu"] = {
                "linear_acceleration": {
                    "x": imu_raw.get("accel_x", 0.0),
                    "y": imu_raw.get("accel_y", 0.0),
                    "z": imu_raw.get("accel_z", 9.81),
                },
                "angular_velocity": {
                    "x": imu_raw.get("gyro_x", 0.0),
                    "y": imu_raw.get("gyro_y", 0.0),
                    "z": imu_raw.get("gyro_z", 0.0),
                },
            }
            data["gps"] = {
                "latitude": gps_raw.get("latitude", 38.406),
                "longitude": gps_raw.get("longitude", -110.792),
                "altitude": gps_raw.get("altitude", 1500.0),
            }
            data["battery"] = {
                "voltage": 24.0 + (simulation_time % 10) * 0.1 - 0.5,
                "current": 2.0 + (simulation_time % 5) * 0.2 - 0.5,
                "percentage": 85.0 + (simulation_time % 10) * 2 - 5,
            }
            data["rover_state"] = (
                self.rover.get_state()
                if hasattr(self.rover, "get_state")
                else {"speed": 0.0}
            )
        else:
            # Fallback to basic simulation
            data["imu"] = {
                "linear_acceleration": {
                    "x": 0.1 + (simulation_time % 2) * 0.05,
                    "y": -0.05 + (simulation_time % 2) * 0.03,
                    "z": 9.81,
                },
                "angular_velocity": {
                    "x": (simulation_time % 3) * 0.1,
                    "y": (simulation_time % 3) * 0.1,
                    "z": (simulation_time % 3) * 0.1,
                },
            }
            data["gps"] = {
                "latitude": 38.406 + (simulation_time % 100) * 0.001,
                "longitude": -110.792 + (simulation_time % 100) * 0.001,
                "altitude": 1500.0,
            }
            data["battery"] = {
                "voltage": 24.0 + (simulation_time % 10) * 0.1 - 0.5,
                "current": 2.0 + (simulation_time % 5) * 0.2 - 0.5,
                "percentage": 85.0 + (simulation_time % 10) * 2 - 5,
            }
            data["rover_state"] = {"speed": 0.5 + (simulation_time % 5) * 0.1}

        # Generate CAN messages
        data["can_messages"] = self._generate_can_messages(simulation_time)

        return data

    def _generate_can_messages(self, simulation_time):
        """Generate CAN bus messages using the CAN simulator"""
        messages = []

        if HAS_SIMULATION and self.can_simulator:
            # Get data from CAN simulator sensors
            try:
                # IMU data from CAN sensors
                imu_data = self.can_simulator.sensors["imu_0"].get_reading()
                messages.append(
                    {
                        "id": 0x100,
                        "type": "imu_data",
                        "priority": "status_update",
                        "data": imu_data,
                        "timestamp": time.time(),
                    }
                )

                # GPS data from CAN sensors
                gps_data = self.can_simulator.sensors["gps_0"].get_reading()
                messages.append(
                    {
                        "id": 0x101,
                        "type": "gps_data",
                        "priority": "status_update",
                        "data": gps_data,
                        "timestamp": time.time(),
                    }
                )

                # Motor status from CAN controllers
                for name, controller in self.can_simulator.motor_controllers.items():
                    if "front_left" in name:  # Just get one motor for demo
                        motor_status = controller.get_status()
                        messages.append(
                            {
                                "id": 0x200,
                                "type": "motor_status",
                                "priority": "status_update",
                                "data": motor_status,
                                "timestamp": time.time(),
                            }
                        )
                        break

                # Battery/system status (simulated)
                messages.append(
                    {
                        "id": 0x300,
                        "type": "system_status",
                        "priority": "status_update",
                        "data": {
                            "voltage": 24.0 + (simulation_time % 2),
                            "current": 2.0,
                            "percentage": 85.0 + (simulation_time % 10),
                            "temperature": 25.0,
                        },
                        "timestamp": time.time(),
                    }
                )

            except Exception as e:
                logger.warning(f"Error getting CAN data: {e}, using fallback")
                # Fallback to basic messages
                messages = self._generate_fallback_can_messages(simulation_time)
        else:
            # Fallback CAN messages
            messages = self._generate_fallback_can_messages(simulation_time)

        return messages

    def _generate_fallback_can_messages(self, simulation_time):
        """Generate basic CAN messages when simulation is not available"""
        messages = []

        # IMU message
        messages.append(
            {
                "id": 0x100,
                "type": "imu_data",
                "priority": "status_update",
                "data": {
                    "accel_x": 0.1 + (simulation_time % 2) * 0.05,
                    "accel_y": -0.05 + (simulation_time % 2) * 0.03,
                    "accel_z": 9.81,
                    "gyro_x": (simulation_time % 3) * 0.1,
                    "gyro_y": (simulation_time % 3) * 0.1,
                    "gyro_z": (simulation_time % 3) * 0.1,
                },
                "timestamp": time.time(),
            }
        )

        # GPS message
        messages.append(
            {
                "id": 0x101,
                "type": "gps_data",
                "priority": "status_update",
                "data": {
                    "latitude": 38.406 + (simulation_time % 100) * 0.001,
                    "longitude": -110.792 + (simulation_time % 100) * 0.001,
                    "altitude": 1500.0,
                    "satellites_visible": 8,
                },
                "timestamp": time.time(),
            }
        )

        # Motor message
        messages.append(
            {
                "id": 0x200,
                "type": "motor_status",
                "priority": "status_update",
                "data": {
                    "motor_id": 0,
                    "position": (simulation_time * 10) % 360,
                    "velocity": 5.0 + (simulation_time % 5),
                    "current": 2.0,
                    "temperature": 30.0,
                },
                "timestamp": time.time(),
            }
        )

        # Battery message
        messages.append(
            {
                "id": 0x300,
                "type": "system_status",
                "priority": "status_update",
                "data": {
                    "voltage": 24.0 + (simulation_time % 2),
                    "current": 2.0,
                    "percentage": 85.0 + (simulation_time % 10),
                    "temperature": 25.0,
                },
                "timestamp": time.time(),
            }
        )

        return messages

    def _update_communication_metrics(self, simulation_data):
        """Update communication metrics based on simulation data"""

        # WebSocket metrics
        self.metrics["websocket"].message_count += len(self.clients)
        self.metrics["websocket"].message_rate = float(len(self.clients))
        self.metrics["websocket"].latency = 5.0 + (time.time() % 10)
        self.metrics["websocket"].last_activity = time.time()

        # ROS2 metrics
        if HAS_ROS2 and self.ros2_node:
            self.metrics["ros2"].message_count += 4  # IMU, GPS, battery, odom
            self.metrics["ros2"].message_rate = 40.0  # 10 Hz * 4 topics
            self.metrics["ros2"].latency = 2.0 + (time.time() % 5)
            self.metrics["ros2"].last_activity = time.time()
            self.metrics["ros2"].connection_status = True
        else:
            self.metrics["ros2"].connection_status = False

        # CAN metrics
        can_message_count = len(simulation_data.get("can_messages", []))
        self.metrics["can"].message_count += can_message_count
        self.metrics["can"].message_rate = can_message_count * 10  # 10 Hz
        self.metrics["can"].latency = 1.0 + (time.time() % 2)
        self.metrics["can"].last_activity = time.time()

    def _publish_to_ros2(self, simulation_data):
        """Publish simulation data to ROS2 topics"""
        if not self.ros2_node:
            return

        current_time = self.ros2_node.get_clock().now().to_msg()

        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = "imu"

        imu_data = simulation_data["imu"]
        imu_msg.linear_acceleration.x = imu_data.get("accel_x", 0.0)
        imu_msg.linear_acceleration.y = imu_data.get("accel_y", 0.0)
        imu_msg.linear_acceleration.z = imu_data.get("accel_z", 9.81)
        imu_msg.angular_velocity.x = imu_data.get("gyro_x", 0.0)
        imu_msg.angular_velocity.y = imu_data.get("gyro_y", 0.0)
        imu_msg.angular_velocity.z = imu_data.get("gyro_z", 0.0)

        self.ros2_node.imu_pub.publish(imu_msg)

        # Publish GPS data
        gps_msg = NavSatFix()
        gps_msg.header.stamp = current_time
        gps_msg.header.frame_id = "gps"

        gps_data = simulation_data["gps"]
        gps_msg.latitude = gps_data.get("latitude", 38.406)
        gps_msg.longitude = gps_data.get("longitude", -110.792)
        gps_msg.altitude = gps_data.get("altitude", 1500.0)

        self.ros2_node.gps_pub.publish(gps_msg)

        # Publish battery data (from CAN messages)
        for can_msg in simulation_data.get("can_messages", []):
            if can_msg.get("type") == "system_status":
                battery_msg = BatteryState()
                battery_msg.header.stamp = current_time

                battery_data = can_msg["data"]
                battery_msg.voltage = battery_data.get("voltage", 24.0)
                battery_msg.current = battery_data.get("current", 2.0)
                battery_msg.percentage = battery_data.get("percentage", 85.0) / 100.0

                self.ros2_node.battery_pub.publish(battery_msg)
                break

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.x = (simulation_data["simulation_time"]) * 0.1
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.orientation.w = 1.0

        self.ros2_node.odom_pub.publish(odom_msg)

    async def start(self):
        """Start the WebSocket server"""
        logger.info(f"Starting Test Dashboard Backend on {self.host}:{self.port}")

        # Start metrics update loop
        asyncio.create_task(self.update_metrics_loop())

        # Start WebSocket server
        async with websockets.serve(self.handle_client, self.host, self.port):
            logger.info(f"Test Dashboard Backend ready at ws://{self.host}:{self.port}")
            await asyncio.Future()  # Run forever


async def main():
    """Main entry point"""
    # Simple backend for testing dashboard
    backend = TestDashboardBackend(host="0.0.0.0", port=8766)
    await backend.start()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Test Dashboard Backend stopped by user")
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
