#!/usr/bin/env python3
"""
Full Stack Communication Simulator

Provides complete end-to-end communication simulation for testing
all paths without hardware:
1. Frontend → WebSocket → ROS2 → SLCAN → Firmware
2. Firmware → SLCAN → ROS2 → WebSocket → Frontend
3. ROS2 Autonomy → SLCAN → Firmware
4. Emergency stop propagation through all layers

Author: URC 2026 Simulation Team
"""

import asyncio
import logging
import time
from typing import Dict, Any, List, Optional, Callable
from dataclasses import dataclass, field
from enum import Enum

from simulation.network.websocket_server_simulator import WebSocketServerSimulator
from simulation.can.slcan_protocol_simulator import SLCANProtocolSimulator, VelocityCommand
from simulation.firmware.stm32_firmware_simulator import STM32FirmwareSimulator
from simulation.ros2.ros2_message_adapter import ROS2MessageAdapter, ROS2TopicBridge, STANDARD_TOPICS

logger = logging.getLogger(__name__)


class CommunicationPath(Enum):
    """Communication paths through the system."""
    TELEOPERATION = "teleoperation"  # Frontend → WebSocket → ROS2 → CAN → Firmware
    AUTONOMY = "autonomy"            # ROS2 Nav → CAN → Firmware
    FEEDBACK = "feedback"            # Firmware → CAN → ROS2 → WebSocket → Frontend
    EMERGENCY = "emergency"          # E-stop propagation through all layers


class ScenarioType(Enum):
    """Test scenario types."""
    BASIC_VELOCITY = "basic_velocity"
    EMERGENCY_STOP = "emergency_stop"
    NETWORK_FAILURE = "network_failure"
    FIRMWARE_FAULT = "firmware_fault"
    HIGH_LOAD = "high_load"
    RECOVERY = "recovery"


@dataclass
class ScenarioResult:
    """Result of a simulation scenario."""
    scenario_type: ScenarioType
    success: bool
    duration_s: float
    messages_sent: int
    messages_received: int
    errors: List[str] = field(default_factory=list)
    metrics: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


class FullStackSimulator:
    """
    Complete communication stack simulator.
    
    Simulates the entire rover communication system including:
    - Frontend/operator interface
    - WebSocket/Socket.IO server
    - ROS2 autonomy system
    - Protocol adapters
    - SLCAN/CAN communication
    - STM32 firmware
    """
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """Initialize full stack simulator.
        
        Args:
            config: Configuration dictionary for all components
        """
        self.logger = logging.getLogger(f"{__name__}.FullStackSimulator")
        
        # Configuration
        config = config or {}
        
        # Create simulation components
        self.websocket_sim = WebSocketServerSimulator(config.get('websocket', {}))
        self.slcan_sim = SLCANProtocolSimulator(config.get('slcan', {}))
        self.firmware_sim = STM32FirmwareSimulator(config.get('firmware', {}))
        
        # ROS2 integration
        self.ros2_adapter = ROS2MessageAdapter()
        self.ros2_bridge = ROS2TopicBridge()
        
        # ROS2 state (simulated)
        self.ros2_state = {
            'cmd_vel_teleop': None,
            'cmd_vel_autonomy': None,
            'velocity_feedback': None,
            'emergency_stop': False,
            'last_update': time.time()
        }
        
        # Scenario results
        self.scenario_results: List[ScenarioResult] = []
        
        # Event queue for message tracking
        self.event_queue = []
        self.max_queue_size = config.get('max_queue_size', 1000)
        
        # Statistics
        self.stats = {
            'scenarios_run': 0,
            'scenarios_passed': 0,
            'scenarios_failed': 0,
            'total_messages': 0,
            'total_errors': 0
        }
        
        # Setup pipeline connections
        self._setup_pipeline()
        
        self.logger.info("Full stack simulator initialized")
    
    def _setup_pipeline(self):
        """Setup communication pipeline between components."""
        
        # Frontend → WebSocket → ROS2
        async def handle_drive_command(data):
            """Handle drive command from frontend."""
            try:
                # Convert to ROS2 Twist message
                twist = self.ros2_adapter.dict_to_twist(data)
                
                # Publish to ROS2 topic
                self.ros2_bridge.publish(STANDARD_TOPICS['cmd_vel_teleop'], twist)
                
                # Update ROS2 state
                self.ros2_state['cmd_vel_teleop'] = twist
                self.ros2_state['last_update'] = time.time()
                
                # Forward to SLCAN/Firmware
                self._send_to_firmware(twist.linear_x, twist.angular_z)
                
                self._log_event('drive_command', data, CommunicationPath.TELEOPERATION)
            except Exception as e:
                self.logger.error(f"Drive command error: {e}")
        
        async def handle_emergency_stop(data):
            """Handle emergency stop from frontend."""
            try:
                self.ros2_state['emergency_stop'] = True
                self.firmware_sim.emergency_stop()
                self._log_event('emergency_stop', data, CommunicationPath.EMERGENCY)
            except Exception as e:
                self.logger.error(f"Emergency stop error: {e}")
        
        async def handle_homing(data):
            """Handle homing request from frontend."""
            try:
                self.firmware_sim.start_homing_sequence()
                self._log_event('homing_request', data, CommunicationPath.TELEOPERATION)
            except Exception as e:
                self.logger.error(f"Homing error: {e}")
        
        self.websocket_sim.on('driveCommands', handle_drive_command)
        self.websocket_sim.on('emergencyStop', handle_emergency_stop)
        self.websocket_sim.on('homingRequest', handle_homing)
        
        # Start firmware simulator
        self.firmware_sim.start()
        
        self.logger.info("Communication pipeline setup complete")
    
    def _send_to_firmware(self, linear: float, angular: float):
        """Send velocity command through SLCAN to firmware.
        
        Args:
            linear: Linear velocity (m/s)
            angular: Angular velocity (rad/s)
        """
        try:
            # Encode to SLCAN
            slcan_frame = self.slcan_sim.encode_velocity_command(linear, 0.0, angular)
            
            # Decode and send to firmware
            cmd = self.slcan_sim.decode_velocity_command(slcan_frame)
            if cmd:
                self.firmware_sim.set_chassis_velocities(
                    cmd.linear_x, cmd.linear_y, cmd.angular_z
                )
        except Exception as e:
            self.logger.error(f"Error sending to firmware: {e}")
    
    async def run_scenario(self, scenario_type: ScenarioType, 
                          params: Optional[Dict[str, Any]] = None) -> ScenarioResult:
        """Run a test scenario.
        
        Args:
            scenario_type: Type of scenario to run
            params: Scenario-specific parameters
            
        Returns:
            ScenarioResult with test results
        """
        start_time = time.time()
        params = params or {}
        errors = []
        
        try:
            if scenario_type == ScenarioType.BASIC_VELOCITY:
                success = await self._scenario_basic_velocity(params)
            elif scenario_type == ScenarioType.EMERGENCY_STOP:
                success = await self._scenario_emergency_stop(params)
            elif scenario_type == ScenarioType.NETWORK_FAILURE:
                success = await self._scenario_network_failure(params)
            elif scenario_type == ScenarioType.FIRMWARE_FAULT:
                success = await self._scenario_firmware_fault(params)
            elif scenario_type == ScenarioType.HIGH_LOAD:
                success = await self._scenario_high_load(params)
            elif scenario_type == ScenarioType.RECOVERY:
                success = await self._scenario_recovery(params)
            else:
                errors.append(f"Unknown scenario type: {scenario_type}")
                success = False
        
        except Exception as e:
            errors.append(f"Scenario exception: {e}")
            success = False
        
        duration = time.time() - start_time
        
        # Create result
        result = ScenarioResult(
            scenario_type=scenario_type,
            success=success,
            duration_s=duration,
            messages_sent=self.websocket_sim.stats['messages_sent'],
            messages_received=self.websocket_sim.stats['messages_received'],
            errors=errors,
            metrics=self._collect_metrics()
        )
        
        self.scenario_results.append(result)
        self.stats['scenarios_run'] += 1
        if success:
            self.stats['scenarios_passed'] += 1
        else:
            self.stats['scenarios_failed'] += 1
        
        return result
    
    async def _scenario_basic_velocity(self, params: Dict[str, Any]) -> bool:
        """Test basic velocity command flow.
        
        Args:
            params: Test parameters
            
        Returns:
            bool: True if test passed
        """
        # Connect client
        client_id = await self.websocket_sim.connect()
        
        # Send drive command
        await self.websocket_sim.receive('driveCommands', {
            'linear': 0.5,
            'angular': 0.0
        }, client_id)
        
        # Wait for processing
        await asyncio.sleep(0.1)
        
        # Verify firmware received command
        motor_status = self.firmware_sim.get_motor_status(0)
        if not motor_status:
            return False
        
        # Check if velocity setpoint was updated
        return abs(motor_status['velocity_setpoint']) > 0.0
    
    async def _scenario_emergency_stop(self, params: Dict[str, Any]) -> bool:
        """Test emergency stop propagation.
        
        Args:
            params: Test parameters
            
        Returns:
            bool: True if E-stop propagated within 100ms
        """
        # Connect and send drive command first
        client_id = await self.websocket_sim.connect()
        await self.websocket_sim.receive('driveCommands', {
            'linear': 1.0,
            'angular': 0.5
        }, client_id)
        
        await asyncio.sleep(0.05)
        
        # Trigger emergency stop
        stop_start = time.time()
        await self.websocket_sim.receive('emergencyStop', {}, client_id)
        
        await asyncio.sleep(0.05)
        stop_duration = time.time() - stop_start
        
        # Verify all motors stopped
        all_stopped = True
        for motor_id in range(self.firmware_sim.num_motors):
            status = self.firmware_sim.get_motor_status(motor_id)
            if status and abs(status['velocity_actual']) > 0.01:
                all_stopped = False
        
        # Check propagation time (<100ms requirement)
        return all_stopped and stop_duration < 0.1
    
    async def _scenario_network_failure(self, params: Dict[str, Any]) -> bool:
        """Test behavior during network failure.
        
        Args:
            params: Test parameters
            
        Returns:
            bool: True if system handles network failure correctly
        """
        # Set high packet loss
        self.websocket_sim.set_network_conditions({'packet_loss_rate': 0.5})
        
        client_id = await self.websocket_sim.connect()
        
        # Send multiple commands
        success_count = 0
        for i in range(10):
            result = await self.websocket_sim.receive('driveCommands', {
                'linear': 0.5,
                'angular': 0.0
            }, client_id)
            if result:
                success_count += 1
        
        # Restore normal conditions
        self.websocket_sim.set_network_conditions({'packet_loss_rate': 0.01})
        
        # Expect some messages to fail (but not all)
        return 2 <= success_count <= 8
    
    async def _scenario_firmware_fault(self, params: Dict[str, Any]) -> bool:
        """Test firmware fault handling.
        
        Args:
            params: Test parameters
            
        Returns:
            bool: True if fault handled correctly
        """
        # Inject fault on motor 0
        from simulation.firmware.stm32_firmware_simulator import MotorFaultType
        self.firmware_sim.inject_fault(0, MotorFaultType.OVERCURRENT)
        
        client_id = await self.websocket_sim.connect()
        
        # Try to send command (should be rejected for faulted motor)
        await self.websocket_sim.receive('driveCommands', {
            'linear': 1.0,
            'angular': 0.0
        }, client_id)
        
        await asyncio.sleep(0.05)
        
        # Verify motor 0 is still stopped
        status = self.firmware_sim.get_motor_status(0)
        motor_stopped = status and abs(status['velocity_actual']) < 0.01
        
        # Clear fault
        self.firmware_sim.clear_fault(0)
        
        return motor_stopped
    
    async def _scenario_high_load(self, params: Dict[str, Any]) -> bool:
        """Test system under high message load.
        
        Args:
            params: Test parameters
            
        Returns:
            bool: True if system handles load correctly
        """
        client_id = await self.websocket_sim.connect()
        
        # Send rapid commands (50Hz for 1 second)
        message_count = 50
        success_count = 0
        
        for i in range(message_count):
            result = await self.websocket_sim.receive('driveCommands', {
                'linear': 0.5,
                'angular': 0.1
            }, client_id)
            if result:
                success_count += 1
            await asyncio.sleep(0.02)  # 50Hz
        
        # Expect >90% success under load
        return success_count >= message_count * 0.9
    
    async def _scenario_recovery(self, params: Dict[str, Any]) -> bool:
        """Test recovery after disconnect.
        
        Args:
            params: Test parameters
            
        Returns:
            bool: True if recovery successful
        """
        # Connect and send command
        client_id = await self.websocket_sim.connect()
        await self.websocket_sim.receive('driveCommands', {
            'linear': 0.5,
            'angular': 0.0
        }, client_id)
        
        await asyncio.sleep(0.05)
        
        # Disconnect
        await self.websocket_sim.disconnect(client_id)
        
        # Wait
        await asyncio.sleep(0.1)
        
        # Reconnect
        new_client_id = await self.websocket_sim.connect()
        
        # Send new command
        result = await self.websocket_sim.receive('driveCommands', {
            'linear': 0.3,
            'angular': 0.0
        }, new_client_id)
        
        return result
    
    def _log_event(self, event_type: str, data: Dict[str, Any], 
                  path: CommunicationPath):
        """Log communication event.
        
        Args:
            event_type: Type of event
            data: Event data
            path: Communication path taken
        """
        event = {
            'timestamp': time.time(),
            'event_type': event_type,
            'data': data,
            'path': path.value
        }
        
        self.event_queue.append(event)
        
        # Trim queue if too long
        if len(self.event_queue) > self.max_queue_size:
            self.event_queue = self.event_queue[-self.max_queue_size:]
        
        self.stats['total_messages'] += 1
    
    def _collect_metrics(self) -> Dict[str, Any]:
        """Collect current metrics from all components.
        
        Returns:
            Dict with comprehensive metrics
        """
        return {
            'websocket': self.websocket_sim.get_statistics(),
            'slcan': self.slcan_sim.get_statistics(),
            'firmware': self.firmware_sim.get_system_status(),
            'ros2': self.ros2_state.copy()
        }
    
    async def run_test_suite(self) -> Dict[str, Any]:
        """Run complete test suite of all scenarios.
        
        Returns:
            Dict with overall test results
        """
        self.logger.info("Starting full test suite...")
        
        scenarios = [
            (ScenarioType.BASIC_VELOCITY, {}),
            (ScenarioType.EMERGENCY_STOP, {}),
            (ScenarioType.NETWORK_FAILURE, {}),
            (ScenarioType.FIRMWARE_FAULT, {}),
            (ScenarioType.HIGH_LOAD, {}),
            (ScenarioType.RECOVERY, {})
        ]
        
        results = []
        for scenario_type, params in scenarios:
            self.logger.info(f"Running scenario: {scenario_type.value}")
            result = await self.run_scenario(scenario_type, params)
            results.append(result)
            
            # Brief pause between scenarios
            await asyncio.sleep(0.1)
        
        # Generate summary
        passed = sum(1 for r in results if r.success)
        total = len(results)
        
        summary = {
            'total_scenarios': total,
            'passed': passed,
            'failed': total - passed,
            'pass_rate': passed / total if total > 0 else 0.0,
            'results': results,
            'statistics': self.stats.copy(),
            'timestamp': time.time()
        }
        
        self.logger.info(f"Test suite complete: {passed}/{total} passed ({summary['pass_rate']*100:.1f}%)")
        
        return summary
    
    def get_status(self) -> Dict[str, Any]:
        """Get current simulator status.
        
        Returns:
            Dict with status information
        """
        return {
            'websocket_sim': self.websocket_sim.get_statistics(),
            'slcan_sim': self.slcan_sim.get_statistics(),
            'firmware_sim': self.firmware_sim.get_system_status(),
            'ros2_state': self.ros2_state.copy(),
            'statistics': self.stats.copy(),
            'event_queue_size': len(self.event_queue)
        }
    
    def reset(self):
        """Reset simulator to initial state."""
        # Reset component states
        self.websocket_sim.clear_history()
        self.websocket_sim.stats = {key: 0 for key in self.websocket_sim.stats}
        self.slcan_sim.reset_statistics()
        
        # Stop and restart firmware
        self.firmware_sim.stop()
        self.firmware_sim = STM32FirmwareSimulator()
        self.firmware_sim.start()
        
        # Clear queues and results
        self.event_queue.clear()
        self.scenario_results.clear()
        
        # Reset stats
        self.stats = {key: 0 for key in self.stats}
        
        # Reconnect pipeline
        self._setup_pipeline()
        
        self.logger.info("Full stack simulator reset")
    
    def shutdown(self):
        """Shutdown simulator and cleanup resources."""
        self.firmware_sim.stop()
        self.logger.info("Full stack simulator shutdown")


# Convenience function
def create_full_stack_simulator(profile: str = 'default') -> FullStackSimulator:
    """Create full stack simulator with predefined configuration.
    
    Args:
        profile: Configuration profile ('default', 'perfect', 'stress')
        
    Returns:
        Configured FullStackSimulator instance
    """
    profiles = {
        'default': {
            'websocket': {'network': {'latency_ms': 50.0, 'packet_loss_rate': 0.01}},
            'slcan': {'error_rate': 0.001, 'serial_delay_ms': 5.0},
            'firmware': {'simulate_faults': True, 'fault_rate': 0.0001}
        },
        'perfect': {
            'websocket': {'network': {'enabled': False}},
            'slcan': {'simulate_errors': False},
            'firmware': {'simulate_faults': False}
        },
        'stress': {
            'websocket': {'network': {'latency_ms': 200.0, 'packet_loss_rate': 0.1}},
            'slcan': {'error_rate': 0.05, 'serial_delay_ms': 20.0},
            'firmware': {'simulate_faults': True, 'fault_rate': 0.01}
        }
    }
    
    config = profiles.get(profile, profiles['default'])
    return FullStackSimulator(config)
