#!/usr/bin/env python3
"""
Teleoperation Server Mock

Simulates vendor/teleoperation/server/py_server.py for testing
without requiring the actual teleoperation server or hardware.

Features:
- Socket.IO server with same event handlers as py_server.py
- CAN serial communication simulation
- Gamepad input injection
- Frontend connection management
- Matches teleoperation submodule interface exactly

Author: URC 2026 Simulation Team
"""

import asyncio
import logging
import time
from typing import Dict, Any, List, Optional, Callable
from dataclasses import dataclass
import math

from simulation.network.websocket_server_simulator import WebSocketServerSimulator
from simulation.can.slcan_protocol_simulator import SLCANProtocolSimulator

logger = logging.getLogger(__name__)


class TeleopServerMock:
    """
    Mock implementation of teleoperation server.
    
    Matches the interface of vendor/teleoperation/server/py_server.py
    for testing integration without the actual server.
    """
    
    # Message IDs matching py_server.py
    SEND_ID = {
        "SET_CHASSIS_VELOCITIES": 0x00C,
        "HEARTBEAT": 0x00E,
        "HOMING_SEQUENCE": 0x110,
        "GET_OFFSET": 0x112,
        "GET_ESTIMATED_VELOCITIES": 0x114,
        "CONFIG": 0x119,
        "SET_MAST_GIMBAL_OFFSET": 0x300,
    }
    
    RECEIVE_ID = {
        "SET_VELOCITIES_RESPONSE": 0x00D,
        "HEARTBEAT_REPLY": 0x00F,
        "HOMING_SEQUENCE_RESPONSE": 0x111,
        "RETURN_OFFSET": 0x113,
        "RETURN_ESTIMATED_CHASSIS_VELOCITIES": 0x115,
        "CONFIG_ACK": 0x11A,
    }
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """Initialize teleoperation server mock.
        
        Args:
            config: Configuration dictionary
        """
        self.logger = logging.getLogger(f"{__name__}.TeleopServerMock")
        
        # Configuration
        config = config or {}
        
        # Create underlying simulators
        self.sio = WebSocketServerSimulator(config.get('websocket', {}))
        self.slcan_sim = SLCANProtocolSimulator(config.get('slcan', {}))
        
        # Server state
        self.running = False
        self.drive_task_started = False
        self.arm_serial_connected = False
        self.drive_serial_connected = True  # Simulated as always connected
        
        # Setup event handlers (matching py_server.py)
        self._setup_event_handlers()
        
        self.logger.info("Teleoperation server mock initialized")
    
    def _setup_event_handlers(self):
        """Setup Socket.IO event handlers matching py_server.py."""
        
        @self.sio.on('driveCommands')
        async def drive_commands_handler(data):
            """Handle drive commands (matches py_server.py format).
            
            Expected data format:
            {
                'xVel': float,  # m/s
                'yVel': float,  # m/s  
                'rotVel': float  # deg/s (note: deg/s not rad/s!)
            }
            """
            try:
                # Extract velocities (py_server.py format)
                x_vel = data.get('xVel', 0.0)
                y_vel = data.get('yVel', 0.0)
                rot_vel = data.get('rotVel', 0.0)  # Already in deg/s
                
                # Scale velocities (matching py_server.py exactly)
                x_vel_scaled = int(x_vel * (2 ** 12))
                y_vel_scaled = int(y_vel * (2 ** 12))
                rot_vel_scaled = int(rot_vel * (2 ** 6))
                
                # Convert to hex bytes
                x_vel_hex = x_vel_scaled.to_bytes(2, 'big', signed=True).hex()
                y_vel_hex = y_vel_scaled.to_bytes(2, 'big', signed=True).hex()
                rot_vel_hex = rot_vel_scaled.to_bytes(2, 'big', signed=True).hex()
                
                # Create CAN message (matching py_server.py format)
                can_msg = f't{self.SEND_ID["SET_CHASSIS_VELOCITIES"]:03X}6{x_vel_hex}{y_vel_hex}{rot_vel_hex}\r'
                
                # Write to SLCAN (simulated)
                self.slcan_sim.write_to_buffer(can_msg)
                
                self.logger.debug(f"Drive command sent: {can_msg.strip()}")
                
            except Exception as e:
                self.logger.error(f"Error in driveCommands: {e}")
        
        @self.sio.on('driveHoming')
        async def drive_homing_handler(data):
            """Handle homing sequence request."""
            try:
                # Create homing CAN message (8 bytes of zeros)
                can_msg = f't{self.SEND_ID["HOMING_SEQUENCE"]:03X}80000000000000000\r'
                
                # Write to SLCAN
                self.slcan_sim.write_to_buffer(can_msg)
                
                self.logger.info("Homing initiated")
                
            except Exception as e:
                self.logger.error(f"Error in driveHoming: {e}")
        
        @self.sio.on('connect')
        async def connect_handler(data):
            """Handle client connection."""
            self.logger.info(f"Client connected: {data}")
        
        @self.sio.on('disconnect')
        async def disconnect_handler(data):
            """Handle client disconnection."""
            self.logger.info(f"Client disconnected: {data}")
    
    def parse_drive_data(self, data: bytes) -> Optional[Dict[str, Any]]:
        """Parse CAN data from drive system (matching py_server.py).
        
        Args:
            data: Raw CAN data bytes
            
        Returns:
            Dict with parsed data or None
        """
        try:
            string_data = data.decode('ascii')
            
            if len(string_data) < 5:
                return None
            
            address = string_data[1:4]
            
            if address == f'{self.RECEIVE_ID["SET_VELOCITIES_RESPONSE"]:03X}':
                # Parse velocity response
                x_vel = int(string_data[5:9], 16)
                if string_data[5] in "89ABCDEF":
                    x_vel = x_vel - int(math.pow(2, 16))
                
                y_vel = int(string_data[9:13], 16)
                if string_data[9] in "89ABCDEF":
                    y_vel = y_vel - int(math.pow(2, 16))
                
                rot_vel = int(string_data[13:], 16)
                if string_data[13] in "89ABCDEF":
                    rot_vel = rot_vel - int(math.pow(2, 16))
                
                return {
                    'type': 'velocity_response',
                    'x_vel': x_vel,
                    'y_vel': y_vel,
                    'rot_vel': rot_vel
                }
            
            elif address == f'{self.RECEIVE_ID["HEARTBEAT_REPLY"]:03X}':
                return {'type': 'heartbeat_reply'}
            
            elif address == f'{self.RECEIVE_ID["HOMING_SEQUENCE_RESPONSE"]:03X}':
                return {'type': 'homing_reply'}
            
            else:
                self.logger.debug(f"Unknown CAN message: {address}")
                return None
                
        except Exception as e:
            self.logger.error(f"Error parsing drive data: {e}")
            return None
    
    async def start(self):
        """Start teleoperation server."""
        self.running = True
        self.logger.info("Teleoperation server mock started")
    
    async def stop(self):
        """Stop teleoperation server."""
        self.running = False
        self.logger.info("Teleoperation server mock stopped")
    
    def get_status(self) -> Dict[str, Any]:
        """Get server status.
        
        Returns:
            Dict with server status
        """
        return {
            'running': self.running,
            'drive_serial_connected': self.drive_serial_connected,
            'arm_serial_connected': self.arm_serial_connected,
            'websocket_stats': self.sio.get_statistics(),
            'slcan_stats': self.slcan_sim.get_statistics()
        }


# Convenience function
def create_teleop_server_mock(config: Optional[Dict[str, Any]] = None) -> TeleopServerMock:
    """Create teleoperation server mock with default configuration.
    
    Args:
        config: Optional configuration
        
    Returns:
        Configured TeleopServerMock instance
    """
    return TeleopServerMock(config)
