#!/usr/bin/env python3
"""
Mock Hardware Server for URC 2026 Testing

Provides simulated hardware interfaces for testing critical systems
without requiring actual hardware dependencies.

Features:
- Mock GPIO operations for emergency stop testing
- Mock CAN bus interface for redundant CAN testing
- Mock IMU and sensor data for sensor fusion testing
- Mock encoder data for motor control testing
- Web API for hardware status and control

Author: URC 2026 Testing Team
"""

import time
import threading
import json
import asyncio
import random
import math
from typing import Dict, Any, List, Optional
from dataclasses import dataclass, asdict
from flask import Flask, jsonify, request
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)

@dataclass
class MockGPIOState:
    """Mock GPIO state."""
    pin: int
    value: bool
    mode: str  # 'input', 'output'
    pull_up_down: Optional[str] = None

@dataclass
class MockCANMessage:
    """Mock CAN message."""
    arbitration_id: int
    data: bytes
    timestamp: float

@dataclass
class MockSensorData:
    """Mock sensor data."""
    sensor_type: str
    data: Dict[str, Any]
    timestamp: float

class MockHardwareServer:
    """Mock hardware server for testing critical systems."""
    
    def __init__(self, host='0.0.0.0', port=8080):
        self.host = host
        self.port = port
        self.running = False
        
        # Mock hardware state
        self.gpio_states = {}
        self.can_messages = []
        self.sensor_data = {}
        self.encoder_counts = {}
        self.motor_commands = {}
        self.emergency_stop_active = False
        
        # Initialize mock hardware
        self._initialize_mock_hardware()
        
        # Threading for concurrent operations
        self.server_thread: Optional[threading.Thread] = None
        self.data_thread: Optional[threading.Thread] = None
    
    def _initialize_mock_hardware(self):
        """Initialize mock hardware components."""
        logger.info("Initializing mock hardware components")
        
        # Mock GPIO pins
        gpio_pins = [17, 18, 24]  # E-stop, motor relay, status LED
        for pin in gpio_pins:
            self.gpio_states[pin] = MockGPIOState(
                pin=pin,
                value=False,
                mode='input',
                pull_up_down='up' if pin == 17 else None
            )
        
        # Mock encoder counts
        encoder_names = ['front_left', 'front_right', 'middle_left', 'middle_right', 'rear_left', 'rear_right']
        for encoder_name in encoder_names:
            self.encoder_counts[encoder_name] = 0
        
        # Mock sensor data
        self._generate_initial_sensor_data()
        
        logger.info(f"Mock hardware initialized with {len(self.gpio_states)} GPIO pins, {len(self.encoder_counts)} encoders")
    
    def _generate_initial_sensor_data(self):
        """Generate initial mock sensor data."""
        # IMU data
        self.sensor_data['imu'] = MockSensorData(
            sensor_type='imu',
            data={
                'linear_acceleration_x': 0.0,
                'linear_acceleration_y': 0.0,
                'linear_acceleration_z': 9.81,
                'angular_velocity_x': 0.0,
                'angular_velocity_y': 0.0,
                'angular_velocity_z': 0.0
            },
            timestamp=time.time()
        )
        
        # GPS data
        self.sensor_data['gps'] = MockSensorData(
            sensor_type='gps',
            data={
                'latitude': 40.0125,  # Example coordinates
                'longitude': -105.2705,
                'altitude': 1605.0,  # Boulder, CO altitude
                'fix_type': 3,  # 3D fix
                'satellites': 8
            },
            timestamp=time.time()
        )
        
        # Battery data
        self.sensor_data['battery'] = MockSensorData(
            sensor_type='battery',
            data={
                'voltage': 24.5,
                'current': 5.2,
                'percentage': 85.0,
                'temperature': 25.0
            },
            timestamp=time.time()
        )
    
    def _update_sensor_data(self):
        """Update sensor data in real-time simulation."""
        current_time = time.time()
        
        # Update IMU with some noise
        imu_data = self.sensor_data['imu'].data.copy()
        imu_data['linear_acceleration_x'] += random.gauss(0, 0.01)
        imu_data['linear_acceleration_y'] += random.gauss(0, 0.01)
        imu_data['angular_velocity_z'] += random.gauss(0, 0.005)
        self.sensor_data['imu'] = MockSensorData('imu', imu_data, current_time)
        
        # Update GPS with slow movement
        if random.random() < 0.1:  # 10% chance of GPS update
            gps_data = self.sensor_data['gps'].data.copy()
            gps_data['latitude'] += random.gauss(0, 0.000001)
            gps_data['longitude'] += random.gauss(0, 0.000001)
            gps_data['satellites'] = max(4, gps_data['satellites'] + random.randint(-1, 2))
            self.sensor_data['gps'] = MockSensorData('gps', gps_data, current_time)
        
        # Update battery with gradual discharge
        battery_data = self.sensor_data['battery'].data.copy()
        battery_data['percentage'] = max(0.0, battery_data['percentage'] - random.gauss(0.01, 0.005))
        battery_data['temperature'] = min(40.0, battery_data['temperature'] + random.gauss(0.05, 0.02))
        self.sensor_data['battery'] = MockSensorData('battery', battery_data, current_time)
    
    def _update_encoder_counts(self):
        """Update encoder counts based on motor commands."""
        for motor_name, command in self.motor_commands.items():
            # Simple simulation: encoder count increases with motor command
            if motor_name in self.encoder_counts:
                velocity_change = abs(command) * 10  # Scale factor
                self.encoder_counts[motor_name] += int(velocity_change * 0.1)  # 10Hz update
    
    def start(self):
        """Start the mock hardware server."""
        logger.info(f"Starting mock hardware server on {self.host}:{self.port}")
        self.running = True
        
        # Start Flask app in separate thread
        self.server_thread = threading.Thread(target=app.run, kwargs={
            'host': self.host,
            'port': self.port,
            'debug': False,
            'threaded': True
        })
        self.server_thread.daemon = True
        self.server_thread.start()
        
        # Start sensor data simulation thread
        self.data_thread = threading.Thread(target=self._sensor_simulation_loop)
        self.data_thread.daemon = True
        self.data_thread.start()
        
        logger.info("Mock hardware server started")
    
    def stop(self):
        """Stop the mock hardware server."""
        logger.info("Stopping mock hardware server")
        self.running = False
        
        if self.server_thread:
            # Note: Flask doesn't have a clean stop mechanism in this simple setup
            logger.info("Mock hardware server stopped")
    
    def _sensor_simulation_loop(self):
        """Background loop for sensor data simulation."""
        while self.running:
            self._update_sensor_data()
            self._update_encoder_counts()
            time.sleep(0.1)  # 10Hz sensor update

# Flask API routes
@app.route('/health')
def health_check():
    """Health check endpoint."""
    return jsonify({'status': 'healthy', 'timestamp': time.time()})

@app.route('/gpio/<int:pin>', methods=['GET', 'POST'])
def gpio_control(pin):
    """GPIO control endpoint."""
    if not hasattr(app, 'mock_hardware'):
        # Initialize mock hardware for Flask app
        app.mock_hardware = MockHardwareServer()
    
    if request.method == 'GET':
        if pin in app.mock_hardware.gpio_states:
            return jsonify(asdict(app.mock_hardware.gpio_states[pin]))
        else:
            return jsonify({'error': 'Pin not found'}), 404
    
    elif request.method == 'POST':
        data = request.get_json()
        if 'value' in data:
            if pin in app.mock_hardware.gpio_states:
                app.mock_hardware.gpio_states[pin].value = bool(data['value'])
                app.mock_hardware.gpio_states[pin].mode = 'output'
                
                # Special handling for emergency stop
                if pin == 17 and data['value'] == False:  # Emergency stop pressed
                    app.mock_hardware.emergency_stop_active = True
                    logger.warning("Emergency stop activated via mock GPIO")
                
                return jsonify({'success': True})
            else:
                return jsonify({'error': 'Pin not found'}), 404
        else:
            return jsonify({'error': 'Missing value parameter'}), 400

@app.route('/can/send', methods=['POST'])
def can_send():
    """CAN message send endpoint."""
    data = request.get_json()
    
    required_fields = ['arbitration_id', 'data']
    if not all(field in data for field in required_fields):
        return jsonify({'error': 'Missing required fields'}), 400
    
    try:
        can_msg = MockCANMessage(
            arbitration_id=data['arbitration_id'],
            data=bytes.fromhex(data['data']),
            timestamp=time.time()
        )
        
        if not hasattr(app, 'mock_hardware'):
            app.mock_hardware = MockHardwareServer()
        
        app.mock_hardware.can_messages.append(can_msg)
        
        # Simulate CAN response for critical messages
        if can_msg.arbitration_id == 0xFFF:  # Emergency stop
            # Emergency response
            response_data = "00D6000000000000"  # Stop response
        elif can_msg.arbitration_id == 0x00C:  # Velocity command
            # Velocity feedback response
            response_data = "00D6000000000000"  # Zero velocity feedback
        else:
            response_data = "000"  # ACK
        
        return jsonify({'success': True, 'response_data': response_data})
        
    except Exception as e:
        logger.error(f"CAN send error: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/can/receive', methods=['GET'])
def can_receive():
    """CAN message receive endpoint."""
    if not hasattr(app, 'mock_hardware'):
        app.mock_hardware = MockHardwareServer()
    
    # Return recent CAN messages
    recent_messages = app.mock_hardware.can_messages[-10:]  # Last 10 messages
    return jsonify({
        'messages': [
            {
                'arbitration_id': msg.arbitration_id,
                'data': msg.data.hex(),
                'timestamp': msg.timestamp
            }
            for msg in recent_messages
        ]
    })

@app.route('/sensors/<sensor_type>', methods=['GET'])
def get_sensor_data(sensor_type):
    """Get sensor data endpoint."""
    if not hasattr(app, 'mock_hardware'):
        app.mock_hardware = MockHardwareServer()
    
    if sensor_type in app.mock_hardware.sensor_data:
        sensor_data = app.mock_hardware.sensor_data[sensor_type]
        return jsonify({
            'sensor_type': sensor_data.sensor_type,
            'data': sensor_data.data,
            'timestamp': sensor_data.timestamp
        })
    else:
        return jsonify({'error': 'Sensor not found'}), 404

@app.route('/encoders/<encoder_name>', methods=['GET', 'POST'])
def encoder_data(encoder_name):
    """Encoder data endpoint."""
    if not hasattr(app, 'mock_hardware'):
        app.mock_hardware = MockHardwareServer()
    
    if request.method == 'GET':
        if encoder_name in app.mock_hardware.encoder_counts:
            return jsonify({
                'encoder': encoder_name,
                'count': app.mock_hardware.encoder_counts[encoder_name],
                'timestamp': time.time()
            })
        else:
            return jsonify({'error': 'Encoder not found'}), 404
    
    elif request.method == 'POST':
        data = request.get_json()
        if 'command' in data:
            if encoder_name in app.mock_hardware.encoder_counts:
                # Store motor command (used for encoder simulation)
                app.mock_hardware.motor_commands[encoder_name] = float(data['command'])
                return jsonify({'success': True})
            else:
                return jsonify({'error': 'Encoder not found'}), 404
        else:
            return jsonify({'error': 'Missing command parameter'}), 400

@app.route('/motors', methods=['GET'])
def get_motor_status():
    """Get all motor status."""
    if not hasattr(app, 'mock_hardware'):
        app.mock_hardware = MockHardwareServer()
    
    motor_status = {}
    for encoder_name, count in app.mock_hardware.encoder_counts.items():
        # Simulate velocity from encoder counts
        motor_status[encoder_name] = {
            'encoder_count': count,
            'command': app.mock_hardware.motor_commands.get(encoder_name, 0.0),
            'estimated_velocity': abs(count * 0.01),  # Rough velocity estimate
            'timestamp': time.time()
        }
    
    return jsonify(motor_status)

@app.route('/emergency_stop', methods=['GET', 'POST'])
def emergency_stop_control():
    """Emergency stop control endpoint."""
    if not hasattr(app, 'mock_hardware'):
        app.mock_hardware = MockHardwareServer()
    
    if request.method == 'GET':
        return jsonify({
            'active': app.mock_hardware.emergency_stop_active,
            'timestamp': time.time()
        })
    
    elif request.method == 'POST':
        data = request.get_json()
        action = data.get('action', 'status')
        
        if action == 'activate':
            app.mock_hardware.emergency_stop_active = True
            # Set all GPIO pins to safe state
            for pin in app.mock_hardware.gpio_states.values():
                if pin.pin == 18:  # Motor power relay
                    pin.value = False  # Cut power
                elif pin.pin == 24:  # Status LED
                    pin.value = True   # Turn on LED
            
            return jsonify({'success': True, 'action': 'activated'})
        
        elif action == 'reset':
            app.mock_hardware.emergency_stop_active = False
            # Reset GPIO pins
            for pin in app.mock_hardware.gpio_states.values():
                if pin.pin == 18:  # Motor power relay
                    pin.value = True   # Restore power
                elif pin.pin == 24:  # Status LED
                    pin.value = False  # Turn off LED
            
            return jsonify({'success': True, 'action': 'reset'})
        
        else:
            return jsonify({'error': 'Invalid action'}), 400

@app.route('/status', methods=['GET'])
def get_system_status():
    """Get complete system status."""
    if not hasattr(app, 'mock_hardware'):
        app.mock_hardware = MockHardwareServer()
    
    status = {
        'timestamp': time.time(),
        'emergency_stop_active': app.mock_hardware.emergency_stop_active,
        'gpio_states': {
            str(pin): asdict(state) 
            for pin, state in app.mock_hardware.gpio_states.items()
        },
        'motor_commands': app.mock_hardware.motor_commands,
        'encoder_counts': app.mock_hardware.encoder_counts,
        'sensor_data': {
            sensor_type: {
                'data': data.data,
                'timestamp': data.timestamp
            }
            for sensor_type, data in app.mock_hardware.sensor_data.items()
        },
        'can_messages_count': len(app.mock_hardware.can_messages)
    }
    
    return jsonify(status)

if __name__ == '__main__':
    server = MockHardwareServer()
    try:
        server.start()
        
        # Keep main thread alive
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        logger.info("Received interrupt, shutting down mock hardware server")
        server.stop()