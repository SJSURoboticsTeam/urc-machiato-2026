#!/usr/bin/env python3
"""
Hardware-in-the-Loop (HIL) Manager

Manages mixed real/simulated component testing, allowing gradual
transition from full simulation to full hardware.

Features:
- Mix real and simulated components
- Hardware device discovery
- Automatic fallback to simulation
- Comparison reporting (sim vs. real)
- Performance profiling

Author: URC 2026 HIL Framework Team
"""

import logging
import time
import serial.tools.list_ports
from typing import Dict, Any, List, Optional, Set
from dataclasses import dataclass
from enum import Enum

from simulation.firmware.stm32_firmware_simulator import STM32FirmwareSimulator
from simulation.can.slcan_protocol_simulator import SLCANProtocolSimulator
from simulation.network.websocket_server_simulator import WebSocketServerSimulator

logger = logging.getLogger(__name__)


class ComponentType(Enum):
    """Types of components that can be real or simulated."""
    FIRMWARE = "firmware"
    CAN_BUS = "can_bus"
    WEBSOCKET_SERVER = "websocket_server"
    SENSORS = "sensors"
    MOTORS = "motors"


class ComponentMode(Enum):
    """Component operation mode."""
    SIMULATED = "simulated"
    REAL = "real"
    AUTO = "auto"  # Try real, fallback to simulated


@dataclass
class ComponentStatus:
    """Status of a HIL component."""
    component_type: ComponentType
    mode: ComponentMode
    is_real: bool
    device_path: Optional[str] = None
    error: Optional[str] = None
    last_update: float = 0.0


class HILManager:
    """
    Manages hardware-in-the-loop testing.
    
    Coordinates mixed real/simulated components for gradual
    hardware integration and validation.
    """
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """Initialize HIL manager.
        
        Args:
            config: Configuration dictionary
        """
        self.logger = logging.getLogger(f"{__name__}.HILManager")
        
        # Configuration
        config = config or {}
        self.auto_discover = config.get('auto_discover', True)
        self.fallback_to_simulation = config.get('fallback_to_simulation', True)
        
        # Component configuration
        self.component_config = config.get('components', {})
        
        # Component instances
        self.components: Dict[ComponentType, Any] = {}
        self.component_status: Dict[ComponentType, ComponentStatus] = {}
        
        # Discovered hardware
        self.available_devices: List[Dict[str, str]] = []
        
        # Comparison data (real vs. simulated)
        self.comparison_data: List[Dict[str, Any]] = []
        self.max_comparison_history = config.get('max_comparison_history', 10000)
        
        self.logger.info("HIL manager initialized")
    
    def discover_hardware(self) -> List[Dict[str, str]]:
        """Discover available hardware devices.
        
        Returns:
            List of discovered devices
        """
        self.logger.info("Discovering hardware devices...")
        
        devices = []
        
        # Discover serial ports (for CAN adapters)
        ports = serial.tools.list_ports.comports()
        for port in ports:
            device_info = {
                'type': 'serial',
                'device': port.device,
                'description': port.description,
                'hwid': port.hwid,
                'manufacturer': port.manufacturer or 'Unknown'
            }
            devices.append(device_info)
            self.logger.info(f"Found serial device: {port.device} ({port.description})")
        
        self.available_devices = devices
        return devices
    
    def initialize_component(self, component_type: ComponentType, 
                           mode: ComponentMode = ComponentMode.AUTO) -> bool:
        """Initialize a component (real or simulated).
        
        Args:
            component_type: Type of component to initialize
            mode: Desired mode (auto, real, or simulated)
            
        Returns:
            bool: True if initialized successfully
        """
        self.logger.info(f"Initializing {component_type.value} in {mode.value} mode")
        
        try:
            # Try real hardware if requested
            if mode in [ComponentMode.REAL, ComponentMode.AUTO]:
                real_component = self._try_initialize_real(component_type)
                if real_component:
                    self.components[component_type] = real_component
                    self.component_status[component_type] = ComponentStatus(
                        component_type=component_type,
                        mode=ComponentMode.REAL,
                        is_real=True,
                        last_update=time.time()
                    )
                    self.logger.info(f"{component_type.value} initialized with REAL hardware")
                    return True
            
            # Fall back to simulation
            if mode in [ComponentMode.SIMULATED, ComponentMode.AUTO]:
                if self.fallback_to_simulation or mode == ComponentMode.SIMULATED:
                    simulated_component = self._create_simulated(component_type)
                    self.components[component_type] = simulated_component
                    self.component_status[component_type] = ComponentStatus(
                        component_type=component_type,
                        mode=ComponentMode.SIMULATED,
                        is_real=False,
                        last_update=time.time()
                    )
                    self.logger.info(f"{component_type.value} initialized with SIMULATION")
                    return True
            
            self.logger.error(f"Failed to initialize {component_type.value}")
            return False
            
        except Exception as e:
            self.logger.error(f"Error initializing {component_type.value}: {e}")
            self.component_status[component_type] = ComponentStatus(
                component_type=component_type,
                mode=ComponentMode.SIMULATED,
                is_real=False,
                error=str(e),
                last_update=time.time()
            )
            return False
    
    def _try_initialize_real(self, component_type: ComponentType) -> Optional[Any]:
        """Try to initialize real hardware.
        
        Args:
            component_type: Type of component
            
        Returns:
            Component instance or None if not available
        """
        if component_type == ComponentType.CAN_BUS:
            # Try to find CAN adapter
            for device in self.available_devices:
                if device['type'] == 'serial':
                    # Try to open as CAN adapter
                    try:
                        import serial
                        ser = serial.Serial(device['device'], 115200, timeout=0.1)
                        # Send test command
                        ser.write(b'V\r')  # SLCAN version command
                        time.sleep(0.1)
                        response = ser.read(10)
                        if response:
                            self.logger.info(f"Found CAN adapter: {device['device']}")
                            return ser
                    except:
                        pass
        
        # No real hardware found
        return None
    
    def _create_simulated(self, component_type: ComponentType) -> Any:
        """Create simulated component.
        
        Args:
            component_type: Type of component
            
        Returns:
            Simulated component instance
        """
        if component_type == ComponentType.FIRMWARE:
            return STM32FirmwareSimulator(self.component_config.get('firmware', {}))
        
        elif component_type == ComponentType.CAN_BUS:
            return SLCANProtocolSimulator(self.component_config.get('slcan', {}))
        
        elif component_type == ComponentType.WEBSOCKET_SERVER:
            return WebSocketServerSimulator(self.component_config.get('websocket', {}))
        
        else:
            raise ValueError(f"Unknown component type: {component_type}")
    
    def get_component(self, component_type: ComponentType) -> Optional[Any]:
        """Get component instance.
        
        Args:
            component_type: Type of component
            
        Returns:
            Component instance or None
        """
        return self.components.get(component_type)
    
    def is_real_hardware(self, component_type: ComponentType) -> bool:
        """Check if component is using real hardware.
        
        Args:
            component_type: Type of component
            
        Returns:
            bool: True if real hardware
        """
        status = self.component_status.get(component_type)
        return status.is_real if status else False
    
    def get_status_report(self) -> Dict[str, Any]:
        """Get complete HIL status report.
        
        Returns:
            Dict with status of all components
        """
        return {
            'available_devices': len(self.available_devices),
            'components': {
                comp_type.value: {
                    'mode': status.mode.value,
                    'is_real': status.is_real,
                    'device_path': status.device_path,
                    'error': status.error
                }
                for comp_type, status in self.component_status.items()
            },
            'comparison_data_points': len(self.comparison_data)
        }
    
    def record_comparison(self, real_data: Dict[str, Any], 
                         sim_data: Dict[str, Any]):
        """Record comparison data point.
        
        Args:
            real_data: Data from real hardware
            sim_data: Data from simulation
        """
        comparison = {
            'timestamp': time.time(),
            'real': real_data,
            'simulated': sim_data,
            'difference': self._calculate_difference(real_data, sim_data)
        }
        
        self.comparison_data.append(comparison)
        
        # Trim history
        if len(self.comparison_data) > self.max_comparison_history:
            self.comparison_data = self.comparison_data[-self.max_comparison_history:]
    
    def _calculate_difference(self, real_data: Dict[str, Any], 
                             sim_data: Dict[str, Any]) -> Dict[str, Any]:
        """Calculate difference between real and simulated data.
        
        Args:
            real_data: Real hardware data
            sim_data: Simulated data
            
        Returns:
            Dict with differences
        """
        differences = {}
        
        # Compare numeric fields
        for key in real_data:
            if key in sim_data:
                if isinstance(real_data[key], (int, float)) and isinstance(sim_data[key], (int, float)):
                    diff = abs(real_data[key] - sim_data[key])
                    differences[key] = diff
        
        return differences
    
    def shutdown(self):
        """Shutdown all components."""
        for component_type, component in self.components.items():
            try:
                if hasattr(component, 'stop'):
                    component.stop()
                elif hasattr(component, 'close'):
                    component.close()
            except Exception as e:
                self.logger.error(f"Error stopping {component_type.value}: {e}")
        
        self.logger.info("HIL manager shutdown")


# Convenience function
def create_hil_manager(config: Optional[Dict[str, Any]] = None) -> HILManager:
    """Create HIL manager with configuration.
    
    Args:
        config: Optional configuration
        
    Returns:
        Configured HILManager instance
    """
    return HILManager(config)
