#!/usr/bin/env python3
"""
Hardware Device Discovery

Discovers and identifies hardware devices for HIL testing,
including automatic fallback to simulation.

Features:
- Serial port discovery
- CAN adapter identification
- STM32 device detection
- Capability detection
- Health monitoring

Author: URC 2026 HIL Framework Team
"""

import logging
import serial
import serial.tools.list_ports
import time
from typing import Dict, Any, List, Optional
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class DeviceType(Enum):
    """Types of hardware devices."""
    CAN_ADAPTER = "can_adapter"
    STM32_BOARD = "stm32_board"
    SERIAL_DEVICE = "serial_device"
    USB_DEVICE = "usb_device"
    UNKNOWN = "unknown"


@dataclass
class HardwareDevice:
    """Represents a discovered hardware device."""
    device_path: str
    device_type: DeviceType
    manufacturer: str
    description: str
    serial_number: Optional[str] = None
    capabilities: List[str] = None
    is_available: bool = True
    last_seen: float = 0.0
    
    def __post_init__(self):
        if self.capabilities is None:
            self.capabilities = []
        self.last_seen = time.time()


class DeviceDiscovery:
    """
    Hardware device discovery and identification.
    
    Automatically finds and identifies hardware devices
    for HIL testing with graceful fallback.
    """
    
    # Known device identifiers
    KNOWN_DEVICES = {
        'STM32': ['STMicroelectronics', 'STM32'],
        'SLCAN': ['SLCAN', 'CANable', 'USB2CAN'],
        'FTDI': ['FTDI', 'FT232'],
        'CP2102': ['CP210', 'Silicon Labs']
    }
    
    def __init__(self):
        """Initialize device discovery."""
        self.logger = logging.getLogger(f"{__name__}.DeviceDiscovery")
        self.discovered_devices: List[HardwareDevice] = []
    
    def scan(self) -> List[HardwareDevice]:
        """Scan for all available hardware devices.
        
        Returns:
            List of discovered devices
        """
        self.logger.info("Scanning for hardware devices...")
        
        devices = []
        
        # Scan serial ports
        ports = serial.tools.list_ports.comports()
        for port in ports:
            device = self._identify_device(port)
            devices.append(device)
            self.logger.info(
                f"Found: {device.device_path} - {device.device_type.value} "
                f"({device.manufacturer})"
            )
        
        self.discovered_devices = devices
        return devices
    
    def _identify_device(self, port_info) -> HardwareDevice:
        """Identify device type from port information.
        
        Args:
            port_info: Serial port info from list_ports
            
        Returns:
            HardwareDevice with identified type
        """
        device_path = port_info.device
        manufacturer = port_info.manufacturer or 'Unknown'
        description = port_info.description or 'Unknown'
        serial_number = port_info.serial_number
        
        # Identify device type
        device_type = DeviceType.UNKNOWN
        capabilities = []
        
        # Check for STM32
        for keyword in self.KNOWN_DEVICES['STM32']:
            if keyword.lower() in manufacturer.lower() or keyword.lower() in description.lower():
                device_type = DeviceType.STM32_BOARD
                capabilities.append('firmware_programming')
                capabilities.append('can_communication')
                break
        
        # Check for SLCAN adapter
        if device_type == DeviceType.UNKNOWN:
            for keyword in self.KNOWN_DEVICES['SLCAN']:
                if keyword.lower() in description.lower():
                    device_type = DeviceType.CAN_ADAPTER
                    capabilities.append('can_communication')
                    break
        
        # Default to generic serial
        if device_type == DeviceType.UNKNOWN:
            device_type = DeviceType.SERIAL_DEVICE
        
        return HardwareDevice(
            device_path=device_path,
            device_type=device_type,
            manufacturer=manufacturer,
            description=description,
            serial_number=serial_number,
            capabilities=capabilities
        )
    
    def find_by_type(self, device_type: DeviceType) -> List[HardwareDevice]:
        """Find all devices of a specific type.
        
        Args:
            device_type: Type to search for
            
        Returns:
            List of matching devices
        """
        return [d for d in self.discovered_devices if d.device_type == device_type]
    
    def find_can_adapter(self) -> Optional[HardwareDevice]:
        """Find first available CAN adapter.
        
        Returns:
            HardwareDevice or None
        """
        can_devices = self.find_by_type(DeviceType.CAN_ADAPTER)
        if can_devices:
            return can_devices[0]
        
        # Fallback to STM32 boards (they can do CAN)
        stm32_devices = self.find_by_type(DeviceType.STM32_BOARD)
        if stm32_devices:
            return stm32_devices[0]
        
        return None
    
    def test_device(self, device: HardwareDevice) -> bool:
        """Test if device is functional.
        
        Args:
            device: Device to test
            
        Returns:
            bool: True if device is working
        """
        try:
            ser = serial.Serial(device.device_path, 115200, timeout=1.0)
            
            # Send test command
            ser.write(b'V\r')  # SLCAN version
            time.sleep(0.1)
            response = ser.read(100)
            
            ser.close()
            
            is_working = len(response) > 0
            device.is_available = is_working
            device.last_seen = time.time()
            
            return is_working
            
        except Exception as e:
            self.logger.error(f"Device test failed for {device.device_path}: {e}")
            device.is_available = False
            return False
    
    def get_summary(self) -> Dict[str, Any]:
        """Get discovery summary.
        
        Returns:
            Dict with summary information
        """
        return {
            'total_devices': len(self.discovered_devices),
            'by_type': {
                dtype.value: len(self.find_by_type(dtype))
                for dtype in DeviceType
            },
            'available': sum(1 for d in self.discovered_devices if d.is_available),
            'devices': [
                {
                    'path': d.device_path,
                    'type': d.device_type.value,
                    'manufacturer': d.manufacturer,
                    'available': d.is_available
                }
                for d in self.discovered_devices
            ]
        }


# Convenience function
def discover_devices() -> List[HardwareDevice]:
    """Quick device discovery.
    
    Returns:
        List of discovered devices
    """
    discovery = DeviceDiscovery()
    return discovery.scan()
