# URC 2026 Testing Infrastructure Implementation Plan
## Comprehensive Ready-to-Execute Plan for Testing Improvements

---

## 📋 EXECUTIVE SUMMARY

This document provides a complete implementation plan for addressing all identified testing infrastructure issues in the URC 2026 robotics platform. The plan covers 5 critical areas with specific code implementations, hardware requirements, and step-by-step procedures.

**Key Issues Addressed:**
1. Import path inconsistencies in Python modules
2. Missing HIL testing infrastructure
3. Inadequate WebSocket network testing
4. Performance degradation testing gaps
5. Integration and CI/CD pipeline deficiencies

**Implementation Timeline:** 6 weeks total
**Resource Requirements:** 2-3 developers, $2,500 hardware budget
**Risk Level:** Medium (mitigated with comprehensive testing)

---

## 🎯 IMPLEMENTATION OVERVIEW

### Phase 1: Import Path Fixes (Week 1)
### Phase 2: HIL Testing Infrastructure (Weeks 2-3)  
### Phase 3: WebSocket Network Testing (Week 3)
### Phase 4: Performance Testing Framework (Week 4)
### Phase 5: Integration & CI/CD Pipeline (Weeks 5-6)

---

## 📁 PHASE 1: IMPORT PATH FIXES

### 1.1 Current Issues Identified

**Problem Files:**
- `src/autonomy/control/integrated_critical_systems.py:30-38` - Relative import inconsistencies
- Multiple files using inconsistent relative import patterns
- Missing proper package structure configuration

### 1.2 Implementation Steps

#### Step 1.1: Run Import Path Standardization
```bash
# Execute the import fix script
python tools/scripts/phase1_import_fixes.py
```

#### Step 1.2: Validation Criteria
- [ ] All relative imports standardized to absolute imports
- [ ] Package structure properly configured
- [ ] Import validation tests pass
- [ ] LSP errors resolved

#### Step 1.3: Success Metrics
- Zero import errors in IDE
- All critical modules import successfully
- Reduced build time by 15%

---

## 🔧 PHASE 2: HIL TESTING INFRASTRUCTURE

### 2.1 Hardware Requirements

#### Essential Hardware (Budget: $2,500)
| Component | Quantity | Cost | Purpose |
|-----------|----------|------|---------|
| STM32 Nucleo Boards | 4 | $200 | Motor controllers, science payload |
| CAN Bus Adapters (USB) | 2 | $100 | Real CAN bus testing |
| Serial Communication Devices | 6 | $120 | Hardware communication |
| Power Monitoring Hardware | 1 | $300 | Power system testing |
| Network Switch (Managed) | 1 | $150 | Network failure simulation |
| Raspberry Pi 4 | 2 | $160 | Distributed testing nodes |

#### Optional Enhancement Hardware
- Oscilloscope (4-channel): $800
- Logic Analyzer: $300
- Power Analyzer: $400

### 2.2 Implementation Steps

#### Step 2.1: Enhanced HIL Manager
```bash
# File: simulation/hil/enhanced_hil_manager.py
```

**Key Features:**
- Hardware auto-discovery
- Fallback to simulation
- Real vs. simulated comparison
- Performance profiling

#### Step 2.2: Hardware Device Registry
<tool_call>write
<arg_key>content</arg_key>
<arg_value>#!/usr/bin/env python3
"""
Hardware Device Registry for HIL Testing

Maintains registry of available hardware devices and their capabilities.
Provides automatic device discovery and validation.
"""

import logging
import time
import json
from typing import Dict, List, Any, Optional, Set
from dataclasses import dataclass, asdict
from enum import Enum
from pathlib import Path

import serial.tools.list_ports
import subprocess

logger = logging.getLogger(__name__)

class DeviceType(Enum):
    """Types of hardware devices."""
    STM32_CONTROLLER = "stm32_controller"
    CAN_ADAPTER = "can_adapter"
    SERIAL_DEVICE = "serial_device"
    POWER_MONITOR = "power_monitor"
    NETWORK_DEVICE = "network_device"
    SENSOR_IMU = "sensor_imu"
    SENSOR_GPS = "sensor_gps"

class DeviceStatus(Enum):
    """Device status."""
    UNKNOWN = "unknown"
    AVAILABLE = "available"
    IN_USE = "in_use"
    ERROR = "error"
    SIMULATED = "simulated"

@dataclass
class HardwareDevice:
    """Hardware device information."""
    device_id: str
    device_type: DeviceType
    device_path: str
    description: str
    manufacturer: str
    serial_number: Optional[str] = None
    capabilities: List[str] = None
    status: DeviceStatus = DeviceStatus.UNKNOWN
    last_seen: float = 0.0
    test_results: Dict[str, Any] = None
    
    def __post_init__(self):
        if self.capabilities is None:
            self.capabilities = []
        if self.test_results is None:
            self.test_results = {}

class HardwareDeviceRegistry:
    """Registry for hardware devices used in HIL testing."""
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """Initialize device registry."""
        self.logger = logging.getLogger(f"{__name__}.HardwareDeviceRegistry")
        
        self.config = config or {}
        self.registry_file = Path(self.config.get('registry_file', 'hardware_registry.json'))
        self.auto_discover = self.config.get('auto_discover', True)
        
        # Device registry
        self.devices: Dict[str, HardwareDevice] = {}
        self.device_groups: Dict[str, List[str]] = {}
        
        # Load existing registry
        self.load_registry()
        
        # Auto-discover devices
        if self.auto_discover:
            self.discover_devices()
    
    def discover_devices(self) -> List[HardwareDevice]:
        """Discover available hardware devices."""
        self.logger.info("Discovering hardware devices...")
        
        discovered_devices = []
        
        # Discover serial devices
        discovered_devices.extend(self._discover_serial_devices())
        
        # Discover network devices
        discovered_devices.extend(self._discover_network_devices())
        
        # Discover USB devices
        discovered_devices.extend(self._discover_usb_devices())
        
        # Update registry
        for device in discovered_devices:
            self.register_device(device)
        
        # Save registry
        self.save_registry()
        
        self.logger.info(f"Discovered {len(discovered_devices)} devices")
        return discovered_devices
    
    def _discover_serial_devices(self) -> List[HardwareDevice]:
        """Discover serial devices."""
        devices = []
        
        try:
            ports = serial.tools.list_ports.comports()
            
            for port in ports:
                device = HardwareDevice(
                    device_id=f"serial_{port.device}",
                    device_type=DeviceType.SERIAL_DEVICE,
                    device_path=port.device,
                    description=port.description or "Unknown Serial Device",
                    manufacturer=port.manufacturer or "Unknown",
                    serial_number=getattr(port, 'serial_number', None),
                    capabilities=["serial_communication"],
                    status=DeviceStatus.AVAILABLE,
                    last_seen=time.time()
                )
                
                # Determine if this is a special device
                if "stm32" in port.description.lower():
                    device.device_type = DeviceType.STM32_CONTROLLER
                    device.capabilities.extend(["motor_control", "sensor_reading"])
                
                elif "can" in port.description.lower():
                    device.device_type = DeviceType.CAN_ADAPTER
                    device.capabilities.extend(["can_bus", "vehicle_communication"])
                
                devices.append(device)
                self.logger.info(f"Found serial device: {port.device} ({port.description})")
                
        except Exception as e:
            self.logger.error(f"Error discovering serial devices: {e}")
        
        return devices
    
    def _discover_network_devices(self) -> List[HardwareDevice]:
        """Discover network devices."""
        devices = []
        
        try:
            # Use ip command to discover network interfaces
            result = subprocess.run(['ip', 'link', 'show'], 
                                  capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                lines = result.stdout.split('\n')
                for line in lines:
                    if ': ' in line and 'lo:' not in line:
                        parts = line.split(': ')
                        if len(parts) >= 2:
                            interface_name = parts[1].split('@')[0]
                            
                            device = HardwareDevice(
                                device_id=f"net_{interface_name}",
                                device_type=DeviceType.NETWORK_DEVICE,
                                device_path=interface_name,
                                description=f"Network Interface {interface_name}",
                                manufacturer="System",
                                capabilities=["network_communication"],
                                status=DeviceStatus.AVAILABLE,
                                last_seen=time.time()
                            )
                            
                            devices.append(device)
                            self.logger.info(f"Found network device: {interface_name}")
                            
        except Exception as e:
            self.logger.error(f"Error discovering network devices: {e}")
        
        return devices
    
    def _discover_usb_devices(self) -> List[HardwareDevice]:
        """Discover USB devices."""
        devices = []
        
        try:
            # Use lsusb to discover USB devices
            result = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                lines = result.stdout.split('\n')
                for line in lines:
                    if line.strip():
                        # Parse lsusb output
                        parts = line.split()
                        if len(parts) >= 6:
                            bus = parts[1]
                            device = parts[3][:-1]  # Remove trailing ':'
                            description = ' '.join(parts[6:])
                            
                            # Check for known device types
                            device_type = DeviceType.SERIAL_DEVICE
                            capabilities = ["usb_communication"]
                            
                            if "stm32" in description.lower():
                                device_type = DeviceType.STM32_CONTROLLER
                                capabilities.extend(["motor_control", "sensor_reading"])
                            
                            hw_device = HardwareDevice(
                                device_id=f"usb_{bus}_{device}",
                                device_type=device_type,
                                device_path=f"/dev/bus/usb/{bus}/{device}",
                                description=description,
                                manufacturer="Unknown USB",
                                capabilities=capabilities,
                                status=DeviceStatus.AVAILABLE,
                                last_seen=time.time()
                            )
                            
                            devices.append(hw_device)
                            self.logger.info(f"Found USB device: {description}")
                            
        except Exception as e:
            self.logger.error(f"Error discovering USB devices: {e}")
        
        return devices
    
    def register_device(self, device: HardwareDevice) -> bool:
        """Register a device in the registry."""
        try:
            self.devices[device.device_id] = device
            self.logger.info(f"Registered device: {device.device_id}")
            return True
        except Exception as e:
            self.logger.error(f"Error registering device {device.device_id}: {e}")
            return False
    
    def get_device(self, device_id: str) -> Optional[HardwareDevice]:
        """Get device by ID."""
        return self.devices.get(device_id)
    
    def get_devices_by_type(self, device_type: DeviceType) -> List[HardwareDevice]:
        """Get all devices of a specific type."""
        return [device for device in self.devices.values() 
                if device.device_type == device_type]
    
    def get_available_devices(self) -> List[HardwareDevice]:
        """Get all available devices."""
        return [device for device in self.devices.values() 
                if device.status == DeviceStatus.AVAILABLE]
    
    def set_device_status(self, device_id: str, status: DeviceStatus):
        """Set device status."""
        if device_id in self.devices:
            self.devices[device_id].status = status
            self.devices[device_id].last_seen = time.time()
    
    def test_device(self, device_id: str) -> Dict[str, Any]:
        """Test a device functionality."""
        device = self.get_device(device_id)
        if not device:
            return {"success": False, "error": "Device not found"}
        
        self.logger.info(f"Testing device: {device_id}")
        test_results = {"success": False, "tests": {}}
        
        try:
            if device.device_type == DeviceType.SERIAL_DEVICE:
                test_results = self._test_serial_device(device)
            elif device.device_type == DeviceType.STM32_CONTROLLER:
                test_results = self._test_stm32_device(device)
            elif device.device_type == DeviceType.CAN_ADAPTER:
                test_results = self._test_can_adapter(device)
            elif device.device_type == DeviceType.NETWORK_DEVICE:
                test_results = self._test_network_device(device)
            
            # Update device test results
            device.test_results = test_results
            device.last_seen = time.time()
            
            if test_results.get("success", False):
                device.status = DeviceStatus.AVAILABLE
            else:
                device.status = DeviceStatus.ERROR
                
        except Exception as e:
            test_results["error"] = str(e)
            device.status = DeviceStatus.ERROR
            self.logger.error(f"Error testing device {device_id}: {e}")
        
        return test_results
    
    def _test_serial_device(self, device: HardwareDevice) -> Dict[str, Any]:
        """Test serial device functionality."""
        results = {"success": False, "tests": {}}
        
        try:
            import serial
            
            # Test serial connection
            ser = serial.Serial(device.device_path, 115200, timeout=2)
            
            # Send test command
            ser.write(b'AT\r\n')
            response = ser.read(100)
            
            ser.close()
            
            results["tests"]["connection"] = {
                "success": True,
                "response_length": len(response)
            }
            results["success"] = True
            
        except Exception as e:
            results["tests"]["connection"] = {"success": False, "error": str(e)}
        
        return results
    
    def _test_stm32_device(self, HardwareDevice) -> Dict[str, Any]:
        """Test STM32 device functionality."""
        results = {"success": False, "tests": {}}
        
        try:
            import serial
            
            # Test STM32-specific communication
            ser = serial.Serial(device.device_path, 115200, timeout=2)
            
            # Send STM32 identification command
            ser.write(b'ID\r\n')
            response = ser.read(100)
            
            ser.close()
            
            results["tests"]["identification"] = {
                "success": len(response) > 0,
                "response": response.hex()
            }
            results["success"] = len(response) > 0
            
        except Exception as e:
            results["tests"]["identification"] = {"success": False, "error": str(e)}
        
        return results
    
    def _test_can_adapter(self, device: HardwareDevice) -> Dict[str, Any]:
        """Test CAN adapter functionality."""
        results = {"success": False, "tests": {}}
        
        try:
            import serial
            
            # Test CAN adapter using SLCAN protocol
            ser = serial.Serial(device.device_path, 115200, timeout=2)
            
            # Send SLCAN version command
            ser.write(b'V\r')
            response = ser.read(10)
            
            ser.close()
            
            results["tests"]["slcan_protocol"] = {
                "success": len(response) > 0,
                "version": response.decode('utf-8', errors='ignore').strip()
            }
            results["success"] = len(response) > 0
            
        except Exception as e:
            results["tests"]["slcan_protocol"] = {"success": False, "error": str(e)}
        
        return results
    
    def _test_network_device(self, device: HardwareDevice) -> Dict[str, Any]:
        """Test network device functionality."""
        results = {"success": False, "tests": {}}
        
        try:
            # Test network interface status
            result = subprocess.run(['ip', 'link', 'show', device.device_path], 
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                output = result.stdout
                is_up = "UP" in output
                has_carrier = "LOWER_UP" in output
                
                results["tests"]["interface_status"] = {
                    "success": True,
                    "up": is_up,
                    "carrier": has_carrier
                }
                results["success"] = is_up
            else:
                results["tests"]["interface_status"] = {"success": False}
                
        except Exception as e:
            results["tests"]["interface_status"] = {"success": False, "error": str(e)}
        
        return results
    
    def save_registry(self):
        """Save registry to file."""
        try:
            registry_data = {
                "devices": {device_id: asdict(device) for device_id, device in self.devices.items()},
                "device_groups": self.device_groups,
                "last_updated": time.time()
            }
            
            with open(self.registry_file, 'w') as f:
                json.dump(registry_data, f, indent=2, default=str)
            
            self.logger.info(f"Registry saved to {self.registry_file}")
            
        except Exception as e:
            self.logger.error(f"Error saving registry: {e}")
    
    def load_registry(self):
        """Load registry from file."""
        try:
            if self.registry_file.exists():
                with open(self.registry_file, 'r') as f:
                    registry_data = json.load(f)
                
                # Load devices
                for device_id, device_data in registry_data.get("devices", {}).items():
                    # Convert string enums back to enums
                    device_data["device_type"] = DeviceType(device_data["device_type"])
                    device_data["status"] = DeviceStatus(device_data["status"])
                    
                    device = HardwareDevice(**device_data)
                    self.devices[device_id] = device
                
                # Load device groups
                self.device_groups = registry_data.get("device_groups", {})
                
                self.logger.info(f"Registry loaded from {self.registry_file}")
                
        except Exception as e:
            self.logger.error(f"Error loading registry: {e}")
    
    def get_registry_summary(self) -> Dict[str, Any]:
        """Get registry summary."""
        device_counts = {}
        for device_type in DeviceType:
            device_counts[device_type.value] = len(self.get_devices_by_type(device_type))
        
        status_counts = {}
        for status in DeviceStatus:
            status_counts[status.value] = len([d for d in self.devices.values() 
                                             if d.status == status])
        
        return {
            "total_devices": len(self.devices),
            "device_counts": device_counts,
            "status_counts": status_counts,
            "device_groups": len(self.device_groups),
            "last_updated": max([d.last_seen for d in self.devices.values()]) if self.devices else 0
        }


# Global registry instance
_registry_instance = None

def get_hardware_registry(config: Optional[Dict[str, Any]] = None) -> HardwareDeviceRegistry:
    """Get global hardware registry instance."""
    global _registry_instance
    if _registry_instance is None:
        _registry_instance = HardwareDeviceRegistry(config)
    return _registry_instance