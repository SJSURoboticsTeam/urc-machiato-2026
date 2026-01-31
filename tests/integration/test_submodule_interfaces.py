#!/usr/bin/env python3
"""
Test Submodule Interface Compatibility

Tests the integration between:
- Main ROS2 codebase
- Teleoperation server
- Control-systems firmware

Author: URC 2026 Integration Test Team
"""

import asyncio
import json
import struct
import time
from pathlib import Path
from typing import Dict, List, Optional
import sys
import os

# Debug logging configuration
DEBUG_LOG_PATH = Path("/home/durian/urc-machiato-2026/.cursor/debug.log")
SESSION_ID = "interface-test"


def log_debug(location: str, message: str, data: Dict = None, hypothesis_id: str = None):
    """Write debug log entry for instrumentation."""
    log_entry = {
        "sessionId": SESSION_ID,
        "timestamp": int(time.time() * 1000),
        "location": location,
        "message": message,
        "data": data or {},
    }
    if hypothesis_id:
        log_entry["hypothesisId"] = hypothesis_id
    
    try:
        with open(DEBUG_LOG_PATH, "a") as f:
            f.write(json.dumps(log_entry) + "\n")
    except Exception as e:
        print(f"Failed to write debug log: {e}")


class InterfaceTestResult:
    """Test result container."""
    def __init__(self, test_name: str):
        self.test_name = test_name
        self.passed = False
        self.errors = []
        self.warnings = []
        self.details = {}
    
    def add_error(self, error: str):
        self.errors.append(error)
    
    def add_warning(self, warning: str):
        self.warnings.append(warning)
    
    def __str__(self):
        status = "PASS" if self.passed else "FAIL"
        result = f"\n{'='*60}\n"
        result += f"Test: {self.test_name}\n"
        result += f"Status: {status}\n"
        if self.errors:
            result += f"\nErrors ({len(self.errors)}):\n"
            for err in self.errors:
                result += f"  - {err}\n"
        if self.warnings:
            result += f"\nWarnings ({len(self.warnings)}):\n"
            for warn in self.warnings:
                result += f"  - {warn}\n"
        if self.details:
            result += f"\nDetails:\n"
            for key, value in self.details.items():
                result += f"  {key}: {value}\n"
        result += f"{'='*60}\n"
        return result


class TeleopProtocolTester:
    """Test teleoperation CAN protocol compatibility."""
    
    # Teleoperation protocol message IDs
    SEND_IDS = {
        "SET_CHASSIS_VELOCITIES": 0x00C,
        "HEARTBEAT": 0x00E,
        "HOMING_SEQUENCE": 0x110,
        "GET_OFFSET": 0x112,
        "GET_ESTIMATED_VELOCITIES": 0x114,
        "CONFIG": 0x119,
        "SET_MAST_GIMBAL_OFFSET": 0x300,
    }
    
    RECEIVE_IDS = {
        "SET_VELOCITIES_RESPONSE": 0x00D,
        "HEARTBEAT_REPLY": 0x00F,
        "HOMING_SEQUENCE_RESPONSE": 0x111,
        "RETURN_OFFSET": 0x113,
        "RETURN_ESTIMATED_CHASSIS_VELOCITIES": 0x115,
        "CONFIG_ACK": 0x11A,
    }
    
    def __init__(self):
        self.results = []
    
    def test_message_encoding(self) -> InterfaceTestResult:
        """Test velocity message encoding matches teleoperation protocol."""
        # #region agent log
        log_debug("test_submodule_interfaces.py:101", "Starting message encoding test", 
                 {"test": "message_encoding"}, "H3")
        # #endregion
        
        result = InterfaceTestResult("Teleoperation Message Encoding")
        
        try:
            # Test case: 0.5 m/s forward, 0 m/s lateral, 15 deg/s rotation
            x_vel = 0.5  # m/s
            y_vel = 0.0  # m/s
            rot_vel = 15.0  # deg/s
            
            # #region agent log
            log_debug("test_submodule_interfaces.py:112", "Input velocities", 
                     {"x_vel": x_vel, "y_vel": y_vel, "rot_vel": rot_vel}, "H3")
            # #endregion
            
            # Teleoperation encoding
            x_vel_scaled = int(x_vel * (2 ** 12))
            y_vel_scaled = int(y_vel * (2 ** 12))
            rot_vel_scaled = int(rot_vel * (2 ** 6))
            
            # #region agent log
            log_debug("test_submodule_interfaces.py:122", "Scaled velocities", 
                     {"x_vel_scaled": x_vel_scaled, "y_vel_scaled": y_vel_scaled, 
                      "rot_vel_scaled": rot_vel_scaled}, "H3")
            # #endregion
            
            # Convert to bytes (big-endian, signed)
            x_bytes = x_vel_scaled.to_bytes(2, 'big', signed=True)
            y_bytes = y_vel_scaled.to_bytes(2, 'big', signed=True)
            rot_bytes = rot_vel_scaled.to_bytes(2, 'big', signed=True)
            
            # Create SLCAN frame
            msg_id = self.SEND_IDS["SET_CHASSIS_VELOCITIES"]
            slcan_frame = f't{msg_id:03X}6{x_bytes.hex()}{y_bytes.hex()}{rot_bytes.hex()}\r'
            
            result.details["Expected SLCAN Frame"] = slcan_frame
            result.details["X Velocity (scaled)"] = f"{x_vel_scaled} (0x{x_vel_scaled:04X})"
            result.details["Y Velocity (scaled)"] = f"{y_vel_scaled} (0x{y_vel_scaled:04X})"
            result.details["Rot Velocity (scaled)"] = f"{rot_vel_scaled} (0x{rot_vel_scaled:04X})"
            
            # #region agent log
            log_debug("test_submodule_interfaces.py:142", "Generated SLCAN frame", 
                     {"slcan_frame": slcan_frame, "msg_id": hex(msg_id)}, "H3")
            # #endregion
            
            # Verify decoding
            decoded_x = int.from_bytes(x_bytes, 'big', signed=True) / (2 ** 12)
            decoded_y = int.from_bytes(y_bytes, 'big', signed=True) / (2 ** 12)
            decoded_rot = int.from_bytes(rot_bytes, 'big', signed=True) / (2 ** 6)
            
            # #region agent log
            log_debug("test_submodule_interfaces.py:153", "Decoded velocities", 
                     {"decoded_x": decoded_x, "decoded_y": decoded_y, "decoded_rot": decoded_rot}, "H3")
            # #endregion
            
            # Verify round-trip accuracy
            if abs(decoded_x - x_vel) > 0.001:
                result.add_error(f"X velocity round-trip error: {decoded_x} != {x_vel}")
            if abs(decoded_y - y_vel) > 0.001:
                result.add_error(f"Y velocity round-trip error: {decoded_y} != {y_vel}")
            if abs(decoded_rot - rot_vel) > 0.1:
                result.add_error(f"Rot velocity round-trip error: {decoded_rot} != {rot_vel}")
            
            if not result.errors:
                result.passed = True
                # #region agent log
                log_debug("test_submodule_interfaces.py:168", "Message encoding test PASSED", 
                         {"result": "success"}, "H3")
                # #endregion
            else:
                # #region agent log
                log_debug("test_submodule_interfaces.py:173", "Message encoding test FAILED", 
                         {"errors": result.errors}, "H3")
                # #endregion
                
        except Exception as e:
            result.add_error(f"Exception during encoding test: {e}")
            # #region agent log
            log_debug("test_submodule_interfaces.py:180", "Exception in message encoding test", 
                     {"exception": str(e)}, "H3")
            # #endregion
        
        return result
    
    def test_message_id_coverage(self) -> InterfaceTestResult:
        """Test that all teleoperation message IDs are documented."""
        # #region agent log
        log_debug("test_submodule_interfaces.py:189", "Starting message ID coverage test", 
                 {"test": "message_id_coverage"}, "H2")
        # #endregion
        
        result = InterfaceTestResult("Teleoperation Message ID Coverage")
        
        try:
            # Check for overlapping IDs
            all_ids = set(self.SEND_IDS.values()) | set(self.RECEIVE_IDS.values())
            send_set = set(self.SEND_IDS.values())
            receive_set = set(self.RECEIVE_IDS.values())
            
            overlap = send_set & receive_set
            if overlap:
                result.add_error(f"Overlapping message IDs found: {[hex(x) for x in overlap]}")
                # #region agent log
                log_debug("test_submodule_interfaces.py:204", "Overlapping IDs detected", 
                         {"overlap": [hex(x) for x in overlap]}, "H2")
                # #endregion
            
            result.details["Total Message IDs"] = len(all_ids)
            result.details["Send IDs"] = len(send_set)
            result.details["Receive IDs"] = len(receive_set)
            result.details["ID Range"] = f"{hex(min(all_ids))} - {hex(max(all_ids))}"
            
            # #region agent log
            log_debug("test_submodule_interfaces.py:214", "Message ID coverage analysis", 
                     {"total_ids": len(all_ids), "send_ids": len(send_set), 
                      "receive_ids": len(receive_set), "id_range": f"{hex(min(all_ids))}-{hex(max(all_ids))}"}, "H2")
            # #endregion
            
            if not result.errors:
                result.passed = True
                
        except Exception as e:
            result.add_error(f"Exception during ID coverage test: {e}")
            # #region agent log
            log_debug("test_submodule_interfaces.py:225", "Exception in message ID coverage test", 
                     {"exception": str(e)}, "H2")
            # #endregion
        
        return result
    
    def test_firmware_compatibility(self) -> InterfaceTestResult:
        """Test compatibility with control-systems firmware requirements."""
        # #region agent log
        log_debug("test_submodule_interfaces.py:235", "Starting firmware compatibility test", 
                 {"test": "firmware_compatibility"}, "H5")
        # #endregion
        
        result = InterfaceTestResult("Control-Systems Firmware Compatibility")
        
        try:
            # Check if firmware application exists
            firmware_path = Path("/home/durian/urc-machiato-2026/vendor/control-systems/drive/applications/application.cpp")
            
            # #region agent log
            log_debug("test_submodule_interfaces.py:246", "Checking firmware application file", 
                     {"firmware_path": str(firmware_path), "exists": firmware_path.exists()}, "H5")
            # #endregion
            
            if not firmware_path.exists():
                result.add_error(f"Firmware application not found: {firmware_path}")
                result.passed = False
                return result
            
            # Read firmware application
            with open(firmware_path, 'r') as f:
                firmware_code = f.read()
            
            # #region agent log
            log_debug("test_submodule_interfaces.py:259", "Read firmware application", 
                     {"file_size": len(firmware_code), "line_count": firmware_code.count('\n')}, "H5")
            # #endregion
            
            # Check for CAN message handling implementation
            checks = {
                "CAN message parser": ["can_receive", "CAN", "message", "parse"],
                "Velocity command handler": ["SET_CHASSIS_VELOCITIES", "00C", "velocity"],
                "Heartbeat handler": ["HEARTBEAT", "00E", "heartbeat"],
                "Homing handler": ["HOMING", "110", "home"],
            }
            
            for check_name, keywords in checks.items():
                found = any(keyword in firmware_code for keyword in keywords)
                if found:
                    result.details[check_name] = "✓ Found"
                    # #region agent log
                    log_debug("test_submodule_interfaces.py:276", f"Check passed: {check_name}", 
                             {"check": check_name, "found": True}, "H5")
                    # #endregion
                else:
                    result.details[check_name] = "✗ Not found"
                    result.add_warning(f"{check_name} not implemented in firmware")
                    # #region agent log
                    log_debug("test_submodule_interfaces.py:283", f"Check failed: {check_name}", 
                             {"check": check_name, "found": False}, "H5")
                    # #endregion
            
            # Check if it's just a stub
            if "each loop:" in firmware_code and firmware_code.count("\n") < 25:
                result.add_error("Firmware application appears to be a stub (not implemented)")
                result.details["Implementation Status"] = "STUB - Needs implementation"
                # #region agent log
                log_debug("test_submodule_interfaces.py:292", "Firmware is a stub", 
                         {"status": "stub", "implementation_needed": True}, "H5")
                # #endregion
            else:
                result.details["Implementation Status"] = "Partial or complete"
                # #region agent log
                log_debug("test_submodule_interfaces.py:298", "Firmware has implementation", 
                         {"status": "implemented"}, "H5")
                # #endregion
            
            # Pass if no errors (warnings are acceptable)
            if not result.errors:
                result.passed = True
                
        except Exception as e:
            result.add_error(f"Exception during firmware compatibility test: {e}")
            # #region agent log
            log_debug("test_submodule_interfaces.py:309", "Exception in firmware compatibility test", 
                     {"exception": str(e)}, "H5")
            # #endregion
        
        return result


class MainCodebaseProtocolTester:
    """Test main codebase CAN protocol compatibility."""
    
    # Main codebase message IDs
    MESSAGE_IDS = {
        "MOTOR_COMMAND": 0x100,
        "MOTOR_FEEDBACK": 0x101,
        "IMU_DATA": 0x200,
        "ENCODER_DATA": 0x201,
        "BATTERY_STATUS": 0x300,
        "SYSTEM_STATUS": 0x400,
        "EMERGENCY_STOP": 0xFFF,
    }
    
    def test_protocol_compatibility(self) -> InterfaceTestResult:
        """Test protocol compatibility between main codebase and teleoperation."""
        # #region agent log
        log_debug("test_submodule_interfaces.py:334", "Starting protocol compatibility test", 
                 {"test": "protocol_compatibility"}, "H2")
        # #endregion
        
        result = InterfaceTestResult("Main Codebase Protocol Compatibility")
        
        try:
            teleop_tester = TeleopProtocolTester()
            teleop_ids = set(teleop_tester.SEND_IDS.values()) | set(teleop_tester.RECEIVE_IDS.values())
            main_ids = set(self.MESSAGE_IDS.values())
            
            # #region agent log
            log_debug("test_submodule_interfaces.py:346", "Comparing message ID sets", 
                     {"teleop_ids_count": len(teleop_ids), "main_ids_count": len(main_ids)}, "H2")
            # #endregion
            
            # Check for ID conflicts
            conflicts = teleop_ids & main_ids
            if conflicts:
                result.add_error(f"Message ID conflicts: {[hex(x) for x in conflicts]}")
                result.details["Conflicting IDs"] = [hex(x) for x in conflicts]
                # #region agent log
                log_debug("test_submodule_interfaces.py:356", "Message ID conflicts found", 
                         {"conflicts": [hex(x) for x in conflicts]}, "H2")
                # #endregion
            else:
                result.details["ID Conflicts"] = "None"
                # #region agent log
                log_debug("test_submodule_interfaces.py:362", "No message ID conflicts", 
                         {"conflicts": 0}, "H2")
                # #endregion
            
            # Check ID ranges
            teleop_range = (min(teleop_ids), max(teleop_ids))
            main_range = (min(main_ids), max(main_ids))
            
            result.details["Teleoperation ID Range"] = f"{hex(teleop_range[0])} - {hex(teleop_range[1])}"
            result.details["Main Codebase ID Range"] = f"{hex(main_range[0])} - {hex(main_range[1])}"
            
            # #region agent log
            log_debug("test_submodule_interfaces.py:375", "Message ID range comparison", 
                     {"teleop_range": f"{hex(teleop_range[0])}-{hex(teleop_range[1])}", 
                      "main_range": f"{hex(main_range[0])}-{hex(main_range[1])}"}, "H2")
            # #endregion
            
            # Warning if ranges overlap
            if (main_range[0] <= teleop_range[1] and main_range[1] >= teleop_range[0]):
                result.add_warning("Message ID ranges overlap - potential conflicts")
                # #region agent log
                log_debug("test_submodule_interfaces.py:385", "Message ID ranges overlap", 
                         {"warning": "potential_conflicts"}, "H2")
                # #endregion
            
            if not result.errors:
                result.passed = True
                
        except Exception as e:
            result.add_error(f"Exception during protocol compatibility test: {e}")
            # #region agent log
            log_debug("test_submodule_interfaces.py:396", "Exception in protocol compatibility test", 
                     {"exception": str(e)}, "H2")
            # #endregion
        
        return result
    
    def test_hardware_interface_config(self) -> InterfaceTestResult:
        """Test hardware interface node configuration."""
        # #region agent log
        log_debug("test_submodule_interfaces.py:406", "Starting hardware interface config test", 
                 {"test": "hardware_interface_config"}, "H4")
        # #endregion
        
        result = InterfaceTestResult("Hardware Interface Node Configuration")
        
        try:
            # Check if hardware interface node exists (autonomy_core package path)
            script_dir = Path(__file__).resolve().parent
            workspace_root = script_dir.parent.parent
            hw_interface_path = workspace_root / "src/autonomy/autonomy_core/autonomy_core/control/hardware_interface_node.py"
            
            # #region agent log
            log_debug("test_submodule_interfaces.py:417", "Checking hardware interface node", 
                     {"path": str(hw_interface_path), "exists": hw_interface_path.exists()}, "H4")
            # #endregion
            
            if not hw_interface_path.exists():
                result.add_error(f"Hardware interface node not found: {hw_interface_path}")
                result.passed = False
                return result
            
            # Read hardware interface code
            with open(hw_interface_path, 'r') as f:
                hw_interface_code = f.read()
            
            # #region agent log
            log_debug("test_submodule_interfaces.py:431", "Read hardware interface node", 
                     {"file_size": len(hw_interface_code)}, "H4")
            # #endregion
            
            # Check for required components
            checks = {
                "CAN Serial Import": "can_serial" in hw_interface_code or "CanSerial" in hw_interface_code,
                "Twist Mux": "mux" in hw_interface_code.lower() and "priority" in hw_interface_code.lower(),
                "Emergency Stop": "emergency" in hw_interface_code.lower(),
                "Device Configuration": "/dev/tty" in hw_interface_code,
                "ROS2 Lifecycle": "LifecycleNode" in hw_interface_code,
            }
            
            for check_name, passed in checks.items():
                if passed:
                    result.details[check_name] = "✓ Found"
                    # #region agent log
                    log_debug("test_submodule_interfaces.py:448", f"Check passed: {check_name}", 
                             {"check": check_name, "found": True}, "H4")
                    # #endregion
                else:
                    result.details[check_name] = "✗ Not found"
                    result.add_warning(f"{check_name} not found in hardware interface")
                    # #region agent log
                    log_debug("test_submodule_interfaces.py:455", f"Check failed: {check_name}", 
                             {"check": check_name, "found": False}, "H4")
                    # #endregion
            
            # Check for device paths
            device_paths = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyAMA10"]
            found_devices = [path for path in device_paths if path in hw_interface_code]
            
            result.details["Device Paths Found"] = found_devices if found_devices else "None"
            
            # #region agent log
            log_debug("test_submodule_interfaces.py:467", "Device path analysis", 
                     {"found_devices": found_devices}, "H4")
            # #endregion
            
            if not result.errors:
                result.passed = True
                
        except Exception as e:
            result.add_error(f"Exception during hardware interface config test: {e}")
            # #region agent log
            log_debug("test_submodule_interfaces.py:478", "Exception in hardware interface config test", 
                     {"exception": str(e)}, "H4")
            # #endregion
        
        return result


class InterfaceTestSuite:
    """Complete interface test suite."""
    
    def __init__(self):
        self.teleop_tester = TeleopProtocolTester()
        self.main_tester = MainCodebaseProtocolTester()
        self.results = []
    
    def run_all_tests(self):
        """Run all interface tests."""
        print("\n" + "="*60)
        print("SUBMODULE INTERFACE COMPATIBILITY TEST SUITE")
        print("="*60 + "\n")
        
        # #region agent log
        log_debug("test_submodule_interfaces.py:501", "Starting full test suite", 
                 {"session": SESSION_ID}, "H1")
        # #endregion
        
        # Teleoperation protocol tests
        print("Running Teleoperation Protocol Tests...")
        self.results.append(self.teleop_tester.test_message_encoding())
        self.results.append(self.teleop_tester.test_message_id_coverage())
        self.results.append(self.teleop_tester.test_firmware_compatibility())
        
        # Main codebase protocol tests
        print("Running Main Codebase Protocol Tests...")
        self.results.append(self.main_tester.test_protocol_compatibility())
        self.results.append(self.main_tester.test_hardware_interface_config())
        
        # #region agent log
        log_debug("test_submodule_interfaces.py:517", "All tests completed", 
                 {"total_tests": len(self.results)}, "H1")
        # #endregion
        
        # Print results
        for result in self.results:
            print(result)
        
        # Summary
        passed = sum(1 for r in self.results if r.passed)
        failed = len(self.results) - passed
        
        print("\n" + "="*60)
        print("TEST SUMMARY")
        print("="*60)
        print(f"Total Tests: {len(self.results)}")
        print(f"Passed: {passed}")
        print(f"Failed: {failed}")
        print(f"Pass Rate: {passed/len(self.results)*100:.1f}%")
        print("="*60 + "\n")
        
        # #region agent log
        log_debug("test_submodule_interfaces.py:539", "Test suite complete", 
                 {"total_tests": len(self.results), "passed": passed, "failed": failed, 
                  "pass_rate": passed/len(self.results)*100}, "H1")
        # #endregion
        
        return passed == len(self.results)


def main():
    """Main test entry point."""
    print("Submodule Interface Compatibility Test")
    print("Testing integration between:")
    print("  - Main ROS2 Codebase")
    print("  - Teleoperation Server")
    print("  - Control-Systems Firmware")
    print()
    
    # #region agent log
    log_debug("test_submodule_interfaces.py:557", "Test suite starting", 
             {"session": SESSION_ID, "timestamp": time.time()}, "H1")
    # #endregion
    
    suite = InterfaceTestSuite()
    success = suite.run_all_tests()
    
    # #region agent log
    log_debug("test_submodule_interfaces.py:565", "Test suite finished", 
             {"session": SESSION_ID, "success": success, "timestamp": time.time()}, "H1")
    # #endregion
    
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
