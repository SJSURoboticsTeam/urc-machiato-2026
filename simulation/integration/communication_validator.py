#!/usr/bin/env python3
"""
Communication Validator

Validates communication paths and message flows in the simulation,
ensuring all interfaces work correctly before hardware testing.

Features:
- Message flow validation
- Latency measurement
- Data integrity checking
- Protocol compliance verification

Author: URC 2026 Simulation Team
"""

import logging
import time
import hashlib
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum

logger = logging.getLogger(__name__)


class ValidationStatus(Enum):
    """Validation result status."""
    PASS = "pass"
    FAIL = "fail"
    WARNING = "warning"
    SKIPPED = "skipped"


@dataclass
class ValidationResult:
    """Result of a validation check."""
    check_name: str
    status: ValidationStatus
    message: str
    details: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


class CommunicationValidator:
    """
    Validates communication paths in simulation.
    
    Ensures all interfaces, protocols, and message flows
    work correctly before hardware deployment.
    """
    
    def __init__(self):
        """Initialize communication validator."""
        self.logger = logging.getLogger(f"{__name__}.CommunicationValidator")
        self.validation_results: List[ValidationResult] = []
    
    def validate_websocket_to_firmware(self, full_stack_sim) -> ValidationResult:
        """Validate WebSocket → Firmware path.
        
        Args:
            full_stack_sim: FullStackSimulator instance
            
        Returns:
            ValidationResult
        """
        try:
            start_time = time.time()
            
            # Send test message
            import asyncio
            async def test():
                client_id = await full_stack_sim.websocket_sim.connect()
                await full_stack_sim.websocket_sim.receive('driveCommands', {
                    'linear': 0.5,
                    'angular': 0.0
                }, client_id)
                await asyncio.sleep(0.1)
            
            asyncio.run(test())
            
            latency = time.time() - start_time
            
            # Check firmware received command
            motor_status = full_stack_sim.firmware_sim.get_motor_status(0)
            
            if motor_status and abs(motor_status['velocity_setpoint']) > 0.0:
                result = ValidationResult(
                    check_name="websocket_to_firmware",
                    status=ValidationStatus.PASS,
                    message=f"Path validated in {latency*1000:.1f}ms",
                    details={'latency_ms': latency * 1000}
                )
            else:
                result = ValidationResult(
                    check_name="websocket_to_firmware",
                    status=ValidationStatus.FAIL,
                    message="Command did not reach firmware"
                )
            
            self.validation_results.append(result)
            return result
            
        except Exception as e:
            result = ValidationResult(
                check_name="websocket_to_firmware",
                status=ValidationStatus.FAIL,
                message=f"Validation error: {e}"
            )
            self.validation_results.append(result)
            return result
    
    def validate_slcan_protocol(self, slcan_sim) -> ValidationResult:
        """Validate SLCAN protocol encoding/decoding.
        
        Args:
            slcan_sim: SLCANProtocolSimulator instance
            
        Returns:
            ValidationResult
        """
        try:
            test_cases = [
                (0.5, 0.0, 0.0),
                (0.0, 0.0, 0.2618),
                (0.5, 0.25, 0.2618)
            ]
            
            errors = []
            
            for linear_x, linear_y, angular_z in test_cases:
                # Encode
                frame = slcan_sim.encode_velocity_command(linear_x, linear_y, angular_z)
                
                # Decode
                cmd = slcan_sim.decode_velocity_command(frame)
                
                if not cmd:
                    errors.append(f"Failed to decode: {frame}")
                    continue
                
                # Check accuracy
                if abs(cmd.linear_x - linear_x) > 0.001:
                    errors.append(f"Linear X mismatch: {cmd.linear_x} vs {linear_x}")
                if abs(cmd.linear_y - linear_y) > 0.001:
                    errors.append(f"Linear Y mismatch: {cmd.linear_y} vs {linear_y}")
                if abs(cmd.angular_z - angular_z) > 0.001:
                    errors.append(f"Angular Z mismatch: {cmd.angular_z} vs {angular_z}")
            
            if errors:
                result = ValidationResult(
                    check_name="slcan_protocol",
                    status=ValidationStatus.FAIL,
                    message=f"{len(errors)} encoding errors",
                    details={'errors': errors}
                )
            else:
                result = ValidationResult(
                    check_name="slcan_protocol",
                    status=ValidationStatus.PASS,
                    message=f"All {len(test_cases)} test cases passed"
                )
            
            self.validation_results.append(result)
            return result
            
        except Exception as e:
            result = ValidationResult(
                check_name="slcan_protocol",
                status=ValidationStatus.FAIL,
                message=f"Validation error: {e}"
            )
            self.validation_results.append(result)
            return result
    
    def validate_emergency_stop_timing(self, full_stack_sim) -> ValidationResult:
        """Validate emergency stop propagation time (<100ms requirement).
        
        Args:
            full_stack_sim: FullStackSimulator instance
            
        Returns:
            ValidationResult
        """
        try:
            import asyncio
            
            async def test():
                client_id = await full_stack_sim.websocket_sim.connect()
                
                # Send drive command
                await full_stack_sim.websocket_sim.receive('driveCommands', {
                    'linear': 1.0,
                    'angular': 0.5
                }, client_id)
                
                await asyncio.sleep(0.05)
                
                # Trigger emergency stop
                stop_start = time.time()
                await full_stack_sim.websocket_sim.receive('emergencyStop', {}, client_id)
                await asyncio.sleep(0.05)
                
                return time.time() - stop_start
            
            latency = asyncio.run(test())
            
            # Check if all motors stopped
            all_stopped = True
            for motor_id in range(full_stack_sim.firmware_sim.num_motors):
                status = full_stack_sim.firmware_sim.get_motor_status(motor_id)
                if status and abs(status['velocity_actual']) > 0.01:
                    all_stopped = False
            
            if all_stopped and latency < 0.1:
                result = ValidationResult(
                    check_name="emergency_stop_timing",
                    status=ValidationStatus.PASS,
                    message=f"E-stop propagated in {latency*1000:.1f}ms (requirement: <100ms)",
                    details={'latency_ms': latency * 1000}
                )
            elif all_stopped:
                result = ValidationResult(
                    check_name="emergency_stop_timing",
                    status=ValidationStatus.WARNING,
                    message=f"E-stop worked but slow: {latency*1000:.1f}ms > 100ms"
                )
            else:
                result = ValidationResult(
                    check_name="emergency_stop_timing",
                    status=ValidationStatus.FAIL,
                    message="Motors did not stop"
                )
            
            self.validation_results.append(result)
            return result
            
        except Exception as e:
            result = ValidationResult(
                check_name="emergency_stop_timing",
                status=ValidationStatus.FAIL,
                message=f"Validation error: {e}"
            )
            self.validation_results.append(result)
            return result
    
    def run_full_validation(self, full_stack_sim) -> Dict[str, Any]:
        """Run complete validation suite.
        
        Args:
            full_stack_sim: FullStackSimulator instance
            
        Returns:
            Dict with validation summary
        """
        self.logger.info("Running full communication validation...")
        
        results = [
            self.validate_websocket_to_firmware(full_stack_sim),
            self.validate_slcan_protocol(full_stack_sim.slcan_sim),
            self.validate_emergency_stop_timing(full_stack_sim)
        ]
        
        passed = sum(1 for r in results if r.status == ValidationStatus.PASS)
        warnings = sum(1 for r in results if r.status == ValidationStatus.WARNING)
        failed = sum(1 for r in results if r.status == ValidationStatus.FAIL)
        
        summary = {
            'total_checks': len(results),
            'passed': passed,
            'warnings': warnings,
            'failed': failed,
            'results': results,
            'overall_status': 'PASS' if failed == 0 else 'FAIL'
        }
        
        self.logger.info(f"Validation complete: {passed}/{len(results)} passed")
        
        return summary


# Convenience function
def validate_simulation(full_stack_sim) -> Dict[str, Any]:
    """Quick validation of simulation.
    
    Args:
        full_stack_sim: FullStackSimulator instance
        
    Returns:
        Dict with validation results
    """
    validator = CommunicationValidator()
    return validator.run_full_validation(full_stack_sim)
