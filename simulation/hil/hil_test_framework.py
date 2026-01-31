#!/usr/bin/env python3
"""
HIL Testing Framework

Comprehensive hardware-in-the-loop testing framework that provides:
- Mixed real/simulated component testing
- Automatic hardware discovery and validation
- Performance comparison between real and simulated
- Test scenario orchestration
- Result reporting and analysis
"""

import asyncio
import logging
import time
import json
import threading
from typing import Dict, List, Any, Optional, Callable, Tuple
from dataclasses import dataclass, asdict
from enum import Enum
from pathlib import Path

from .hardware_device_registry import get_hardware_registry, DeviceType, DeviceStatus
from .hil_manager import HILManager, ComponentType, ComponentMode

logger = logging.getLogger(__name__)

class TestType(Enum):
    """Types of HIL tests."""
    FUNCTIONAL = "functional"
    PERFORMANCE = "performance"
    STRESS = "stress"
    REGRESSION = "regression"
    INTEGRATION = "integration"

class TestResult(Enum):
    """Test result status."""
    PASS = "pass"
    FAIL = "fail"
    SKIP = "skip"
    ERROR = "error"
    TIMEOUT = "timeout"

@dataclass
class TestScenario:
    """HIL test scenario definition."""
    name: str
    test_type: TestType
    description: str
    components_required: Dict[ComponentType, ComponentMode]
    test_steps: List[Dict[str, Any]]
    expected_results: Dict[str, Any]
    timeout: float = 300.0
    retry_count: int = 3

@dataclass
class TestExecution:
    """Test execution result."""
    scenario_name: str
    start_time: float
    end_time: float
    duration: float
    result: TestResult
    details: Dict[str, Any]
    component_status: Dict[str, Any]
    performance_metrics: Dict[str, float]
    error_message: Optional[str] = None

class HILTestFramework:
    """Main HIL testing framework."""
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """Initialize HIL test framework."""
        self.logger = logging.getLogger(f"{__name__}.HILTestFramework")
        
        self.config = config or {}
        
        # Initialize components
        self.hardware_registry = get_hardware_registry(self.config.get('hardware_registry', {}))
        self.hil_manager = HILManager(self.config.get('hil_manager', {}))
        
        # Test scenarios
        self.test_scenarios: Dict[str, TestScenario] = {}
        self.test_executions: List[TestExecution] = []
        
        # Test execution state
        self.is_running = False
        self.current_test = None
        self.test_stop_event = threading.Event()
        
        # Load test scenarios
        self._load_test_scenarios()
        
        self.logger.info("HIL test framework initialized")
    
    def _load_test_scenarios(self):
        """Load test scenarios from configuration."""
        
        # Basic motor control test
        motor_test = TestScenario(
            name="basic_motor_control",
            test_type=TestType.FUNCTIONAL,
            description="Test basic motor control functionality",
            components_required={
                ComponentType.FIRMWARE: ComponentMode.AUTO,
                ComponentType.CAN_BUS: ComponentMode.AUTO
            },
            test_steps=[
                {"action": "initialize", "component": "firmware"},
                {"action": "connect", "component": "can_bus"},
                {"action": "send_command", "type": "motor", "command": "forward", "duration": 2.0},
                {"action": "send_command", "type": "motor", "command": "stop", "duration": 1.0},
                {"action": "send_command", "type": "motor", "command": "reverse", "duration": 2.0},
                {"action": "send_command", "type": "motor", "command": "stop", "duration": 1.0}
            ],
            expected_results={
                "motor_response": True,
                "can_communication": True,
                "no_errors": True
            },
            timeout=60.0
        )
        
        # Sensor fusion test
        sensor_test = TestScenario(
            name="sensor_fusion_validation",
            test_type=TestType.FUNCTIONAL,
            description="Test sensor fusion and data processing",
            components_required={
                ComponentType.FIRMWARE: ComponentMode.AUTO,
                ComponentType.SENSORS: ComponentMode.AUTO
            },
            test_steps=[
                {"action": "initialize", "component": "firmware"},
                {"action": "enable_sensors", "sensors": ["imu", "gps"]},
                {"action": "collect_data", "duration": 10.0},
                {"action": "validate_fusion", "type": "position_estimation"}
            ],
            expected_results={
                "sensor_data_received": True,
                "fusion_output_valid": True,
                "position_accuracy": 1.0  # meters
            },
            timeout=30.0
        )
        
        # Performance stress test
        stress_test = TestScenario(
            name="high_frequency_control",
            test_type=TestType.STRESS,
            description="Test high-frequency control loop performance",
            components_required={
                ComponentType.FIRMWARE: ComponentMode.AUTO,
                ComponentType.CAN_BUS: ComponentMode.AUTO
            },
            test_steps=[
                {"action": "initialize", "component": "firmware"},
                {"action": "connect", "component": "can_bus"},
                {"action": "start_control_loop", "frequency": 50.0, "duration": 60.0},
                {"action": "monitor_performance", "metrics": ["latency", "jitter", "throughput"]}
            ],
            expected_results={
                "control_loop_stable": True,
                "max_latency": 0.02,  # 20ms
                "jitter_under_5ms": True,
                "no_packet_loss": True
            },
            timeout=120.0
        )
        
        # WebSocket communication test
        websocket_test = TestScenario(
            name="websocket_communication",
            test_type=TestType.INTEGRATION,
            description="Test WebSocket communication with dashboard",
            components_required={
                ComponentType.WEBSOCKET_SERVER: ComponentMode.AUTO
            },
            test_steps=[
                {"action": "start_websocket_server", "port": 8080},
                {"action": "connect_client", "client_id": "test_client"},
                {"action": "send_telemetry", "duration": 30.0},
                {"action": "receive_commands", "expected_count": 5},
                {"action": "validate_connection", "type": "bidirectional"}
            ],
            expected_results={
                "server_started": True,
                "client_connected": True,
                "telemetry_sent": True,
                "commands_received": True,
                "connection_stable": True
            },
            timeout=60.0
        )
        
        # Register scenarios
        self.test_scenarios = {
            motor_test.name: motor_test,
            sensor_test.name: sensor_test,
            stress_test.name: stress_test,
            websocket_test.name: websocket_test
        }
        
        self.logger.info(f"Loaded {len(self.test_scenarios)} test scenarios")
    
    def run_test_scenario(self, scenario_name: str) -> TestExecution:
        """Run a single test scenario."""
        scenario = self.test_scenarios.get(scenario_name)
        if not scenario:
            raise ValueError(f"Test scenario '{scenario_name}' not found")
        
        self.logger.info(f"Running test scenario: {scenario_name}")
        
        execution = TestExecution(
            scenario_name=scenario_name,
            start_time=time.time(),
            end_time=0.0,
            duration=0.0,
            result=TestResult.ERROR,
            details={},
            component_status={},
            performance_metrics={}
        )
        
        try:
            # Initialize required components
            self._initialize_test_components(scenario, execution)
            
            # Execute test steps
            self._execute_test_steps(scenario, execution)
            
            # Validate results
            self._validate_test_results(scenario, execution)
            
            execution.result = TestResult.PASS
            
        except Exception as e:
            execution.result = TestResult.ERROR
            execution.error_message = str(e)
            self.logger.error(f"Test scenario '{scenario_name}' failed: {e}")
        
        finally:
            # Cleanup
            self._cleanup_test_components(scenario, execution)
            
            execution.end_time = time.time()
            execution.duration = execution.end_time - execution.start_time
            
            # Store execution
            self.test_executions.append(execution)
            
            self.logger.info(f"Test scenario '{scenario_name}' completed: {execution.result.value}")
        
        return execution
    
    def _initialize_test_components(self, scenario: TestScenario, execution: TestExecution):
        """Initialize required components for test."""
        self.logger.info(f"Initializing components for {scenario.name}")
        
        for component_type, mode in scenario.components_required.items():
            success = self.hil_manager.initialize_component(component_type, mode)
            
            execution.component_status[component_type.value] = {
                "initialized": success,
                "mode": mode.value,
                "is_real": self.hil_manager.is_real_hardware(component_type)
            }
            
            if not success:
                raise RuntimeError(f"Failed to initialize {component_type.value}")
    
    def _execute_test_steps(self, scenario: TestScenario, execution: TestExecution):
        """Execute test steps."""
        self.logger.info(f"Executing {len(scenario.test_steps)} test steps")
        
        step_start_time = time.time()
        
        for i, step in enumerate(scenario.test_steps):
            step_start = time.time()
            
            try:
                self._execute_single_step(step, execution)
                
                step_duration = time.time() - step_start
                self.logger.info(f"Step {i+1}/{len(scenario.test_steps)} completed in {step_duration:.2f}s")
                
            except Exception as e:
                self.logger.error(f"Step {i+1} failed: {e}")
                raise
        
        total_step_time = time.time() - step_start_time
        execution.performance_metrics["step_execution_time"] = total_step_time
    
    def _execute_single_step(self, step: Dict[str, Any], execution: TestExecution):
        """Execute a single test step."""
        action = step.get("action")
        
        if action == "initialize":
            component = step.get("component")
            self.logger.info(f"Initializing component: {component}")
            
        elif action == "connect":
            component = step.get("component")
            self.logger.info(f"Connecting to component: {component}")
            
        elif action == "send_command":
            self._send_command_step(step, execution)
            
        elif action == "enable_sensors":
            self._enable_sensors_step(step, execution)
            
        elif action == "collect_data":
            duration = step.get("duration", 5.0)
            self.logger.info(f"Collecting data for {duration}s")
            time.sleep(duration)
            
        elif action == "start_control_loop":
            self._start_control_loop_step(step, execution)
            
        elif action == "start_websocket_server":
            self._start_websocket_server_step(step, execution)
            
        elif action == "connect_client":
            self._connect_client_step(step, execution)
            
        elif action == "send_telemetry":
            duration = step.get("duration", 10.0)
            self.logger.info(f"Sending telemetry for {duration}s")
            time.sleep(duration)
            
        else:
            self.logger.warning(f"Unknown step action: {action}")
    
    def _send_command_step(self, step: Dict[str, Any], execution: TestExecution):
        """Execute send command step."""
        command_type = step.get("type")
        command = step.get("command")
        duration = step.get("duration", 1.0)
        
        self.logger.info(f"Sending {command_type} command: {command}")
        
        # Get appropriate component
        if command_type == "motor":
            component = self.hil_manager.get_component(ComponentType.FIRMWARE)
            if component:
                # Send motor command
                success = self._send_motor_command(component, command)
                execution.details[f"motor_command_{command}"] = success
                time.sleep(duration)
    
    def _send_motor_command(self, component, command: str) -> bool:
        """Send motor command to component."""
        try:
            # Implementation depends on component interface
            if hasattr(component, 'send_motor_command'):
                return component.send_motor_command(command)
            elif hasattr(component, 'write'):
                # Serial interface
                if command == "forward":
                    component.write(b'MF\r\n')
                elif command == "reverse":
                    component.write(b'MR\r\n')
                elif command == "stop":
                    component.write(b'MS\r\n')
                return True
            return False
        except Exception as e:
            self.logger.error(f"Error sending motor command: {e}")
            return False
    
    def _enable_sensors_step(self, step: Dict[str, Any], execution: TestExecution):
        """Execute enable sensors step."""
        sensors = step.get("sensors", [])
        self.logger.info(f"Enabling sensors: {sensors}")
        
        component = self.hil_manager.get_component(ComponentType.FIRMWARE)
        if component and hasattr(component, 'enable_sensors'):
            success = component.enable_sensors(sensors)
            execution.details["sensors_enabled"] = success
    
    def _start_control_loop_step(self, step: Dict[str, Any], execution: TestExecution):
        """Execute control loop step."""
        frequency = step.get("frequency", 10.0)
        duration = step.get("duration", 30.0)
        
        self.logger.info(f"Starting control loop at {frequency}Hz for {duration}s")
        
        # Simulate control loop
        start_time = time.time()
        iterations = 0
        
        while time.time() - start_time < duration:
            loop_start = time.time()
            
            # Simulate control loop work
            time.sleep(0.001)  # 1ms work
            
            iterations += 1
            
            # Maintain frequency
            loop_duration = time.time() - loop_start
            target_duration = 1.0 / frequency
            if loop_duration < target_duration:
                time.sleep(target_duration - loop_duration)
        
        actual_frequency = iterations / duration
        execution.performance_metrics["control_loop_frequency"] = actual_frequency
        execution.performance_metrics["control_loop_iterations"] = iterations
    
    def _start_websocket_server_step(self, step: Dict[str, Any], execution: TestExecution):
        """Execute WebSocket server step."""
        port = step.get("port", 8080)
        self.logger.info(f"Starting WebSocket server on port {port}")
        
        component = self.hil_manager.get_component(ComponentType.WEBSOCKET_SERVER)
        if component:
            success = component.start_server(port)
            execution.details["websocket_server_started"] = success
            time.sleep(2.0)  # Give server time to start
    
    def _connect_client_step(self, step: Dict[str, Any], execution: TestExecution):
        """Execute connect client step."""
        client_id = step.get("client_id", "test_client")
        self.logger.info(f"Connecting client: {client_id}")
        
        component = self.hil_manager.get_component(ComponentType.WEBSOCKET_SERVER)
        if component and hasattr(component, 'connect_client'):
            success = component.connect_client(client_id)
            execution.details["client_connected"] = success
    
    def _validate_test_results(self, scenario: TestScenario, execution: TestExecution):
        """Validate test results against expected results."""
        self.logger.info("Validating test results")
        
        validation_results = {}
        
        for expected_key, expected_value in scenario.expected_results.items():
            actual_value = execution.details.get(expected_key)
            
            if actual_value is not None:
                # Simple validation for now
                if isinstance(expected_value, bool):
                    validation_results[expected_key] = actual_value == expected_value
                elif isinstance(expected_value, (int, float)):
                    validation_results[expected_key] = abs(actual_value - expected_value) < 0.1
                else:
                    validation_results[expected_key] = str(actual_value) == str(expected_value)
            else:
                validation_results[expected_key] = False
        
        execution.details["validation_results"] = validation_results
        
        # Overall validation
        all_passed = all(validation_results.values())
        if not all_passed:
            execution.result = TestResult.FAIL
            failed_validations = [k for k, v in validation_results.items() if not v]
            execution.error_message = f"Failed validations: {failed_validations}"
    
    def _cleanup_test_components(self, scenario: TestScenario, execution: TestExecution):
        """Cleanup test components."""
        self.logger.info("Cleaning up test components")
        
        for component_type in scenario.components_required.keys():
            try:
                component = self.hil_manager.get_component(component_type)
                if component:
                    if hasattr(component, 'stop'):
                        component.stop()
                    elif hasattr(component, 'close'):
                        component.close()
            except Exception as e:
                self.logger.error(f"Error cleaning up {component_type.value}: {e}")
    
    def run_all_tests(self) -> List[TestExecution]:
        """Run all test scenarios."""
        self.logger.info("Running all test scenarios")
        
        executions = []
        
        for scenario_name in self.test_scenarios.keys():
            try:
                execution = self.run_test_scenario(scenario_name)
                executions.append(execution)
            except Exception as e:
                self.logger.error(f"Failed to run scenario {scenario_name}: {e}")
        
        return executions
    
    def get_test_report(self) -> Dict[str, Any]:
        """Generate comprehensive test report."""
        if not self.test_executions:
            return {"error": "No test executions available"}
        
        # Calculate statistics
        total_tests = len(self.test_executions)
        passed_tests = len([e for e in self.test_executions if e.result == TestResult.PASS])
        failed_tests = len([e for e in self.test_executions if e.result == TestResult.FAIL])
        error_tests = len([e for e in self.test_executions if e.result == TestResult.ERROR])
        
        # Performance metrics
        avg_duration = sum(e.duration for e in self.test_executions) / total_tests
        
        # Component usage
        component_usage = {}
        for execution in self.test_executions:
            for comp_name, comp_status in execution.component_status.items():
                if comp_name not in component_usage:
                    component_usage[comp_name] = {"real": 0, "simulated": 0}
                
                if comp_status.get("is_real", False):
                    component_usage[comp_name]["real"] += 1
                else:
                    component_usage[comp_name]["simulated"] += 1
        
        return {
            "summary": {
                "total_tests": total_tests,
                "passed": passed_tests,
                "failed": failed_tests,
                "errors": error_tests,
                "success_rate": passed_tests / total_tests if total_tests > 0 else 0,
                "average_duration": avg_duration
            },
            "component_usage": component_usage,
            "executions": [asdict(exec) for execution in self.test_executions],
            "hardware_registry": self.hardware_registry.get_registry_summary()
        }
    
    def save_test_report(self, filename: Optional[str] = None):
        """Save test report to file."""
        if filename is None:
            timestamp = int(time.time())
            filename = f"hil_test_report_{timestamp}.json"
        
        report = self.get_test_report()
        
        try:
            with open(filename, 'w') as f:
                json.dump(report, f, indent=2, default=str)
            
            self.logger.info(f"Test report saved to {filename}")
            
        except Exception as e:
            self.logger.error(f"Error saving test report: {e}")


# Convenience function
def create_hil_test_framework(config: Optional[Dict[str, Any]] = None) -> HILTestFramework:
    """Create HIL test framework with configuration."""
    return HILTestFramework(config)