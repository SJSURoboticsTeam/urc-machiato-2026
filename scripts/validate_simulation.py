#!/usr/bin/env python3
"""
Simulation Framework Validation Script

Runs comprehensive validation of all simulation components with
detailed logging and reporting.

Author: URC 2026 Validation Team
"""

import subprocess
import sys
import json
import time
from pathlib import Path
from datetime import datetime
from typing import Dict, Any, List


class SimulationValidator:
    """Validates simulation framework functionality."""
    
    def __init__(self, output_dir: str = "output/validation"):
        """Initialize validator.
        
        Args:
            output_dir: Directory for validation reports
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.results = {
            'timestamp': datetime.now().isoformat(),
            'validation_id': f"validation_{int(time.time())}",
            'tests': {},
            'summary': {}
        }
    
    def run_unit_tests(self) -> Dict[str, Any]:
        """Run all simulator unit tests.
        
        Returns:
            Test results dictionary
        """
        print("\n" + "=" * 60)
        print("PHASE 1: UNIT TESTS")
        print("=" * 60)
        
        test_files = [
            'tests/unit/test_websocket_server_simulator.py',
            'tests/unit/test_slcan_protocol_simulator.py',
            'tests/unit/test_stm32_firmware_simulator.py',
            'tests/unit/test_full_stack_simulator_unit.py',
        ]
        
        results = {}
        
        for test_file in test_files:
            test_name = Path(test_file).stem
            print(f"\n🧪 Running {test_name}...")
            
            try:
                result = subprocess.run(
                    ['python3', '-m', 'pytest', test_file, '-v', '--tb=short'],
                    capture_output=True,
                    text=True,
                    timeout=60
                )
                
                passed = result.returncode == 0
                
                results[test_name] = {
                    'passed': passed,
                    'returncode': result.returncode,
                    'stdout_lines': len(result.stdout.splitlines()),
                    'stderr_lines': len(result.stderr.splitlines())
                }
                
                # Extract test count from output
                if 'passed' in result.stdout:
                    import re
                    match = re.search(r'(\d+) passed', result.stdout)
                    if match:
                        results[test_name]['tests_passed'] = int(match.group(1))
                
                status = "✅ PASS" if passed else "❌ FAIL"
                print(f"  {status} - {test_name}")
                
                if not passed and result.stderr:
                    print(f"  Error output: {result.stderr[:500]}")
                
            except subprocess.TimeoutExpired:
                print(f"  ⏱️  TIMEOUT - {test_name}")
                results[test_name] = {'passed': False, 'error': 'timeout'}
            except Exception as e:
                print(f"  ❌ ERROR - {test_name}: {e}")
                results[test_name] = {'passed': False, 'error': str(e)}
        
        self.results['tests']['unit_tests'] = results
        return results
    
    def run_integration_tests(self) -> Dict[str, Any]:
        """Run integration tests.
        
        Returns:
            Test results dictionary
        """
        print("\n" + "=" * 60)
        print("PHASE 2: INTEGRATION TESTS")
        print("=" * 60)
        
        test_files = [
            'tests/integration/test_complete_communication_stack.py',
            'tests/integration/test_ros2_simulation_integration.py',
            'tests/integration/test_hil_framework.py',
        ]
        
        results = {}
        
        for test_file in test_files:
            test_name = Path(test_file).stem
            print(f"\n🔗 Running {test_name}...")
            
            try:
                result = subprocess.run(
                    ['python3', '-m', 'pytest', test_file, '-v', '--tb=short'],
                    capture_output=True,
                    text=True,
                    timeout=120
                )
                
                passed = result.returncode == 0
                
                results[test_name] = {
                    'passed': passed,
                    'returncode': result.returncode
                }
                
                status = "✅ PASS" if passed else "❌ FAIL"
                print(f"  {status} - {test_name}")
                
            except Exception as e:
                print(f"  ❌ ERROR - {test_name}: {e}")
                results[test_name] = {'passed': False, 'error': str(e)}
        
        self.results['tests']['integration_tests'] = results
        return results
    
    def run_performance_tests(self) -> Dict[str, Any]:
        """Run performance tests.
        
        Returns:
            Test results dictionary
        """
        print("\n" + "=" * 60)
        print("PHASE 3: PERFORMANCE TESTS")
        print("=" * 60)
        
        test_files = [
            'tests/performance/test_simulation_throughput.py',
            'tests/performance/test_simulation_latency.py',
        ]
        
        results = {}
        
        for test_file in test_files:
            test_name = Path(test_file).stem
            print(f"\n⚡ Running {test_name}...")
            
            try:
                result = subprocess.run(
                    ['python3', '-m', 'pytest', test_file, '-v', '--tb=short'],
                    capture_output=True,
                    text=True,
                    timeout=120
                )
                
                passed = result.returncode == 0
                
                results[test_name] = {
                    'passed': passed,
                    'returncode': result.returncode
                }
                
                status = "✅ PASS" if passed else "❌ FAIL"
                print(f"  {status} - {test_name}")
                
            except Exception as e:
                print(f"  ❌ ERROR - {test_name}: {e}")
                results[test_name] = {'passed': False, 'error': str(e)}
        
        self.results['tests']['performance_tests'] = results
        return results
    
    def verify_component_imports(self) -> Dict[str, bool]:
        """Verify all simulation components can be imported.
        
        Returns:
            Import status dictionary
        """
        print("\n" + "=" * 60)
        print("PHASE 0: COMPONENT IMPORT VERIFICATION")
        print("=" * 60)
        
        components = {
            'websocket': 'simulation.network.websocket_server_simulator',
            'slcan': 'simulation.can.slcan_protocol_simulator',
            'firmware': 'simulation.firmware.stm32_firmware_simulator',
            'full_stack': 'simulation.integration.full_stack_simulator',
            'ros2_adapter': 'simulation.ros2.ros2_message_adapter',
            'config_loader': 'simulation.config.config_loader',
        }
        
        import_results = {}
        
        for name, module_path in components.items():
            try:
                __import__(module_path)
                import_results[name] = True
                print(f"✅ {name}: Import successful")
            except Exception as e:
                import_results[name] = False
                print(f"❌ {name}: Import failed - {e}")
        
        self.results['imports'] = import_results
        return import_results
    
    def generate_summary(self) -> Dict[str, Any]:
        """Generate validation summary.
        
        Returns:
            Summary dictionary
        """
        print("\n" + "=" * 60)
        print("VALIDATION SUMMARY")
        print("=" * 60)
        
        # Count imports
        imports = self.results.get('imports', {})
        imports_passed = sum(1 for v in imports.values() if v)
        imports_total = len(imports)
        
        # Count unit tests
        unit_tests = self.results.get('tests', {}).get('unit_tests', {})
        unit_passed = sum(1 for t in unit_tests.values() if t.get('passed', False))
        unit_total = len(unit_tests)
        
        # Count integration tests
        integration_tests = self.results.get('tests', {}).get('integration_tests', {})
        integration_passed = sum(1 for t in integration_tests.values() if t.get('passed', False))
        integration_total = len(integration_tests)
        
        # Count performance tests
        perf_tests = self.results.get('tests', {}).get('performance_tests', {})
        perf_passed = sum(1 for t in perf_tests.values() if t.get('passed', False))
        perf_total = len(perf_tests)
        
        summary = {
            'imports': {
                'passed': imports_passed,
                'total': imports_total,
                'pass_rate': imports_passed / imports_total if imports_total > 0 else 0
            },
            'unit_tests': {
                'passed': unit_passed,
                'total': unit_total,
                'pass_rate': unit_passed / unit_total if unit_total > 0 else 0
            },
            'integration_tests': {
                'passed': integration_passed,
                'total': integration_total,
                'pass_rate': integration_passed / integration_total if integration_total > 0 else 0
            },
            'performance_tests': {
                'passed': perf_passed,
                'total': perf_total,
                'pass_rate': perf_passed / perf_total if perf_total > 0 else 0
            },
            'overall': {
                'passed': imports_passed + unit_passed + integration_passed + perf_passed,
                'total': imports_total + unit_total + integration_total + perf_total,
            }
        }
        
        summary['overall']['pass_rate'] = (
            summary['overall']['passed'] / summary['overall']['total']
            if summary['overall']['total'] > 0 else 0
        )
        
        self.results['summary'] = summary
        
        # Print summary
        print(f"\n📊 Results:")
        print(f"  Component Imports: {imports_passed}/{imports_total} ({summary['imports']['pass_rate']*100:.0f}%)")
        print(f"  Unit Tests: {unit_passed}/{unit_total} ({summary['unit_tests']['pass_rate']*100:.0f}%)")
        print(f"  Integration Tests: {integration_passed}/{integration_total} ({summary['integration_tests']['pass_rate']*100:.0f}%)")
        print(f"  Performance Tests: {perf_passed}/{perf_total} ({summary['performance_tests']['pass_rate']*100:.0f}%)")
        print(f"\n  OVERALL: {summary['overall']['passed']}/{summary['overall']['total']} ({summary['overall']['pass_rate']*100:.0f}%)")
        
        return summary
    
    def save_report(self):
        """Save validation report to file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_file = self.output_dir / f"validation_report_{timestamp}.json"
        
        with open(report_file, 'w') as f:
            json.dump(self.results, f, indent=2)
        
        print(f"\n💾 Report saved: {report_file}")
        
        return report_file
    
    def run_full_validation(self) -> bool:
        """Run complete validation suite.
        
        Returns:
            True if all validation passes
        """
        print("=" * 60)
        print("SIMULATION FRAMEWORK VALIDATION")
        print("=" * 60)
        print(f"Validation ID: {self.results['validation_id']}")
        print(f"Timestamp: {self.results['timestamp']}")
        
        # Phase 0: Verify imports
        self.verify_component_imports()
        
        # Phase 1: Unit tests
        self.run_unit_tests()
        
        # Phase 2: Integration tests
        self.run_integration_tests()
        
        # Phase 3: Performance tests
        self.run_performance_tests()
        
        # Generate summary
        summary = self.generate_summary()
        
        # Save report
        report_file = self.save_report()
        
        # Determine overall success
        all_passed = summary['overall']['pass_rate'] >= 0.8  # 80% threshold
        
        print("\n" + "=" * 60)
        if all_passed:
            print("✅ VALIDATION PASSED")
        else:
            print("❌ VALIDATION FAILED")
        print("=" * 60)
        
        return all_passed


def main():
    """Run validation."""
    validator = SimulationValidator()
    
    success = validator.run_full_validation()
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
