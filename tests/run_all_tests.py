#!/usr/bin/env python3
"""
Master Test Runner - URC 2026 Testing Pyramid
Orchestrates the complete testing hierarchy in proper order.

Testing Pyramid Order:
1. UNIT TESTS - Individual components in isolation
2. INTEGRATION TESTS - Component interactions and data flow
3. SIMULATION TESTS - Full system under simulated conditions

Each layer must pass before proceeding to the next.

Author: URC 2026 Autonomy Team
"""

import json
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List


class TestingPyramidRunner:
    """Master runner that orchestrates the complete testing pyramid."""

    def __init__(self):
        self.project_root = Path(__file__).parent.parent
        self.test_reports_dir = self.project_root / "test_reports"
        self.test_reports_dir.mkdir(exist_ok=True)

    def run_testing_pyramid(self) -> Dict[str, Any]:
        """Run the complete testing pyramid in order."""
        start_time = time.time()

        print("🏗️  URC 2026 - COMPLETE TESTING PYRAMID")
        print("=" * 60)
        print("Testing hierarchy: Unit → Integration → Simulation")
        print("Each layer must pass before proceeding to the next")
        print("=" * 60)
        print()

        pyramid_results = {
            "pyramid_execution": {
                "start_time": datetime.now().isoformat(),
                "layers": []
            },
            "unit_tests": None,
            "integration_tests": None,
            "simulation_tests": None,
            "overall_success": False,
            "stopping_layer": None
        }

        # Layer 1: Unit Tests
        print("📦 LAYER 1: UNIT TESTS")
        print("-" * 30)
        print("Testing individual components in isolation...")
        print()

        unit_success = self._run_test_layer("./tests/run_unit_tests.py", "unit_tests")
        pyramid_results["unit_tests"] = self._load_test_results("unit_test_results.json")
        pyramid_results["pyramid_execution"]["layers"].append({
            "name": "unit_tests",
            "success": unit_success,
            "order": 1
        })

        if not unit_success:
            print("\n❌ UNIT TESTS FAILED - Stopping pyramid execution")
            pyramid_results["overall_success"] = False
            pyramid_results["stopping_layer"] = "unit_tests"
            return self._finalize_results(pyramid_results, start_time)

        print("\n✅ UNIT TESTS PASSED - Proceeding to integration tests")
        print()

        # Layer 2: Integration Tests
        print("🔗 LAYER 2: INTEGRATION TESTS")
        print("-" * 30)
        print("Testing component interactions and data flow...")
        print()

        integration_success = self._run_test_layer("./tests/run_integration_tests.py", "integration_tests")
        pyramid_results["integration_tests"] = self._load_test_results("integration_test_results.json")
        pyramid_results["pyramid_execution"]["layers"].append({
            "name": "integration_tests",
            "success": integration_success,
            "order": 2
        })

        if not integration_success:
            print("\n⚠️  INTEGRATION TESTS HAVE ISSUES - Stopping pyramid execution")
            print("Components may not work together properly - review integration failures")
            pyramid_results["overall_success"] = False
            pyramid_results["stopping_layer"] = "integration_tests"
            return self._finalize_results(pyramid_results, start_time)

        print("\n✅ INTEGRATION TESTS PASSED - Proceeding to simulation tests")
        print()

        # Layer 3: Simulation Tests
        print("🚀 LAYER 3: SIMULATION & NETWORK TESTS")
        print("-" * 30)
        print("Testing full system under simulated conditions...")
        print()

        simulation_success = self._run_test_layer("./tests/run_simulation_integration.sh", "simulation_tests")
        pyramid_results["simulation_tests"] = self._load_test_results("comprehensive_integration_report.json")
        pyramid_results["pyramid_execution"]["layers"].append({
            "name": "simulation_tests",
            "success": simulation_success,
            "order": 3
        })

        if not simulation_success:
            print("\n⚠️  SIMULATION TESTS HAVE ISSUES - Review simulation failures")
            print("System may have issues in real-world conditions")
            pyramid_results["overall_success"] = False
            pyramid_results["stopping_layer"] = "simulation_tests"
            return self._finalize_results(pyramid_results, start_time)

        print("\n🎉 ALL TESTING LAYERS PASSED!")
        print("✅ Unit tests: PASSED")
        print("✅ Integration tests: PASSED")
        print("✅ Simulation tests: PASSED")
        print("\n🏆 TESTING PYRAMID COMPLETE - System ready for hardware validation")

        pyramid_results["overall_success"] = True
        return self._finalize_results(pyramid_results, start_time)

    def _run_test_layer(self, command: str, layer_name: str) -> bool:
        """Run a single test layer and return success status."""
        try:
            # Set up environment with proper PYTHONPATH
            env = os.environ.copy()
            env['PYTHONPATH'] = f"{self.project_root}:{self.project_root}/Autonomy:{self.project_root}/Autonomy/code:{env.get('PYTHONPATH', '')}"

            result = subprocess.run(
                command,
                shell=True,
                cwd=self.project_root,
                env=env,
                capture_output=False,  # Show output live
                text=True
            )
            return result.returncode == 0
        except Exception as e:
            print(f"❌ Error running {layer_name}: {e}")
            return False

    def _load_test_results(self, filename: str) -> Dict[str, Any]:
        """Load test results from JSON file."""
        results_file = self.test_reports_dir / filename
        if results_file.exists():
            try:
                with open(results_file, 'r') as f:
                    return json.load(f)
            except Exception as e:
                print(f"⚠️  Error loading {filename}: {e}")
        return {"error": "Results file not found or invalid"}

    def _finalize_results(self, results: Dict[str, Any], start_time: float) -> Dict[str, Any]:
        """Finalize and save pyramid results."""
        end_time = time.time()
        results["pyramid_execution"]["end_time"] = datetime.now().isoformat()
        results["pyramid_execution"]["total_duration_seconds"] = end_time - start_time

        # Save master results
        master_results_file = self.test_reports_dir / "testing_pyramid_results.json"
        with open(master_results_file, 'w') as f:
            json.dump(results, f, indent=2, default=str)

        # Print final summary
        self._print_final_summary(results)

        return results

    def _print_final_summary(self, results: Dict[str, Any]):
        """Print final testing pyramid summary."""
        print("\n" + "=" * 60)
        print("📊 TESTING PYRAMID EXECUTION SUMMARY")
        print("=" * 60)

        execution = results["pyramid_execution"]
        print(f"Total Duration: {execution['total_duration_seconds']:.2f} seconds")
        print(f"Layers Executed: {len(execution['layers'])}")

        print("\nLayer Results:")
        for layer in execution['layers']:
            status = "✅ PASSED" if layer['success'] else "❌ FAILED"
            print(f"  {layer['order']}. {layer['name']}: {status}")

        print("\nOverall Status:")
        if results["overall_success"]:
            print("🎉 PYRAMID COMPLETE - All layers passed")
            print("   Ready for hardware integration and deployment")
        else:
            stopping_layer = results.get("stopping_layer", "unknown")
            print(f"🛑 PYRAMID STOPPED at {stopping_layer}")
            print("   Fix issues in failed layer before proceeding")

        print(f"\n📄 Detailed results saved to: test_reports/testing_pyramid_results.json")

    def run_specific_layer(self, layer: str) -> Dict[str, Any]:
        """Run only a specific layer of the testing pyramid."""
        layer_commands = {
            "unit": ("./tests/run_unit_tests.py", "unit_test_results.json"),
            "integration": ("./tests/run_integration_tests.py", "integration_test_results.json"),
            "simulation": ("./tests/run_simulation_integration.sh", "comprehensive_integration_report.json")
        }

        if layer not in layer_commands:
            print(f"❌ Unknown layer: {layer}")
            print(f"Available layers: {', '.join(layer_commands.keys())}")
            return {"error": f"Unknown layer: {layer}"}

        command, results_file = layer_commands[layer]
        print(f"🏃 Running only {layer} layer...")

        success = self._run_test_layer(command, layer)
        results = self._load_test_results(results_file)

        return {
            "layer": layer,
            "success": success,
            "results": results
        }


def main():
    """Main entry point."""
    if len(sys.argv) > 1:
        layer = sys.argv[1]
        runner = TestingPyramidRunner()
        result = runner.run_specific_layer(layer)
        success = result.get("success", False)
        sys.exit(0 if success else 1)
    else:
        # Run complete pyramid
        runner = TestingPyramidRunner()
        results = runner.run_testing_pyramid()
        success = results.get("overall_success", False)
        sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
