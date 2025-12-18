#!/usr/bin/env python3
"""
Full Deployment Validation Script
Executes complete test suite validation with improved meaningful tests.
"""

import subprocess
import sys
import json
import time
from pathlib import Path
from typing import Dict, List, Any


class DeploymentValidator:
    """Comprehensive deployment validation system."""

    def __init__(self):
        self.project_root = Path(__file__).parent.parent.parent
        self.results = {}
        self.start_time = time.time()

    def run_full_validation(self) -> Dict[str, Any]:
        """Run complete deployment validation."""
        print("🚀 STARTING FULL DEPLOYMENT VALIDATION")
        print("=" * 60)

        # Phase 1: Code Quality & Unit Tests
        print("\n📋 Phase 1: Code Quality & Unit Tests")
        self._run_unit_tests()

        # Phase 2: Competition Critical Tests
        print("\n🏆 Phase 2: Competition Critical Tests")
        self._run_competition_tests()

        # Phase 3: Integration Tests
        print("\n🔗 Phase 3: Integration Tests")
        self._run_integration_tests()

        # Phase 4: Performance Validation
        print("\n⚡ Phase 4: Performance Validation")
        self._run_performance_tests()

        # Phase 5: System Integration
        print("\n🔧 Phase 5: System Integration")
        self._run_system_integration()

        # Phase 6: Deployment Readiness
        print("\n🎯 Phase 6: Deployment Readiness")
        self._assess_deployment_readiness()

        # Generate final report
        final_report = self._generate_final_report()
        return final_report

    def _run_unit_tests(self):
        """Run unit tests with quality validation."""
        print("Running unit tests...")
        try:
            result = subprocess.run([
                sys.executable, '-m', 'pytest',
                'tests/unit/',
                '--tb=short',
                '--junitxml=test-results/unit-results.xml',
                '--json=test-results/unit-results.json'
            ], capture_output=True, text=True, cwd=self.project_root)

            # Parse results
            if result.returncode == 0:
                self.results['unit_tests'] = {'status': 'PASSED', 'details': self._parse_test_results('test-results/unit-results.json')}
                print("✅ Unit tests: PASSED")
            else:
                self.results['unit_tests'] = {'status': 'FAILED', 'output': result.stdout + result.stderr}
                print("❌ Unit tests: FAILED")

        except Exception as e:
            self.results['unit_tests'] = {'status': 'ERROR', 'error': str(e)}
            print(f"❌ Unit tests error: {e}")

    def _run_competition_tests(self):
        """Run competition-critical tests."""
        print("Running competition-critical tests...")
        try:
            result = subprocess.run([
                sys.executable, '-m', 'pytest',
                'tests/competition/',
                '--tb=short',
                '--junitxml=test-results/competition-results.xml',
                '--json=test-results/competition-results.json'
            ], capture_output=True, text=True, cwd=self.project_root)

            if result.returncode == 0:
                self.results['competition_tests'] = {'status': 'PASSED', 'details': self._parse_test_results('test-results/competition-results.json')}
                print("✅ Competition tests: PASSED")
            else:
                self.results['competition_tests'] = {'status': 'FAILED', 'output': result.stdout + result.stderr}
                print("❌ Competition tests: FAILED")

        except Exception as e:
            self.results['competition_tests'] = {'status': 'ERROR', 'error': str(e)}
            print(f"❌ Competition tests error: {e}")

    def _run_integration_tests(self):
        """Run integration tests."""
        print("Running integration tests...")
        try:
            result = subprocess.run([
                sys.executable, '-m', 'pytest',
                'tests/integration/',
                '--tb=short',
                '--junitxml=test-results/integration-results.xml',
                '--json=test-results/integration-results.json'
            ], capture_output=True, text=True, cwd=self.project_root)

            if result.returncode == 0:
                self.results['integration_tests'] = {'status': 'PASSED', 'details': self._parse_test_results('test-results/integration-results.json')}
                print("✅ Integration tests: PASSED")
            else:
                self.results['integration_tests'] = {'status': 'FAILED', 'output': result.stdout + result.stderr}
                print("❌ Integration tests: FAILED")

        except Exception as e:
            self.results['integration_tests'] = {'status': 'ERROR', 'error': str(e)}
            print(f"❌ Integration tests error: {e}")

    def _run_performance_tests(self):
        """Run performance tests with standards validation."""
        print("Running performance tests...")
        try:
            # Run benchmarks
            result = subprocess.run([
                sys.executable, '-m', 'pytest',
                'src/autonomy/core/state_management/tests/test_adaptive_performance.py',
                '--benchmark-only',
                '--benchmark-json=test-results/performance-results.json'
            ], capture_output=True, text=True, cwd=self.project_root)

            # Validate against standards
            standards_violations = self._validate_performance_standards('test-results/performance-results.json')

            if result.returncode == 0 and not standards_violations:
                self.results['performance_tests'] = {'status': 'PASSED', 'standards_check': 'PASSED'}
                print("✅ Performance tests: PASSED (meets standards)")
            elif standards_violations:
                self.results['performance_tests'] = {'status': 'FAILED', 'standards_violations': standards_violations}
                print("❌ Performance tests: FAILED (violates standards)")
            else:
                self.results['performance_tests'] = {'status': 'FAILED', 'output': result.stdout + result.stderr}
                print("❌ Performance tests: FAILED")

        except Exception as e:
            self.results['performance_tests'] = {'status': 'ERROR', 'error': str(e)}
            print(f"❌ Performance tests error: {e}")

    def _run_system_integration(self):
        """Run system integration validation."""
        print("Running system integration checks...")
        try:
            # Check configuration files
            config_check = self._validate_configuration()
            # Check deployment scripts
            deployment_check = self._validate_deployment_scripts()
            # Check ROS2 environment
            ros2_check = self._validate_ros2_environment()

            all_passed = all([config_check['passed'], deployment_check['passed'], ros2_check['passed']])

            self.results['system_integration'] = {
                'status': 'PASSED' if all_passed else 'FAILED',
                'config_check': config_check,
                'deployment_check': deployment_check,
                'ros2_check': ros2_check
            }

            if all_passed:
                print("✅ System integration: PASSED")
            else:
                print("❌ System integration: FAILED")

        except Exception as e:
            self.results['system_integration'] = {'status': 'ERROR', 'error': str(e)}
            print(f"❌ System integration error: {e}")

    def _assess_deployment_readiness(self):
        """Assess overall deployment readiness."""
        print("Assessing deployment readiness...")

        # Calculate readiness score
        readiness_score = self._calculate_readiness_score()

        # Determine competition readiness
        competition_ready = readiness_score >= 85

        self.results['deployment_readiness'] = {
            'readiness_score': readiness_score,
            'competition_ready': competition_ready,
            'recommendations': self._generate_readiness_recommendations(readiness_score)
        }

        if competition_ready:
            print(f"🎉 DEPLOYMENT READINESS: {readiness_score:.1f}% - COMPETITION READY!")
        else:
            print(f"⚠️ DEPLOYMENT READINESS: {readiness_score:.1f}% - NOT READY FOR COMPETITION")

    def _parse_test_results(self, results_file: str) -> Dict[str, Any]:
        """Parse pytest JSON results."""
        try:
            with open(self.project_root / results_file, 'r') as f:
                data = json.load(f)
            return {
                'tests_run': data.get('summary', {}).get('num_tests', 0),
                'passed': data.get('summary', {}).get('passed', 0),
                'failed': data.get('summary', {}).get('failed', 0),
                'duration': data.get('duration', 0)
            }
        except:
            return {'error': 'Could not parse results'}

    def _validate_performance_standards(self, results_file: str) -> List[str]:
        """Validate performance results against standards."""
        violations = []
        try:
            with open(self.project_root / results_file, 'r') as f:
                data = json.load(f)

            standards = {
                'test_context_evaluation_performance': {'max_time': 0.1, 'max_variability': 0.5},
                'test_policy_engine_performance': {'max_time': 0.05, 'max_variability': 0.5}
            }

            for benchmark in data.get('benchmarks', []):
                test_name = benchmark['name'].split('::')[-1]
                if test_name in standards:
                    std = standards[test_name]
                    mean_time = benchmark['stats']['mean']
                    variability = benchmark['stats']['stddev'] / benchmark['stats']['mean'] if benchmark['stats']['mean'] > 0 else 0

                    if mean_time > std['max_time']:
                        violations.append(f"{test_name}: mean time {mean_time:.3f}s exceeds {std['max_time']}s")
                    if variability > std['max_variability']:
                        violations.append(f"{test_name}: variability {variability:.2f} exceeds {std['max_variability']}")

        except Exception as e:
            violations.append(f"Performance validation error: {e}")

        return violations

    def _validate_configuration(self) -> Dict[str, Any]:
        """Validate configuration files."""
        config_files = ['config/rover.yaml', 'config/competition.yaml']
        missing_files = []
        invalid_files = []

        for config_file in config_files:
            config_path = self.project_root / config_file
            if not config_path.exists():
                missing_files.append(config_file)
            else:
                try:
                    import yaml
                    with open(config_path, 'r') as f:
                        yaml.safe_load(f)
                except Exception as e:
                    invalid_files.append(f"{config_file}: {e}")

        return {
            'passed': len(missing_files) == 0 and len(invalid_files) == 0,
            'missing_files': missing_files,
            'invalid_files': invalid_files
        }

    def _validate_deployment_scripts(self) -> Dict[str, Any]:
        """Validate deployment scripts exist."""
        required_scripts = [
            'scripts/deployment/automated_deployment.py',
            'scripts/competition/pre_competition_checklist.py',
            'scripts/competition/competition_telemetry.py',
            'scripts/monitoring/service_health_monitor.py'
        ]

        missing_scripts = []
        for script in required_scripts:
            if not (self.project_root / script).exists():
                missing_scripts.append(script)

        return {
            'passed': len(missing_scripts) == 0,
            'missing_scripts': missing_scripts
        }

    def _validate_ros2_environment(self) -> Dict[str, Any]:
        """Validate ROS2 environment."""
        try:
            import rclpy
            ros2_available = True
            ros2_ok = rclpy.ok()
        except ImportError:
            ros2_available = False
            ros2_ok = False

        return {
            'passed': ros2_available and ros2_ok,
            'ros2_available': ros2_available,
            'ros2_initialized': ros2_ok
        }

    def _calculate_readiness_score(self) -> float:
        """Calculate overall readiness score (0-100)."""
        scores = []

        # Unit tests: 20% weight
        if self.results.get('unit_tests', {}).get('status') == 'PASSED':
            unit_score = 100
        else:
            unit_score = 0
        scores.append(('unit_tests', unit_score, 20))

        # Competition tests: 30% weight (most critical)
        if self.results.get('competition_tests', {}).get('status') == 'PASSED':
            comp_score = 100
        else:
            comp_score = 0
        scores.append(('competition_tests', comp_score, 30))

        # Integration tests: 25% weight
        if self.results.get('integration_tests', {}).get('status') == 'PASSED':
            int_score = 100
        else:
            int_score = 0
        scores.append(('integration_tests', int_score, 25))

        # Performance tests: 15% weight
        perf_result = self.results.get('performance_tests', {})
        if perf_result.get('status') == 'PASSED' and perf_result.get('standards_check') == 'PASSED':
            perf_score = 100
        else:
            perf_score = 0
        scores.append(('performance_tests', perf_score, 15))

        # System integration: 10% weight
        if self.results.get('system_integration', {}).get('status') == 'PASSED':
            sys_score = 100
        else:
            sys_score = 0
        scores.append(('system_integration', sys_score, 10))

        # Calculate weighted average
        total_score = sum(score * weight for _, score, weight in scores)
        return min(100.0, max(0.0, total_score))

    def _generate_readiness_recommendations(self, score: float) -> List[str]:
        """Generate recommendations based on readiness score."""
        recommendations = []

        if score < 50:
            recommendations.append("CRITICAL: Major test failures detected. Address all failing tests before deployment.")
        elif score < 75:
            recommendations.append("HIGH PRIORITY: Several test categories failing. Focus on competition-critical tests.")
        elif score < 85:
            recommendations.append("MODERATE: Most tests passing but some issues remain. Verify performance standards.")
        else:
            recommendations.append("EXCELLENT: All systems validated. Ready for competition deployment!")

        # Specific recommendations based on failures
        failed_components = []
        for component, result in self.results.items():
            if result.get('status') != 'PASSED':
                failed_components.append(component.replace('_', ' ').title())

        if failed_components:
            recommendations.append(f"Failed components: {', '.join(failed_components)}")

        return recommendations

    def _generate_final_report(self) -> Dict[str, Any]:
        """Generate comprehensive final report."""
        total_duration = time.time() - self.start_time

        report = {
            'validation_summary': {
                'total_duration_seconds': total_duration,
                'execution_timestamp': time.time(),
                'overall_readiness_score': self.results.get('deployment_readiness', {}).get('readiness_score', 0),
                'competition_ready': self.results.get('deployment_readiness', {}).get('competition_ready', False)
            },
            'test_results': self.results,
            'recommendations': self.results.get('deployment_readiness', {}).get('recommendations', [])
        }

        # Save report
        report_file = self.project_root / 'deployment_validation_report.json'
        with open(report_file, 'w') as f:
            json.dump(report, f, indent=2)

        return report


def main():
    """Main entry point."""
    validator = DeploymentValidator()
    report = validator.run_full_validation()

    # Print summary
    print("\n" + "=" * 60)
    print("DEPLOYMENT VALIDATION COMPLETE")
    print("=" * 60)
    print(".1f")
    print(f"Competition Ready: {'YES' if report['validation_summary']['competition_ready'] else 'NO'}")
    print(".2f")

    if report['recommendations']:
        print("\n📋 RECOMMENDATIONS:")
        for rec in report['recommendations']:
            print(f"  • {rec}")

    # Exit with appropriate code
    if report['validation_summary']['competition_ready']:
        print("\n🎉 SYSTEM IS READY FOR URC 2026 COMPETITION DEPLOYMENT!")
        sys.exit(0)
    else:
        print("\n⚠️ SYSTEM REQUIRES ADDITIONAL VALIDATION BEFORE COMPETITION")
        sys.exit(1)


if __name__ == '__main__':
    main()
