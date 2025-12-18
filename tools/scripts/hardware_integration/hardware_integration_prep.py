#!/usr/bin/env python3
"""
Hardware Integration Preparation Script

Comprehensive script that demonstrates and executes all software preparations
needed for hardware testing, including configuration validation, calibration,
performance monitoring, and migration management.

Author: URC 2026 Autonomy Team
"""

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import Any, Dict, List

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


def main():
    """Main entry point for hardware integration preparation."""
    parser = argparse.ArgumentParser(
        description="URC 2026 Hardware Integration Preparation"
    )
    parser.add_argument(
        "--action",
        choices=["validate", "calibrate", "migrate", "monitor", "full_prep"],
        default="full_prep",
        help="Action to perform",
    )
    parser.add_argument(
        "--config-dir", default="config", help="Configuration directory"
    )
    parser.add_argument(
        "--output-dir",
        default="hardware_prep_output",
        help="Output directory for results",
    )
    parser.add_argument(
        "--force", action="store_true", help="Force re-execution of steps"
    )

    args = parser.parse_args()

    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(exist_ok=True)

    logger.info("[IGNITE] URC 2026 Hardware Integration Preparation")
    logger.info("=" * 50)

    success = True

    try:
        if args.action == "validate":
            success = run_configuration_validation(args.config_dir, output_dir)
        elif args.action == "calibrate":
            success = run_calibration_procedures(output_dir, args.force)
        elif args.action == "migrate":
            success = run_migration_demo(output_dir)
        elif args.action == "monitor":
            success = run_performance_monitoring_demo()
        elif args.action == "full_prep":
            success = run_full_preparation(args.config_dir, output_dir, args.force)

    except Exception as e:
        logger.error(f"[FAIL] Preparation failed with exception: {e}")
        success = False

    if success:
        logger.info("[PASS] Hardware integration preparation completed successfully!")
        logger.info(f" Results saved to: {output_dir.absolute()}")
    else:
        logger.error("[FAIL] Hardware integration preparation failed!")
        sys.exit(1)


def run_configuration_validation(config_dir: str, output_dir: Path) -> bool:
    """Run configuration validation."""
    logger.info("[MAGNIFY] Running Configuration Validation...")

    try:
        from config.config_validator import ConfigurationValidator, validate_all_configs

        # Validate all configurations
        success = validate_all_configs(config_dir)

        # Generate default configs if needed
        if not success:
            logger.info("[TOOL] Generating default configurations...")
            validator = ConfigurationValidator(config_dir)
            configs = validator.generate_default_configs(str(output_dir / "configs"))

            # Re-validate
            success = validate_all_configs(str(output_dir / "configs"))

        # Save validation results
        results_file = output_dir / "configuration_validation_results.json"
        validator = ConfigurationValidator(config_dir)
        validation_results = validator.validate_directory(config_dir)

        import json

        with open(results_file, "w") as f:
            json.dump(
                {
                    "timestamp": time.time(),
                    "config_dir": config_dir,
                    "validation_results": validation_results,
                    "overall_success": success,
                },
                f,
                indent=2,
            )

        logger.info(f" Validation results saved to: {results_file}")
        return success

    except Exception as e:
        logger.error(f"Configuration validation failed: {e}")
        return False


def run_calibration_procedures(output_dir: Path, force_redo: bool = False) -> bool:
    """Run calibration procedures."""
    logger.info("[TOOL] Running Calibration Procedures...")

    try:
        from calibration.calibration_workflow import (
            CalibrationType,
            CalibrationWorkflow,
            get_calibration_status,
        )

        # Initialize calibration workflow
        workflow = CalibrationWorkflow(str(output_dir / "calibration"))

        # Run key calibrations
        calibrations_to_run = [
            CalibrationType.CAMERA_INTRINSICS,
            CalibrationType.ARM_KINEMATICS,
            CalibrationType.IMU_ALIGNMENT,
            CalibrationType.GPS_OFFSET,
        ]

        results = {}
        overall_success = True

        for cal_type in calibrations_to_run:
            logger.info(f"Running {cal_type.value} calibration...")
            result = workflow.run_calibration(cal_type, force_redo)
            results[cal_type.value] = {
                "success": result.success,
                "quality_score": result.quality_score,
                "duration": result.duration,
                "timestamp": result.timestamp,
            }

            if not result.success:
                overall_success = False
                logger.warning(f"Calibration {cal_type.value} failed")
            else:
                logger.info(".2f")

        # Get final status
        final_status = get_calibration_status()

        # Save results
        import json

        results_file = output_dir / "calibration_results.json"
        with open(results_file, "w") as f:
            json.dump(
                {
                    "timestamp": time.time(),
                    "calibration_results": results,
                    "final_status": {k.value: v for k, v in final_status.items()},
                    "overall_success": overall_success,
                },
                f,
                indent=2,
            )

        logger.info(f" Calibration results saved to: {results_file}")
        return overall_success

    except Exception as e:
        logger.error(f"Calibration procedures failed: {e}")
        return False


def run_migration_demo(output_dir: Path) -> bool:
    """Demonstrate migration capabilities."""
    logger.info("[REFRESH] Running Migration Demonstration...")

    try:
        from migration.migration_manager import (
            MigrationManager,
            MigrationPhase,
            migrate_to_phase,
        )

        # Initialize migration manager
        manager = MigrationManager()

        # Demonstrate phased migration
        phases_to_demo = [
            MigrationPhase.MOCK_ONLY,
            MigrationPhase.CAN_BUS_INTEGRATION,
            MigrationPhase.DRIVE_SYSTEM_INTEGRATION,
            MigrationPhase.ARM_INTEGRATION,
        ]

        migration_results = {}

        for phase in phases_to_demo:
            logger.info(f"Migrating to phase: {phase.value}")
            result = migrate_to_phase(phase)

            migration_results[phase.value] = {
                "success": result.success,
                "duration": result.duration,
                "timestamp": time.time(),
            }

            if not result.success:
                logger.error(
                    f"Migration to {phase.value} failed: {result.error_message}"
                )
                break

            # Get component states
            states = manager.get_component_states()
            migration_results[phase.value]["component_states"] = states

            logger.info(f"[PASS] Successfully migrated to {phase.value}")

        # Save results
        import json

        results_file = output_dir / "migration_demo_results.json"
        with open(results_file, "w") as f:
            json.dump(
                {
                    "timestamp": time.time(),
                    "migration_results": migration_results,
                    "final_phase": (
                        manager.get_current_phase().value
                        if manager.get_current_phase()
                        else None
                    ),
                },
                f,
                indent=2,
            )

        logger.info(f" Migration demo results saved to: {results_file}")
        return all(r["success"] for r in migration_results.values())

    except Exception as e:
        logger.error(f"Migration demonstration failed: {e}")
        return False


def run_performance_monitoring_demo() -> bool:
    """Demonstrate performance monitoring capabilities."""
    logger.info("[GRAPH] Running Performance Monitoring Demonstration...")

    try:
        from monitoring.real_time_monitor import (
            RealTimeMonitor,
            get_performance_summary,
            record_loop_end,
            record_loop_start,
            start_monitoring,
        )
        from monitoring.telemetry_collector import TelemetryCollector
        from monitoring.telemetry_collector import (
            get_performance_summary as get_telemetry_summary,
        )
        from monitoring.telemetry_collector import record_event, record_metric

        # Initialize monitoring systems
        monitor = RealTimeMonitor()
        collector = TelemetryCollector()

        # Start monitoring
        monitor.start_monitoring()
        collector.start_auto_flush("performance_demo_logs")

        # Register a custom control loop
        monitor.register_control_loop("demo_loop", target_period=0.1, deadline=0.15)

        # Simulate control loop execution
        logger.info("Simulating control loop execution...")
        for i in range(50):
            # Record telemetry
            record_metric("demo_system", "iteration", i)
            record_metric("demo_system", "load_factor", 0.5 + 0.3 * (i % 10) / 10.0)

            if i % 10 == 0:
                record_event("demo_system", "checkpoint", f"Reached iteration {i}")

            # Simulate control loop
            start_time = record_loop_start("demo_loop")
            time.sleep(0.08 + 0.02 * (i % 5) / 5.0)  # Variable execution time
            record_loop_end("demo_loop", start_time)

            time.sleep(0.02)  # Simulate other processing

        # Get performance summaries
        perf_summary = get_performance_summary()
        telemetry_summary = get_telemetry_summary()

        # Stop monitoring
        monitor.stop_monitoring()
        collector.stop_auto_flush()

        # Log results
        logger.info("Performance monitoring results:")
        logger.info(
            ".1f" f"{perf_summary['system_health']['overall_health_score']:.1f}%"
        )

        for loop_name, metrics in perf_summary["control_loops"].items():
            logger.info(
                f"  {loop_name}: {metrics['success_rate_percent']:.1f}% success, "
                f"{metrics['jitter_percent']:.1f}% jitter"
            )

        logger.info(
            f"Telemetry collected: {telemetry_summary['events_summary']['total_events']} events, "
            f"{len(telemetry_summary['metrics_summary'])} metrics"
        )

        return True

    except Exception as e:
        logger.error(f"Performance monitoring demonstration failed: {e}")
        return False


def run_full_preparation(
    config_dir: str, output_dir: Path, force_redo: bool = False
) -> bool:
    """Run complete hardware integration preparation."""
    logger.info("[OBJECTIVE] Running Full Hardware Integration Preparation...")

    preparation_steps = [
        (
            "Configuration Validation",
            lambda: run_configuration_validation(config_dir, output_dir),
        ),
        (
            "Calibration Procedures",
            lambda: run_calibration_procedures(output_dir, force_redo),
        ),
        ("Migration Demonstration", lambda: run_migration_demo(output_dir)),
        ("Performance Monitoring", run_performance_monitoring_demo),
    ]

    results = {}
    overall_success = True

    for step_name, step_func in preparation_steps:
        logger.info(f"\n  Starting: {step_name}")
        start_time = time.time()

        try:
            success = step_func()
            duration = time.time() - start_time

            results[step_name] = {
                "success": success,
                "duration": duration,
                "timestamp": time.time(),
            }

            if success:
                logger.info(
                    f"[PASS] {step_name} completed successfully in {duration:.1f}s"
                )
            else:
                logger.error(f"[FAIL] {step_name} failed after {duration:.1f}s")
                overall_success = False

        except Exception as e:
            duration = time.time() - start_time
            logger.error(f" {step_name} crashed after {duration:.1f}s: {e}")
            results[step_name] = {
                "success": False,
                "duration": duration,
                "error": str(e),
                "timestamp": time.time(),
            }
            overall_success = False

    # Generate comprehensive report
    generate_preparation_report(results, output_dir)

    return overall_success


def generate_preparation_report(results: Dict[str, Any], output_dir: Path):
    """Generate comprehensive preparation report."""
    import json

    # Calculate summary statistics
    total_steps = len(results)
    successful_steps = sum(1 for r in results.values() if r["success"])
    total_duration = sum(r["duration"] for r in results.values())

    report = {
        "report_type": "hardware_integration_preparation",
        "timestamp": time.time(),
        "summary": {
            "total_steps": total_steps,
            "successful_steps": successful_steps,
            "failed_steps": total_steps - successful_steps,
            "success_rate": (
                successful_steps / total_steps * 100.0 if total_steps > 0 else 0.0
            ),
            "total_duration": total_duration,
            "average_step_duration": (
                total_duration / total_steps if total_steps > 0 else 0.0
            ),
        },
        "step_results": results,
        "recommendations": generate_recommendations(results),
    }

    # Save report
    report_file = output_dir / "hardware_integration_preparation_report.json"
    with open(report_file, "w") as f:
        json.dump(report, f, indent=2)

    # Print summary
    logger.info("\n[GRAPH] Hardware Integration Preparation Summary:")
    logger.info("=" * 50)
    logger.info(".1f")
    logger.info(".1f")
    logger.info(".1f")
    for step_name, result in results.items():
        status = "[PASS]" if result["success"] else "[FAIL]"
        logger.info(f"  {status} {step_name}: {result['duration']:.1f}s")

    recommendations = report["recommendations"]
    if recommendations:
        logger.info("\n Recommendations:")
        for rec in recommendations:
            logger.info(f"  • {rec}")

    logger.info(f"\n Detailed report saved to: {report_file}")


def generate_recommendations(results: Dict[str, Any]) -> List[str]:
    """Generate recommendations based on preparation results."""
    recommendations = []

    # Check for failed steps
    failed_steps = [name for name, result in results.items() if not result["success"]]
    if failed_steps:
        recommendations.append(f"Re-run failed steps: {', '.join(failed_steps)}")

    # Check calibration quality
    if (
        "Calibration Procedures" in results
        and results["Calibration Procedures"]["success"]
    ):
        recommendations.append(
            "Review calibration quality scores and re-run if below 0.8"
        )

    # Check migration
    if (
        "Migration Demonstration" in results
        and results["Migration Demonstration"]["success"]
    ):
        recommendations.append(
            "Test migration rollback capabilities in a safe environment"
        )

    # General recommendations
    recommendations.extend(
        [
            "Set up automated nightly runs of configuration validation",
            "Establish calibration quality monitoring and alerts",
            "Implement performance regression testing in CI/CD",
            "Document hardware-specific configuration parameters",
            "Create hardware health monitoring dashboards",
        ]
    )

    return recommendations


if __name__ == "__main__":
    main()
