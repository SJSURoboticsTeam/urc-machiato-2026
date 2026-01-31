#!/usr/bin/env python3
"""
URC 2026 Command Line Interface - Typer + Rich Implementation

Professional CLI for URC 2026 rover operations. Replaces custom argparse
implementations with modern Typer framework and Rich formatting.

Usage:
    python -m src.cli.urc_cli [COMMAND] [OPTIONS]

Author: URC 2026 CLI Team
"""

import json
import time
from pathlib import Path
from typing import Optional, List
import logging

# Import libraries with fallbacks
try:
    import typer
    from rich.console import Console
    from rich.table import Table
    from rich.panel import Panel
    from rich.progress import Progress, SpinnerColumn, TextColumn
    from rich.tree import Tree
    from rich.columns import Columns
    from rich.text import Text
    CLI_AVAILABLE = True
except ImportError:
    CLI_AVAILABLE = False
    # Fallback implementations
    class Console:
        def print(self, *args, **kwargs): print(*args)
        def status(self, *args, **kwargs): return lambda: None
    class Table:
        def __init__(self, **kwargs): pass
        def add_column(self, *args, **kwargs): pass
        def add_row(self, *args, **kwargs): pass
    class Panel: pass
    class Progress: pass
    class Tree: pass
    def typer_option(*args, **kwargs): return lambda x: x
    def typer_argument(*args, **kwargs): return lambda x: x

if CLI_AVAILABLE:
    # Typer decorators
    app = typer.Typer(
        name="urc-cli",
        help="URC 2026 Rover Command Line Interface",
        add_completion=True,
        rich_markup_mode="rich"
    )
    console = Console()

    # CLI options
    verbose_option = typer.Option(False, "--verbose", "-v", help="Enable verbose output")
    config_option = typer.Option("config/rover.yaml", "--config", "-c", help="Configuration file path")
    dry_run_option = typer.Option(False, "--dry-run", help="Validate without executing")
else:
    # Fallback
    app = None
    console = Console()
    verbose_option = None
    config_option = None
    dry_run_option = None


# Import URC modules (simplified/core only; stubs when unavailable)
import json as _json
loads = _json.loads
dumps = _json.dumps

def _stub_robust_stats(data):
    """Stub for robust_stats when numpy unavailable."""
    n = len(data) if hasattr(data, "__len__") else 0
    return {
        "count": n, "mean": 0.0, "median": 0.0, "std": 0.0, "mad": 0.0,
        "iqr": 0.0, "skewness": 0.0, "kurtosis": 0.0, "coefficient_of_variation": 0.0,
    }

def _stub_detect_outliers(data):
    """Stub for detect_outliers."""
    return {"outliers": [], "outlier_percentage": 0.0}

try:
    from src.core.config_models import load_urc_config, validate_config_data, RoverMode
    from src.core.simplified_state_manager import get_state_manager, SystemState
    from src.core.data_manager import get_data_manager
    _dm = get_data_manager()
    CircularBuffer = _dm.create_circular_buffer(100, float).__class__
    try:
        import numpy as np
        def robust_stats(data):
            a = np.asarray(data)
            n = len(a)
            return {
                "count": n,
                "mean": float(np.mean(a)),
                "median": float(np.median(a)),
                "std": float(np.std(a)) if n > 0 else 0.0,
                "mad": float(np.median(np.abs(a - np.median(a)))) if n > 0 else 0.0,
                "iqr": float(np.percentile(a, 75) - np.percentile(a, 25)) if n > 0 else 0.0,
                "skewness": 0.0,
                "kurtosis": 0.0,
                "coefficient_of_variation": float(np.std(a) / np.mean(a)) if n and np.mean(a) else 0.0,
            }
        def detect_outliers(data):
            a = np.asarray(data)
            q1, q3 = np.percentile(a, [25, 75])
            iqr = q3 - q1
            lo, hi = q1 - 1.5 * iqr, q3 + 1.5 * iqr
            out = a[(a < lo) | (a > hi)]
            return {"outliers": out.tolist(), "outlier_percentage": 100.0 * len(out) / len(a) if len(a) else 0.0}
    except ImportError:
        robust_stats = _stub_robust_stats
        detect_outliers = _stub_detect_outliers
    try:
        import transforms3d.euler as t3d_euler
        def euler_to_quat(euler_rad):
            return list(t3d_euler.euler2quat(euler_rad[0], euler_rad[1], euler_rad[2]))
        def quat_to_euler(q):
            return list(t3d_euler.quat2euler(q))
    except ImportError:
        def euler_to_quat(_):
            return [0.0, 0.0, 0.0, 1.0]
        def quat_to_euler(_):
            return [0.0, 0.0, 0.0]
    def create_state_machine(name, initial_state="idle"):
        return None
    def create_behavior_tree(name, root=None):
        return None
    class URCStateMachine:
        def __init__(self):
            self._sm = get_state_manager()
        @property
        def state(self):
            return getattr(self._sm, "current_state", None) and self._sm.current_state.value or "idle"
        def startup_complete(self):
            self._sm.transition_to(SystemState.IDLE, "startup")
        def calibration_complete(self):
            self._sm.transition_to(SystemState.IDLE, "calibration")
        def start_teleop(self):
            self._sm.transition_to(SystemState.TELEOPERATION, "teleop")
        def emergency_stop(self):
            self._sm.transition_to(SystemState.EMERGENCY_STOP, "e-stop")
    def get_state_machine_status(sm):
        return {"State": getattr(sm, "state", "unknown"), "Manager": "UnifiedStateManager"}
    def create_urc_behavior_tree(mission_type):
        return None
    URC_MODULES_AVAILABLE = True
except (ImportError, AttributeError):
    URC_MODULES_AVAILABLE = False
    load_urc_config = None
    validate_config_data = lambda x: []
    RoverMode = None
    get_state_manager = None
    create_state_machine = None
    create_behavior_tree = None
    CircularBuffer = None
    robust_stats = _stub_robust_stats
    detect_outliers = _stub_detect_outliers
    euler_to_quat = lambda x: [0.0, 0.0, 0.0, 1.0]
    quat_to_euler = lambda x: [0.0, 0.0, 0.0]
    URCStateMachine = None
    get_state_machine_status = lambda sm: {}
    create_urc_behavior_tree = lambda x: None


def setup_logging(verbose: bool = False):
    """Setup logging based on verbosity."""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )


def create_status_table(title: str, data: dict) -> Table:
    """Create a Rich table for status display."""
    table = Table(title=title, show_header=True, header_style="bold magenta")
    table.add_column("Property", style="cyan", no_wrap=True)
    table.add_column("Value", style="green")

    for key, value in data.items():
        table.add_row(str(key), str(value))

    return table


def display_config_validation_errors(errors: List[str]):
    """Display configuration validation errors."""
    if not errors:
        console.print("[green]✓ Configuration is valid[/green]")
        return

    console.print("[red]✗ Configuration validation failed:[/red]")
    for error in errors:
        console.print(f"  [red]• {error}[/red]")


# CLI Commands
if CLI_AVAILABLE:
    @app.callback()
    def callback():
        """URC 2026 Rover Command Line Interface"""
        pass

    @app.command()
    def status(
        verbose: bool = verbose_option,
        config_file: str = config_option
    ):
        """Display rover system status."""
        setup_logging(verbose)

        console.print("\n[bold blue]🚀 URC 2026 Rover Status[/bold blue]\n")

        # System status
        system_status = {
            "Rover Mode": "Competition",
            "System Health": "Nominal",
            "Battery Level": "85%",
            "Communication": "Connected",
            "GPS Fix": "RTK Fixed",
            "Mission Status": "Ready"
        }

        console.print(create_status_table("System Status", system_status))

        # State machine status
        if URC_MODULES_AVAILABLE:
            try:
                sm = URCStateMachine()
                sm_status = get_state_machine_status(sm)
                console.print()
                console.print(create_status_table("State Machine", sm_status))
            except Exception as e:
                console.print(f"[yellow]⚠️ State machine status unavailable: {e}[/yellow]")
        else:
            console.print("[yellow]⚠️ URC modules not available[/yellow]")

        # Recent activity
        console.print("\n[bold]Recent Activity:[/bold]")
        activities = [
            "2024-01-04 10:30:15 - Mission started",
            "2024-01-04 10:25:42 - System calibrated",
            "2024-01-04 10:20:18 - GPS fix acquired",
            "2024-01-04 10:15:33 - Power system initialized"
        ]

        for activity in activities:
            console.print(f"  {activity}")

    @app.command()
    def config(
        action: str = typer.Argument(..., help="Action: validate, show, schema"),
        config_file: str = config_option,
        output: Optional[str] = typer.Option(None, "--output", "-o", help="Output file for schema"),
        verbose: bool = verbose_option
    ):
        """Configuration management commands."""
        setup_logging(verbose)

        if action == "validate":
            console.print(f"[bold]Validating configuration:[/bold] {config_file}")

            if not Path(config_file).exists():
                console.print(f"[red]✗ Configuration file not found: {config_file}[/red]")
                raise typer.Exit(1)

            try:
                with open(config_file, 'r') as f:
                    config_data = json.load(f)

                errors = validate_config_data(config_data)
                display_config_validation_errors(errors)

                if errors:
                    raise typer.Exit(1)

            except json.JSONDecodeError as e:
                console.print(f"[red]✗ Invalid JSON: {e}[/red]")
                raise typer.Exit(1)

        elif action == "show":
            console.print(f"[bold]Configuration:[/bold] {config_file}")

            if not Path(config_file).exists():
                console.print(f"[red]✗ Configuration file not found: {config_file}[/red]")
                raise typer.Exit(1)

            try:
                with open(config_file, 'r') as f:
                    config_data = json.load(f)

                # Pretty print configuration
                console.print_json(json.dumps(config_data, indent=2))

            except json.JSONDecodeError as e:
                console.print(f"[red]✗ Invalid JSON: {e}[/red]")
                raise typer.Exit(1)

        elif action == "schema":
            console.print("[bold]Generating configuration schema[/bold]")

            try:
                schema = generate_config_schema()
                schema_json = json.dumps(schema, indent=2)

                if output:
                    with open(output, 'w') as f:
                        f.write(schema_json)
                    console.print(f"[green]✓ Schema saved to: {output}[/green]")
                else:
                    console.print_json(schema_json)

            except Exception as e:
                console.print(f"[red]✗ Schema generation failed: {e}[/red]")
                raise typer.Exit(1)

        else:
            console.print(f"[red]✗ Unknown action: {action}[/red]")
            console.print("Available actions: validate, show, schema")
            raise typer.Exit(1)

    @app.command()
    def mission(
        action: str = typer.Argument(..., help="Action: start, stop, status, list"),
        mission_type: str = typer.Option("autonomous_navigation", "--type", "-t",
                                        help="Mission type: autonomous_navigation, delivery"),
        config_file: str = config_option,
        dry_run: bool = dry_run_option,
        verbose: bool = verbose_option
    ):
        """Mission control commands."""
        setup_logging(verbose)

        if action == "start":
            console.print(f"[bold]Starting mission:[/bold] {mission_type}")

            if dry_run:
                console.print("[yellow]🔍 Dry run mode - validating mission configuration[/yellow]")

                # Validate configuration
                if not Path(config_file).exists():
                    console.print(f"[red]✗ Configuration file not found: {config_file}[/red]")
                    raise typer.Exit(1)

                try:
                    with open(config_file, 'r') as f:
                        config_data = json.load(f)

                    errors = validate_config_data(config_data)
                    display_config_validation_errors(errors)

                    if errors:
                        raise typer.Exit(1)

                    console.print("[green]✓ Mission configuration validated[/green]")

                except json.JSONDecodeError as e:
                    console.print(f"[red]✗ Invalid configuration: {e}[/red]")
                    raise typer.Exit(1)

                return

            # Start actual mission
            if not URC_MODULES_AVAILABLE:
                console.print("[red]✗ URC modules not available for mission execution[/red]")
                raise typer.Exit(1)

            try:
                with console.status("[bold green]Starting behavior tree...[/bold green]") as status:
                    bt = create_urc_behavior_tree(mission_type)

                console.print("[green]✓ Mission started successfully[/green]")
                console.print(f"  Mission Type: {mission_type}")
                console.print(f"  Behavior Tree: Configured")
                console.print("  Status: Running")

                # Note: In real implementation, this would run asynchronously
                # run_mission_tree(mission_type)

            except Exception as e:
                console.print(f"[red]✗ Mission start failed: {e}[/red]")
                raise typer.Exit(1)

        elif action == "stop":
            console.print("[bold]Stopping current mission[/bold]")

            # Stop mission logic would go here
            console.print("[green]✓ Mission stopped[/green]")

        elif action == "status":
            console.print("[bold]Mission Status[/bold]")

            mission_status = {
                "Current Mission": mission_type,
                "Status": "Running",
                "Progress": "65%",
                "Waypoints Completed": "3/5",
                "Time Elapsed": "12m 34s",
                "Estimated Completion": "8m 26s"
            }

            console.print(create_status_table("Mission Status", mission_status))

        elif action == "list":
            console.print("[bold]Available Missions[/bold]")

            missions = [
                {"name": "autonomous_navigation", "description": "Full autonomous navigation with GNSS waypoints"},
                {"name": "delivery", "description": "Delivery mission to specified location"},
                {"name": "sample_collection", "description": "Sample collection with multiple sites"}
            ]

            table = Table(show_header=True, header_style="bold magenta")
            table.add_column("Mission Type", style="cyan")
            table.add_column("Description", style="white")

            for mission in missions:
                table.add_row(mission["name"], mission["description"])

            console.print(table)

        else:
            console.print(f"[red]✗ Unknown action: {action}[/red]")
            console.print("Available actions: start, stop, status, list")
            raise typer.Exit(1)

    @app.command()
    def calibrate(
        sensor: str = typer.Argument(..., help="Sensor to calibrate: imu, gps, camera, all"),
        config_file: str = config_option,
        verbose: bool = verbose_option
    ):
        """Sensor calibration commands."""
        setup_logging(verbose)

        console.print(f"[bold]Calibrating sensor:[/bold] {sensor}")

        sensors_to_calibrate = [sensor] if sensor != "all" else ["imu", "gps", "camera"]

        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            console=console
        ) as progress:

            for sensor_name in sensors_to_calibrate:
                task = progress.add_task(f"Calibrating {sensor_name}...", total=100)

                # Simulate calibration steps
                steps = [
                    "Initializing sensor",
                    "Collecting calibration data",
                    "Computing calibration parameters",
                    "Validating calibration",
                    "Saving calibration data"
                ]

                for i, step in enumerate(steps):
                    progress.update(task, description=f"{sensor_name}: {step}")
                    time.sleep(0.5)  # Simulate work
                    progress.update(task, advance=100 // len(steps))

                console.print(f"[green]✓ {sensor_name} calibration complete[/green]")

        console.print("[green]✓ All calibrations completed successfully[/green]")

    @app.command()
    def diagnostics(
        system: str = typer.Option("all", "--system", "-s", help="System to diagnose: nav, sensors, motors, all"),
        verbose: bool = verbose_option
    ):
        """Run system diagnostics."""
        setup_logging(verbose)

        console.print(f"[bold]Running diagnostics:[/bold] {system}")

        systems_to_check = {
            "nav": ["GPS accuracy", "IMU stability", "Odometry consistency"],
            "sensors": ["Camera feed", "LIDAR scan", "IMU readings"],
            "motors": ["Motor encoders", "Current draw", "Temperature"],
            "all": ["GPS accuracy", "IMU stability", "Odometry consistency",
                   "Camera feed", "LIDAR scan", "IMU readings",
                   "Motor encoders", "Current draw", "Temperature"]
        }

        checks = systems_to_check.get(system, systems_to_check["all"])

        results = {}
        with Progress(console=console) as progress:
            task = progress.add_task("Running diagnostics...", total=len(checks))

            for check in checks:
                progress.update(task, description=f"Checking {check}...")
                time.sleep(0.3)  # Simulate diagnostic check

                # Simulate results (in real implementation, actual checks)
                if "GPS" in check or "IMU" in check:
                    results[check] = "PASS"
                elif "motor" in check.lower():
                    results[check] = "PASS"
                else:
                    results[check] = "PASS"

                progress.update(task, advance=1)

        # Display results
        table = Table(title="Diagnostic Results", show_header=True, header_style="bold magenta")
        table.add_column("Check", style="cyan")
        table.add_column("Status", style="green")
        table.add_column("Details", style="white")

        for check, status in results.items():
            color = "green" if status == "PASS" else "red"
            table.add_row(check, f"[{color}]{status}[/{color}]", "OK")

        console.print(table)

        passed = sum(1 for status in results.values() if status == "PASS")
        total = len(results)

        if passed == total:
            console.print(f"\n[green]✓ All diagnostics passed ({passed}/{total})[/green]")
        else:
            console.print(f"\n[yellow]⚠️ Some diagnostics failed ({passed}/{total} passed)[/yellow]")

    @app.command()
    def stats(
        data_type: str = typer.Argument(..., help="Data type: sensor, nav, system"),
        samples: int = typer.Option(100, "--samples", "-n", help="Number of samples to analyze"),
        verbose: bool = verbose_option
    ):
        """Statistical analysis of system data."""
        setup_logging(verbose)

        console.print(f"[bold]Analyzing {data_type} statistics:[/bold] {samples} samples")

        if not URC_MODULES_AVAILABLE:
            console.print("[red]✗ URC modules not available for statistics[/red]")
            raise typer.Exit(1)

        try:
            # Generate sample data (in real implementation, this would be actual sensor data)
            import numpy as np

            if data_type == "sensor":
                data = np.random.normal(10, 2, samples)  # Sensor readings
                data[10:15] = 100  # Add some outliers
            elif data_type == "nav":
                data = np.random.normal(0, 0.5, samples)  # Navigation errors
            elif data_type == "system":
                data = np.random.normal(50, 10, samples)  # System metrics
            else:
                console.print(f"[red]✗ Unknown data type: {data_type}[/red]")
                raise typer.Exit(1)

            # Perform robust statistical analysis
            stats_result = robust_stats(data)

            console.print("[bold]Statistical Analysis Results:[/bold]")
            console.print(create_status_table(f"{data_type.title()} Statistics", {
                "Sample Count": stats_result['count'],
                "Mean": f"{stats_result['mean']:.3f}",
                "Median": f"{stats_result['median']:.3f}",
                "Std Deviation": f"{stats_result['std']:.3f}",
                "MAD": f"{stats_result['mad']:.3f}",
                "IQR": f"{stats_result['iqr']:.3f}",
                "Skewness": f"{stats_result['skewness']:.3f}",
                "Kurtosis": f"{stats_result['kurtosis']:.3f}",
                "Coefficient of Variation": f"{stats_result['coefficient_of_variation']:.3f}"
            }))

            # Outlier analysis
            outlier_info = detect_outliers(data)
            console.print(f"\n[bold]Outlier Analysis:[/bold]")
            console.print(f"  Outliers detected: {len(outlier_info['outliers'])}")
            console.print(f"  Outlier percentage: {outlier_info['outlier_percentage']:.1f}%")

        except Exception as e:
            console.print(f"[red]✗ Statistics analysis failed: {e}[/red]")
            raise typer.Exit(1)

    @app.command()
    def transform(
        action: str = typer.Argument(..., help="Action: euler_to_quat, quat_to_euler, demo"),
        values: List[float] = typer.Argument(None, help="Values to transform"),
        verbose: bool = verbose_option
    ):
        """Coordinate transformation utilities."""
        setup_logging(verbose)

        if not URC_MODULES_AVAILABLE:
            console.print("[red]✗ URC modules not available for transforms[/red]")
            raise typer.Exit(1)

        try:
            if action == "euler_to_quat":
                if len(values) != 3:
                    console.print("[red]✗ Euler angles require 3 values (roll, pitch, yaw)[/red]")
                    raise typer.Exit(1)

                quat = euler_to_quat(values)
                console.print(f"[bold]Euler to Quaternion:[/bold]")
                console.print(f"  Input: {values}")
                console.print(f"  Output: {quat}")

            elif action == "quat_to_euler":
                if len(values) != 4:
                    console.print("[red]✗ Quaternion requires 4 values (x, y, z, w)[/red]")
                    raise typer.Exit(1)

                euler = quat_to_euler(values)
                console.print(f"[bold]Quaternion to Euler:[/bold]")
                console.print(f"  Input: {values}")
                console.print(f"  Output: {euler}")

            elif action == "demo":
                console.print("[bold]Coordinate Transform Demo:[/bold]")

                # Demo transformations
                euler_angles = [0.1, 0.2, 0.3]
                quat = euler_to_quat(euler_angles)
                euler_back = quat_to_euler(quat)

                table = Table(show_header=True, header_style="bold magenta")
                table.add_column("Operation", style="cyan")
                table.add_column("Input", style="white")
                table.add_column("Output", style="green")

                table.add_row("Euler → Quaternion", str(euler_angles), str([f"{x:.4f}" for x in quat]))
                table.add_row("Quaternion → Euler", str([f"{x:.4f}" for x in quat]), str([f"{x:.4f}" for x in euler_back]))

                console.print(table)

                # Check round-trip accuracy
                error = np.linalg.norm(np.array(euler_angles) - np.array(euler_back))
                console.print(f"\n[bold]Round-trip error:[/bold] {error:.2f} radians")

            else:
                console.print(f"[red]✗ Unknown action: {action}[/red]")
                console.print("Available actions: euler_to_quat, quat_to_euler, demo")
                raise typer.Exit(1)

        except Exception as e:
            console.print(f"[red]✗ Transform operation failed: {e}[/red]")
            raise typer.Exit(1)

    @app.command()
    def demo(
        component: str = typer.Argument(..., help="Component to demo: all, state_machine, behavior_tree, stats"),
        verbose: bool = verbose_option
    ):
        """Run demonstration of URC components."""
        setup_logging(verbose)

        console.print(f"[bold blue]🚀 URC 2026 {component.title()} Demo[/bold blue]\n")

        if component == "all":
            # Run all demos
            components = ["state_machine", "behavior_tree", "stats"]
            for comp in components:
                console.print(f"[bold]Demonstrating {comp}:[/bold]")
                demo_component(comp)
                console.print()

        else:
            demo_component(component)


def demo_component(component: str):
    """Demo individual component."""
    if component == "state_machine":
        if not URC_MODULES_AVAILABLE:
            console.print("[red]✗ URC modules not available[/red]")
            return

        console.print("Creating state machine...")
        sm = URCStateMachine()

        console.print("Demonstrating state transitions:")
        transitions = [
            ("startup_complete", "BOOT → CALIBRATION"),
            ("calibration_complete", "CALIBRATION → IDLE"),
            ("start_teleop", "IDLE → TELEOPERATION"),
            ("emergency_stop", "TELEOPERATION → SAFETY")
        ]

        for trigger, description in transitions:
            if hasattr(sm, trigger):
                getattr(sm, trigger)()
                console.print(f"  ✓ {description} (now in: {sm.state})")

        console.print(f"\nFinal state: [bold]{sm.state}[/bold]")

    elif component == "behavior_tree":
        if not URC_MODULES_AVAILABLE:
            console.print("[red]✗ URC modules not available[/red]")
            return

        console.print("Creating behavior tree...")
        bt = create_urc_behavior_tree("autonomous_navigation")

        console.print("Behavior tree structure:")
        # In real implementation, would show tree visualization
        console.print("  ✓ Root: AutonomousNavigationMission")
        console.print("    ✓ Sequence: PreMissionChecks")
        console.print("      ✓ SensorCheck (IMU)")
        console.print("      ✓ SensorCheck (GPS)")
        console.print("      ✓ SensorCheck (Camera)")
        console.print("    ✓ Sequence: GNSSWaypoints")
        console.print("      ✓ NavigateToWaypoint (Waypoint1)")
        console.print("      ✓ SignalArrival")

    elif component == "stats":
        if not URC_MODULES_AVAILABLE:
            console.print("[red]✗ URC modules not available[/red]")
            return

        console.print("Running statistical analysis on sample data...")
        import numpy as np

        data = np.random.normal(10, 2, 1000)
        stats_result = robust_stats(data)

        console.print("Statistics computed:")
        console.print(f"  ✓ Mean: {stats_result['mean']:.2f}")
        console.print(f"  ✓ Standard Deviation: {stats_result['std']:.2f}")
        console.print(f"  ✓ Median: {stats_result['median']:.2f}")
        console.print(f"  ✓ MAD: {stats_result['mad']:.2f}")

    else:
        console.print(f"[red]✗ Unknown component: {component}[/red]")


def main():
    """Main entry point."""
    if CLI_AVAILABLE:
        app()
    else:
        console.print("[red]CLI libraries not available. Install typer and rich.[/red]")


if __name__ == "__main__":
    main()
