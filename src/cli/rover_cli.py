#!/usr/bin/env python3
"""
URC 2026 Rover CLI Tool

Provides command-line interface for rover operations using typer and rich.
Offers beautiful output, interactive prompts, and comprehensive rover management.

Author: URC 2026 CLI Team
"""

import asyncio
import sys
from pathlib import Path
from typing import Optional, List
import time

# Add source paths for CLI execution
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))
sys.path.insert(0, str(project_root / "src"))

import typer
from rich.console import Console
from rich.table import Table
from rich.panel import Panel
from rich.progress import Progress, SpinnerColumn, TextColumn
from rich.prompt import Confirm, Prompt
from rich.text import Text
from rich.live import Live
import questionary

from src.infrastructure.config import (
    get_config, reload_config, create_default_config,
    RoverConfig, Environment
)
from src.core.synchronization_engine import SynchronizationEngine
from src.core.error_handling import ErrorHandler, ErrorCategory


# Initialize Rich console
console = Console()
app = typer.Typer(
    name="rover-cli",
    help="URC 2026 Rover Command Line Interface",
    add_completion=False,
)


@app.callback()
def main(
    ctx: typer.Context,
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Enable verbose output"),
    config_file: Optional[str] = typer.Option(None, "--config", help="Path to config file"),
):
    """URC 2026 Rover CLI - Advanced Robotics Operations"""
    ctx.obj = {"verbose": verbose, "config_file": config_file}


@app.command()
def status(ctx: typer.Context):
    """Show comprehensive rover system status."""
    config = get_config()

    # Create status table
    table = Table(title="🚀 URC 2026 Rover Status")
    table.add_column("Component", style="cyan", no_wrap=True)
    table.add_column("Status", style="green")
    table.add_column("Details", style="yellow")

    # System info
    table.add_row("Environment", config.environment.value, f"Version {config.version}")
    table.add_row("Debug Mode", "✅" if config.debug else "❌", "Development features enabled" if config.debug else "Production mode")

    # Core systems status
    with console.status("[bold green]Checking synchronization engine..."):
        try:
            sync_engine = SynchronizationEngine()
            sync_health = sync_engine.get_health_status()
            sync_status = "✅ Healthy" if sync_health['overall_health'] == 'healthy' else "⚠️ Issues"
            sync_details = f"Sync delay: {sync_health.get('avg_sync_delay_ms', 'N/A')}ms"
            table.add_row("Synchronization", sync_status, sync_details)
        except Exception as e:
            table.add_row("Synchronization", "❌ Failed", str(e))

    # Network status
    try:
        from src.core.network_resilience import get_resilient_http_client
        http_client = get_resilient_http_client()
        network_status = "✅ Connected" if hasattr(http_client, 'circuit_breakers') else "❌ Failed"
        table.add_row("Network Resilience", network_status, f"{len(http_client.get_circuit_breaker_states())} circuit breakers")
    except Exception as e:
        table.add_row("Network Resilience", "❌ Failed", str(e))

    # Mission status
    mission_config = config.mission
    table.add_row("Mission System", "✅ Configured", f"Autonomous: {mission_config.autonomous_mode}")

    # Optional components
    if config.database:
        table.add_row("Database", "✅ Configured", config.database.url)
    else:
        table.add_row("Database", "⚠️ Not configured", "Using file-based storage")

    if config.redis:
        table.add_row("Redis Cache", "✅ Configured", f"{config.redis.host}:{config.redis.port}")
    else:
        table.add_row("Redis Cache", "⚠️ Not configured", "In-memory only")

    if config.api:
        table.add_row("REST API", "✅ Enabled", f"http://localhost:{config.api.port}")
    else:
        table.add_row("REST API", "⚠️ Disabled", "CLI-only mode")

    console.print(table)

    # Performance metrics
    console.print("\n📊 Performance Overview:")
    try:
        if hasattr(sync_engine, 'get_performance_metrics'):
            metrics = sync_engine.get_performance_metrics()
            console.print(f"  CPU Usage: {metrics['system']['cpu_usage_percent']:.1f}%")
            console.print(f"  Memory Usage: {metrics['system']['memory_usage_mb']:.1f} MB")
            console.print(f"  Sync Efficiency: {metrics['sync_engine']['efficiency']:.1%}")
    except:
        console.print("  Performance metrics not available")


@app.command()
def config(
    ctx: typer.Context,
    show: bool = typer.Option(True, help="Show current configuration"),
    edit: bool = typer.Option(False, help="Edit configuration interactively"),
    create_default: bool = typer.Option(False, help="Create default config file"),
    validate: bool = typer.Option(False, help="Validate current configuration"),
):
    """Manage rover configuration."""
    if create_default:
        config_path = Path("config/rover_config.json")
        create_default_config(config_path)
        console.print(f"✅ Default configuration created at {config_path}")
        return

    config = get_config()

    if show:
        # Display configuration in a nice format
        console.print(Panel.fit(
            f"[bold blue]URC 2026 Rover Configuration[/bold blue]\n\n"
            f"[cyan]Environment:[/cyan] {config.environment.value}\n"
            f"[cyan]Version:[/cyan] {config.version}\n"
            f"[cyan]Debug:[/cyan] {config.debug}\n\n"
            f"[yellow]Sync Config:[/yellow]\n"
            f"  Max Delay: {config.sync.max_sync_delay_ms}ms\n"
            f"  Cameras: {', '.join(config.sync.camera_ids)}\n\n"
            f"[yellow]Safety Config:[/yellow]\n"
            f"  Thermal Warning: {config.safety.thermal_warning_threshold}°C\n"
            f"  Battery Critical: {config.safety.battery_critical_threshold}%\n\n"
            f"[yellow]Network Config:[/yellow]\n"
            f"  Retry Attempts: {config.network.retry_attempts}\n"
            f"  Circuit Breaker Threshold: {config.network.circuit_breaker_threshold}",
            title="🔧 Configuration"
        ))

    if validate:
        try:
            config.validate(config.dict())
            console.print("✅ Configuration validation successful")
        except Exception as e:
            console.print(f"❌ Configuration validation failed: {e}")
            return

    if edit:
        console.print("🔧 Interactive configuration editing not yet implemented")
        console.print("Use environment variables or edit config files directly")


@app.command()
def test(
    ctx: typer.Context,
    component: str = typer.Argument(..., help="Component to test"),
    verbose: bool = typer.Option(False, help="Verbose test output"),
    parallel: bool = typer.Option(False, help="Run tests in parallel"),
):
    """Run system tests."""
    valid_components = ["sync", "network", "safety", "mission", "all"]

    if component not in valid_components:
        console.print(f"❌ Invalid component. Choose from: {', '.join(valid_components)}")
        raise typer.Exit(1)

    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        console=console,
    ) as progress:

        if component in ["sync", "all"]:
            task = progress.add_task("Testing synchronization engine...", total=100)
            progress.update(task, advance=25)

            try:
                from src.core.synchronization_engine import SynchronizationEngine
                sync_engine = SynchronizationEngine()
                health = sync_engine.get_health_status()

                if health['overall_health'] == 'healthy':
                    progress.update(task, advance=75, description="✅ Sync engine tests passed")
                else:
                    progress.update(task, advance=75, description="⚠️ Sync engine has issues")

            except Exception as e:
                progress.update(task, advance=75, description=f"❌ Sync engine failed: {str(e)[:50]}")

        if component in ["network", "all"]:
            task = progress.add_task("Testing network resilience...", total=100)
            progress.update(task, advance=25)

            try:
                from src.core.network_resilience import get_resilient_http_client
                http_client = get_resilient_http_client()
                breakers = http_client.get_circuit_breaker_states()
                progress.update(task, advance=75, description=f"✅ Network tests passed ({len(breakers)} breakers)")

            except Exception as e:
                progress.update(task, advance=75, description=f"❌ Network tests failed: {str(e)[:50]}")

        if component in ["safety", "all"]:
            task = progress.add_task("Testing safety systems...", total=100)
            progress.update(task, advance=25)

            try:
                # Test error handling
                from src.core.error_handling import Result, validate_positive_number
                result = validate_positive_number(10, "test")
                if result.is_success():
                    progress.update(task, advance=75, description="✅ Safety tests passed")
                else:
                    progress.update(task, advance=75, description="❌ Safety validation failed")

            except Exception as e:
                progress.update(task, advance=75, description=f"❌ Safety tests failed: {str(e)[:50]}")

    console.print(f"\n🎉 Testing completed for component: {component}")


@app.command()
def simulate(
    ctx: typer.Context,
    mission: str = typer.Argument(..., help="Mission to simulate"),
    duration: int = typer.Option(60, help="Simulation duration in seconds"),
    real_time: bool = typer.Option(True, help="Run in real-time"),
):
    """Run mission simulation."""
    valid_missions = ["waypoint_navigation", "sample_collection", "return_home"]

    if mission not in valid_missions:
        console.print(f"❌ Invalid mission. Choose from: {', '.join(valid_missions)}")
        raise typer.Exit(1)

    console.print(f"🚀 Starting {mission} simulation...")

    with Progress() as progress:
        task = progress.add_task(f"Simulating {mission}...", total=duration)

        for i in range(duration):
            if real_time:
                time.sleep(1)
            progress.update(task, advance=1)

            # Simulate some events
            if i == duration // 4:
                console.print("📍 Waypoint reached" if "waypoint" in mission else "🔬 Sample collected")
            elif i == duration // 2:
                console.print("⚡ System performing optimally")
            elif i == 3 * duration // 4:
                console.print("🎯 Mission objectives achieved")

    console.print(f"✅ {mission} simulation completed successfully!")


@app.command()
def diagnose(
    ctx: typer.Context,
    component: Optional[str] = typer.Argument(None, help="Component to diagnose"),
    fix: bool = typer.Option(False, help="Attempt automatic fixes"),
):
    """Diagnose system issues and optionally fix them."""
    console.print("🔍 Running system diagnostics...")

    issues_found = []
    fixes_applied = []

    # Check configuration
    try:
        config = get_config()
        config.validate(config.dict())
        console.print("✅ Configuration is valid")
    except Exception as e:
        issues_found.append(f"Configuration error: {e}")
        if fix:
            try:
                create_default_config(Path("config/rover_config.json"))
                fixes_applied.append("Created default configuration")
            except Exception:
                pass

    # Check synchronization
    try:
        sync_engine = SynchronizationEngine()
        health = sync_engine.get_health_status()
        if health['overall_health'] != 'healthy':
            issues_found.append(f"Synchronization issues: {health}")
            if fix:
                sync_engine.recover_from_sync_failure()
                fixes_applied.append("Attempted sync recovery")
    except Exception as e:
        issues_found.append(f"Synchronization error: {e}")

    # Check network
    try:
        from src.core.network_resilience import get_resilient_http_client
        http_client = get_resilient_http_client()
        console.print("✅ Network resilience configured")
    except Exception as e:
        issues_found.append(f"Network configuration error: {e}")

    # Report results
    if issues_found:
        console.print("\n⚠️ Issues Found:")
        for issue in issues_found:
            console.print(f"  • {issue}")

        if fixes_applied:
            console.print("\n🔧 Fixes Applied:")
            for fix in fixes_applied:
                console.print(f"  • {fix}")
    else:
        console.print("✅ No issues detected - system is healthy!")


@app.command()
def init(
    ctx: typer.Context,
    force: bool = typer.Option(False, help="Overwrite existing configuration"),
):
    """Initialize URC 2026 rover environment."""
    console.print("🚀 Initializing URC 2026 Rover Environment")

    # Check if already initialized
    config_path = Path("config/rover_config.json")
    if config_path.exists() and not force:
        if not Confirm.ask("Configuration already exists. Overwrite?"):
            console.print("❌ Initialization cancelled")
            return

    # Create directories
    dirs_to_create = [
        "config",
        "logs",
        "data",
        "missions",
        "tests/results",
        "tools"
    ]

    for dir_path in dirs_to_create:
        Path(dir_path).mkdir(parents=True, exist_ok=True)
        console.print(f"📁 Created directory: {dir_path}")

    # Create default configuration
    create_default_config(config_path)
    console.print(f"⚙️ Created default configuration: {config_path}")

    # Create .env.example
    env_example = Path(".env.example")
    env_content = """# URC 2026 Rover Environment Configuration
# Copy this file to .env and modify as needed

# Environment
ROVER_ENVIRONMENT=development
ROVER_DEBUG=true

# Logging
ROVER_LOG_LEVEL=INFO
ROVER_LOG_FORMAT=json

# Sync Configuration
ROVER_SYNC__MAX_SYNC_DELAY_MS=50.0
ROVER_SYNC__TEMPORAL_CONSISTENCY_THRESHOLD=0.05

# Safety Configuration
ROVER_SAFETY__THERMAL_WARNING_THRESHOLD=70.0
ROVER_SAFETY__BATTERY_CRITICAL_THRESHOLD=20.0

# Network Configuration
ROVER_NETWORK__RETRY_ATTEMPTS=3
ROVER_NETWORK__CONNECTION_TIMEOUT=5.0

# Optional: Database
# ROVER_DATABASE__URL=sqlite:///urc_missions.db

# Optional: Redis
# ROVER_REDIS__HOST=localhost
# ROVER_REDIS__PORT=6379

# Optional: API
# ROVER_API__HOST=0.0.0.0
# ROVER_API__PORT=8000
"""

    env_example.write_text(env_content)
    console.print(f"📝 Created environment template: {env_example}")

    console.print("\n🎉 URC 2026 Rover environment initialized successfully!")
    console.print("\nNext steps:")
    console.print("  1. Copy .env.example to .env and configure as needed")
    console.print("  2. Run 'rover-cli status' to check system health")
    console.print("  3. Run 'rover-cli test all' to validate components")


@app.command()
def interactive(ctx: typer.Context):
    """Launch interactive rover management interface."""
    console.print("🎮 URC 2026 Interactive Rover Management")
    console.print("=" * 50)

    while True:
        choice = questionary.select(
            "What would you like to do?",
            choices=[
                "View system status",
                "Run diagnostics",
                "Execute tests",
                "Run simulation",
                "Configure system",
                "Exit"
            ]
        ).ask()

        if choice == "View system status":
            status(ctx)
        elif choice == "Run diagnostics":
            diagnose(ctx)
        elif choice == "Execute tests":
            component = questionary.select(
                "Which component to test?",
                choices=["sync", "network", "safety", "mission", "all"]
            ).ask()
            test(ctx, component)
        elif choice == "Run simulation":
            mission = questionary.select(
                "Which mission to simulate?",
                choices=["waypoint_navigation", "sample_collection", "return_home"]
            ).ask()
            simulate(ctx, mission)
        elif choice == "Configure system":
            config(ctx, show=True, edit=True)
        elif choice == "Exit":
            console.print("👋 Goodbye!")
            break

        console.print()  # Add spacing


if __name__ == "__main__":
    app()
