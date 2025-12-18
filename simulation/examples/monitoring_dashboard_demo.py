#!/usr/bin/env python3
"""
Real-time Monitoring Dashboard Demo

Demonstrates the monitoring capabilities of the enhanced simulation framework.
Shows real-time metrics, alerts, and performance data.

Author: URC 2026 Autonomy Team
"""

import json
import time
from pathlib import Path

from simulation import SimulationManager
from simulation.core.logging_config import setup_simulation_logging


def create_monitoring_config():
    """Create configuration focused on monitoring."""
    return {
        "logging": {
            "enabled": True,
            "level": "INFO",
            "file": "monitoring_demo.log",
            "structured": True,
        },
        "monitoring": {
            "enabled": True,
            "interval": 0.5,  # Fast monitoring for demo
        },
        "tracing": {
            "enabled": True,
            "auto_profile_threshold": 0.05,  # Lower threshold for demo
        },
        "environment": {"tier": "real_life"},
        "sensors": [
            {"name": "gps", "type": "gps"},
            {"name": "imu", "type": "imu"},
        ],
        "network": {"profile": "rural_wifi"},
        "rover": {"model": "urc_rover"},
        "time": {"step_size": 0.1},
        "recording": {"record_interval": 0.1},
    }


def display_dashboard(sim_manager, duration_seconds=30):
    """Display real-time monitoring dashboard."""
    print("📊 Real-time Simulation Monitoring Dashboard")
    print("=" * 60)
    print("Monitoring for {} seconds...".format(duration_seconds))
    print("Press Ctrl+C to stop early")
    print()

    start_time = time.time()
    last_display = 0

    try:
        while time.time() - start_time < duration_seconds:
            current_time = time.time()

            # Update dashboard every 2 seconds
            if current_time - last_display >= 2.0:
                dashboard_data = sim_manager.monitor.get_dashboard_data()

                print("\033[2J\033[H", end="")  # Clear screen
                print("📊 Real-time Simulation Monitoring Dashboard")
                print("=" * 60)
                print(".1f")

                # Current metrics
                current = dashboard_data.get("current_metrics", {})
                system = current.get("system", {})
                simulation = current.get("simulation", {})

                print("💻 System Resources:")
                print(".1f")
                print(".1f")
                print(".1f")

                print("\n🎮 Simulation Status:")
                print(f"   Steps: {simulation.get('simulation_step_count', 0)}")
                print(".1f")
                print(f"   Active Sensors: {simulation.get('active_sensors', 0)}")
                print(".1f")

                # Performance trends
                trends = dashboard_data.get("performance_trends", {})
                if trends:
                    memory = trends.get("memory_usage", {})
                    cpu = trends.get("cpu_usage", {})

                    print("\n📈 Performance Trends (last 100 measurements):")
                    print(".1f")
                    print(".1f")

                # Recent alerts
                alerts = dashboard_data.get("recent_alerts", [])
                if alerts:
                    print("\n🚨 Recent Alerts:")
                    for alert in alerts[-3:]:  # Show last 3
                        level = alert.get("level", "info")
                        emoji = {
                            "critical": "🔴",
                            "warning": "🟡",
                            "error": "🔴",
                            "info": "ℹ️",
                        }.get(level, "❓")
                        print(f"   {emoji} {alert.get('message', 'Unknown alert')}")

                # Active operations (if tracing enabled)
                if hasattr(sim_manager.tracer, "get_active_operations"):
                    active_ops = sim_manager.tracer.get_active_operations()
                    if active_ops:
                        print("\n🔍 Active Operations:")
                        for op in active_ops[:3]:  # Show first 3
                            duration = op.get("duration_so_far", 0)
                            print(".2f")

                # RL stats if available
                rl_stats = dashboard_data.get("simulation_stats", {}).get("rl_training")
                if rl_stats:
                    print("\n🤖 RL Training Stats:")
                    print(f"   Episodes: {rl_stats.get('total_episodes', 0)}")
                    print(".3f")
                    print(".1f")

                print("\n" + "=" * 60)
                last_display = current_time

            # Small delay to prevent busy waiting
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n⏹️  Monitoring stopped by user")

    # Final summary
    print("\n📋 Final Monitoring Summary")
    print("=" * 60)

    final_data = sim_manager.monitor.get_dashboard_data()
    performance_report = sim_manager.monitor.get_performance_report()

    print("⏱️  Total Monitoring Time:")
    print(".1f")

    print("\n📊 Performance Summary:")
    sys_perf = performance_report.get("system_performance", {})
    sim_perf = performance_report.get("simulation_performance", {})

    if sys_perf:
        print(".1f")
        print(".1f")

    if sim_perf:
        print(f"   Simulation Steps: {sim_perf.get('total_steps', 0)}")
        print(".3f")

    # Alert summary
    alert_counts = final_data.get("alert_counts", {})
    if any(alert_counts.values()):
        print("\n🚨 Alert Summary:")
        for level, count in alert_counts.items():
            if count > 0:
                emoji = {
                    "critical": "🔴",
                    "warning": "🟡",
                    "error": "🔴",
                    "info": "ℹ️",
                }.get(level, "❓")
                print(f"   {emoji} {level.capitalize()}: {count}")

    # Export monitoring data
    export_file = "monitoring_dashboard_data.json"
    sim_manager.monitor.export_metrics(export_file)
    print(f"\n💾 Monitoring data exported to: {export_file}")

    if hasattr(sim_manager.tracer, "export_traces"):
        trace_file = "monitoring_traces.json"
        sim_manager.tracer.export_traces(trace_file)
        print(f"🔍 Trace data exported to: {trace_file}")


def run_monitoring_demo():
    """Run the monitoring dashboard demo."""
    # Setup logging
    setup_simulation_logging(
        log_level="WARNING",  # Reduce log noise for demo
        log_file="monitoring_demo.log",
        enable_structured=True,
    )

    # Create and initialize simulation
    config = create_monitoring_config()
    sim_manager = SimulationManager()

    print("🚀 Initializing simulation for monitoring demo...")
    if not sim_manager.initialize(config):
        print("❌ Failed to initialize simulation")
        return

    # Start simulation in background
    if not sim_manager.start():
        print("❌ Failed to start simulation")
        return

    try:
        # Run monitoring dashboard
        display_dashboard(sim_manager, duration_seconds=15)

    finally:
        # Cleanup
        print("\n🧹 Cleaning up...")
        sim_manager.monitor.stop_monitoring()
        sim_manager.stop()
        print("✅ Demo complete!")


if __name__ == "__main__":
    run_monitoring_demo()
