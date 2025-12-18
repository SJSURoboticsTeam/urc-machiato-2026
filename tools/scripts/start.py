#!/usr/bin/env python3
"""
URC 2026 Unified Launcher

Single entry point for starting different system components.

Usage:
    ./start.py [environment] [component]

Examples:
    ./start.py dev frontend     # Start frontend in development
    ./start.py dev dashboard    # Start testing dashboard
    ./start.py prod autonomy    # Start full autonomy system
    ./start.py dev simulation   # Start simulation
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path
from typing import Optional

# Project paths
PROJECT_ROOT = Path(__file__).parent.parent.parent  # Go up to project root
SCRIPTS_DIR = PROJECT_ROOT / "tools" / "scripts"
SRC_DIR = PROJECT_ROOT / "src"
AUTONOMY_DIR = SRC_DIR / "autonomy"


def run_command(
    cmd: str | list[str], cwd: Optional[Path] = None, env: Optional[dict] = None
) -> bool:
    """Unified command runner with consistent error handling."""
    cmd_list = cmd if isinstance(cmd, list) else cmd.split()

    try:
        print(f"[IGNITE] Running: {' '.join(cmd_list)}")
        subprocess.run(cmd_list, cwd=cwd, env=env, check=True)
        print("[PASS] Success")
        return True
    except subprocess.CalledProcessError as e:
        print(f"[FAIL] Command failed: {e}")
        return False
    except KeyboardInterrupt:
        print("\n Interrupted by user")
        return True
    except Exception as e:
        print(f"[FAIL] Unexpected error: {e}")
        return False


def start_frontend(environment: str = "dev") -> bool:
    """Start the web frontend."""
    print("[IGNITE] Starting Frontend...")

    frontend_dir = SRC_DIR / "frontend"
    if not frontend_dir.exists():
        print("[FAIL] Frontend directory not found")
        return False

    # Install dependencies if needed
    if not (frontend_dir / "node_modules").exists():
        print(" Installing frontend dependencies...")
        if not run_command("npm install", cwd=frontend_dir):
            return False

    # Start development server
    print("[NETWORK] Starting development server on http://localhost:5173")
    return run_command("npm run dev -- --host 0.0.0.0", cwd=frontend_dir)


def start_dashboard() -> bool:
    """Start the testing dashboard (backend + frontend)."""
    print("[EXPERIMENT] Starting Testing Dashboard...")

    dashboard_script = SCRIPTS_DIR / "testing" / "start_dashboard.sh"
    if not dashboard_script.exists():
        print("[FAIL] Dashboard script not found")
        return False

    return run_command(str(dashboard_script))


def start_autonomy(environment: str = "dev") -> bool:
    """Start the full autonomy system."""
    print(" Starting Autonomy System...")

    # Set environment variable for config
    env = os.environ.copy()
    env["URC_ENV"] = environment

    # Source ROS2 if available
    ros_env = {}
    if environment == "dev":
        # For development, try to source ROS2
        ros_setup = Path("/opt/ros/humble/setup.bash")
        if ros_setup.exists():
            env.update({"ROS_DOMAIN_ID": "42", "URC_ENV": "development"})

    # Launch the integrated system
    launch_file = AUTONOMY_DIR / "launch" / "integrated_system.launch.py"
    if not launch_file.exists():
        print("[FAIL] Launch file not found")
        return False

    cmd = f"ros2 launch {launch_file}"
    return run_command(cmd, env=env)


def start_simulation() -> bool:
    """Start the simulation environment."""
    print(" Starting Simulation...")

    launch_file = SCRIPTS_DIR / "launch" / "rover_simulation.launch.py"
    if not launch_file.exists():
        print("[FAIL] Simulation launch file not found")
        return False

    cmd = f"ros2 launch {launch_file}"
    return run_command(cmd)


def main():
    parser = argparse.ArgumentParser(
        description="URC 2026 Unified Launcher",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  ./start.py dev frontend     # Start frontend in development
  ./start.py dev dashboard    # Start testing dashboard
  ./start.py prod autonomy    # Start full autonomy system
  ./start.py dev simulation   # Start simulation

Environments:
  dev    - Development mode (default)
  prod   - Production mode

Components:
  frontend   - Web interface only
  dashboard  - Testing dashboard (backend + frontend)
  autonomy   - Full autonomy system
  simulation - Gazebo simulation
        """,
    )

    parser.add_argument(
        "environment",
        nargs="?",
        default="dev",
        choices=["dev", "prod"],
        help="Environment (dev/prod)",
    )

    parser.add_argument(
        "component",
        nargs="?",
        choices=["frontend", "dashboard", "autonomy", "simulation"],
        help="Component to start",
    )

    args = parser.parse_args()

    # If no component specified, show help
    if not args.component:
        parser.print_help()
        return 1

    print("[OBJECTIVE] URC 2026 Launcher")
    print(f"   Environment: {args.environment}")
    print(f"   Component: {args.component}")
    print()

    # Route to appropriate starter
    starters = {
        "frontend": lambda: start_frontend(args.environment),
        "dashboard": start_dashboard,
        "autonomy": lambda: start_autonomy(args.environment),
        "simulation": start_simulation,
    }

    success = starters[args.component]()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
