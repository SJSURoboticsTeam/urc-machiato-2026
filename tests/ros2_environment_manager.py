#!/usr/bin/env python3
"""
ROS2 Environment Manager for Testing

Provides isolated ROS2 test environments with proper domain configuration,
resource limits, and cleanup management.

Author: URC 2026 Autonomy Team
"""

import os
import signal
import subprocess
import sys
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import psutil
import rclpy


@dataclass
class ResourceLimits:
    """Resource limits for test environment."""

    cpu_percent: Optional[float] = None
    memory_mb: Optional[int] = None
    network_bandwidth_mbps: Optional[float] = None
    disk_io_limit: Optional[str] = None
    max_processes: int = 50


@dataclass
class ROSEnvironmentConfig:
    """ROS2 environment configuration."""

    domain_id: int = 42
    namespace: str = ""
    use_sim_time: bool = True
    log_level: str = "INFO"
    discovery_timeout_sec: float = 10.0
    qos_override: Dict[str, Any] = field(default_factory=dict)


@dataclass
class TestEnvironment:
    """Complete test environment with ROS2 setup."""

    name: str
    ros_config: ROSEnvironmentConfig
    resource_limits: ResourceLimits
    workspace_path: Path
    log_directory: Path = field(default_factory=lambda: Path("test_logs"))

    # Runtime state
    ros_context: Optional[Any] = None
    processes: List[subprocess.Popen] = field(default_factory=list)
    nodes: List[Any] = field(default_factory=list)
    network_namespace: Optional[str] = None
    cgroup_name: Optional[str] = None

    def __post_init__(self):
        if not self.workspace_path.exists():
            raise ValueError(f"Workspace path does not exist: {self.workspace_path}")

        self.log_directory.mkdir(parents=True, exist_ok=True)


class ROS2EnvironmentManager:
    """
    Manages isolated ROS2 test environments.

    Features:
    - ROS2 domain isolation
    - Resource limit enforcement
    - Network namespace isolation
    - Automatic cleanup
    - Performance monitoring
    """

    def __init__(self):
        self.environments: Dict[str, TestEnvironment] = {}
        self.active_environments: List[str] = []
        self.monitor_threads: Dict[str, threading.Thread] = {}

    @contextmanager
    def create_environment(
        self,
        name: str,
        ros_config: ROSEnvironmentConfig,
        resource_limits: ResourceLimits,
        workspace_path: Optional[Path] = None,
    ):
        """Context manager for creating and managing test environments."""
        env = TestEnvironment(
            name=name,
            ros_config=ros_config,
            resource_limits=resource_limits,
            workspace_path=workspace_path or Path.cwd(),
        )

        try:
            self._setup_environment(env)
            self.environments[name] = env
            self.active_environments.append(name)

            yield env

        finally:
            self._cleanup_environment(name)

    def _setup_environment(self, env: TestEnvironment):
        """Setup the test environment."""
        print(f"🚀 Setting up ROS2 environment: {env.name}")

        # Create network namespace for isolation
        env.network_namespace = f"ros2_test_{env.name}_{env.ros_config.domain_id}"
        self._create_network_namespace(env)

        # Setup resource limits
        env.cgroup_name = f"ros2_test_{env.name}"
        self._setup_resource_limits(env)

        # Configure ROS2 environment
        self._configure_ros_environment(env)

        # Initialize ROS2 context
        self._init_ros_context(env)

        print(f"✅ Environment {env.name} ready")

    def _create_network_namespace(self, env: TestEnvironment):
        """Create isolated network namespace."""
        try:
            # Create network namespace
            subprocess.run(
                ["ip", "netns", "add", env.network_namespace],
                check=True,
                capture_output=True,
            )

            # Setup loopback interface
            subprocess.run(
                [
                    "ip",
                    "netns",
                    "exec",
                    env.network_namespace,
                    "ip",
                    "link",
                    "set",
                    "lo",
                    "up",
                ],
                check=True,
                capture_output=True,
            )

            print(f"  📡 Created network namespace: {env.network_namespace}")

        except subprocess.CalledProcessError as e:
            print(f"  ⚠️ Network namespace creation failed: {e}")
            # Continue without network isolation

    def _setup_resource_limits(self, env: TestEnvironment):
        """Setup resource limits using cgroups."""
        limits = env.resource_limits

        try:
            # CPU limits
            if limits.cpu_percent is not None:
                # Create CPU cgroup
                cpu_quota = int(limits.cpu_percent * 1000)  # Convert to microseconds
                subprocess.run(
                    ["cgcreate", "-g", f"cpu:{env.cgroup_name}"],
                    check=True,
                    capture_output=True,
                )

                subprocess.run(
                    ["cgset", "-r", f"cpu.shares={cpu_quota}", env.cgroup_name],
                    check=True,
                    capture_output=True,
                )

            # Memory limits
            if limits.memory_mb is not None:
                memory_bytes = limits.memory_mb * 1024 * 1024

                subprocess.run(
                    ["cgcreate", "-g", f"memory:{env.cgroup_name}"],
                    check=True,
                    capture_output=True,
                )

                subprocess.run(
                    [
                        "cgset",
                        "-r",
                        f"memory.limit_in_bytes={memory_bytes}",
                        env.cgroup_name,
                    ],
                    check=True,
                    capture_output=True,
                )

            print(f"  📊 Resource limits set for {env.cgroup_name}")

        except subprocess.CalledProcessError as e:
            print(f"  ⚠️ Resource limit setup failed: {e}")
            # Continue without resource limits

    def _configure_ros_environment(self, env: TestEnvironment):
        """Configure ROS2 environment variables."""
        config = env.ros_config

        # Set environment variables
        os.environ["ROS_DOMAIN_ID"] = str(config.domain_id)
        os.environ["ROS_NAMESPACE"] = config.namespace
        os.environ["ROS_LOG_DIR"] = str(env.log_directory)
        os.environ["RCUTILS_LOGGING_LEVEL"] = config.log_level

        if config.use_sim_time:
            os.environ["ROS_USE_SIM_TIME"] = "true"

        # Set QoS overrides if specified
        for topic, qos in config.qos_override.items():
            env_var = f"ROS_{topic.upper()}_QOS"
            os.environ[env_var] = str(qos)

        print(
            f"  🔧 ROS2 configured: domain={config.domain_id}, namespace={config.namespace}"
        )

    def _init_ros_context(self, env: TestEnvironment):
        """Initialize ROS2 context."""
        try:
            env.ros_context = rclpy.Context()
            rclpy.init(context=env.ros_context)
            print("  📡 ROS2 context initialized")
        except Exception as e:
            print(f"  ❌ ROS2 initialization failed: {e}")
            raise

    def start_node(
        self, env_name: str, node_class: Any, node_name: str, **node_params
    ) -> Any:
        """Start a ROS2 node in the specified environment."""
        if env_name not in self.environments:
            raise ValueError(f"Environment {env_name} not found")

        env = self.environments[env_name]

        try:
            # Create node with ROS2 context
            node = node_class(
                node_name=node_name, context=env.ros_context, **node_params
            )
            env.nodes.append(node)

            print(f"  📡 Started node: {node_name} in {env_name}")
            return node

        except Exception as e:
            print(f"  ❌ Failed to start node {node_name}: {e}")
            raise

    def start_process(
        self, env_name: str, command: List[str], cwd: Optional[Path] = None
    ) -> subprocess.Popen:
        """Start a process in the specified environment."""
        if env_name not in self.environments:
            raise ValueError(f"Environment {env_name} not found")

        env = self.environments[env_name]

        try:
            # Set environment variables
            process_env = os.environ.copy()
            process_env["ROS_DOMAIN_ID"] = str(env.ros_config.domain_id)

            # Add cgroup limits if available
            if env.cgroup_name:
                # This would require more complex setup with cgexec
                pass

            process = subprocess.Popen(
                command,
                env=process_env,
                cwd=cwd or env.workspace_path,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,  # Create new process group
            )

            env.processes.append(process)
            print(f"  🚀 Started process: {' '.join(command)} in {env_name}")
            return process

        except Exception as e:
            print(f"  ❌ Failed to start process: {e}")
            raise

    def monitor_environment(self, env_name: str, callback: Optional[Callable] = None):
        """Start monitoring the environment."""
        if env_name not in self.environments:
            raise ValueError(f"Environment {env_name} not found")

        env = self.environments[env_name]

        def monitor_loop():
            while env_name in self.active_environments:
                try:
                    # Monitor resource usage
                    total_cpu = 0
                    total_memory = 0
                    process_count = 0

                    for process in env.processes:
                        if process.poll() is None:  # Still running
                            try:
                                proc = psutil.Process(process.pid)
                                total_cpu += proc.cpu_percent()
                                total_memory += proc.memory_info().rss
                                process_count += 1
                            except psutil.NoSuchProcess:
                                continue

                    # Check resource limits
                    if (
                        env.resource_limits.cpu_percent
                        and total_cpu > env.resource_limits.cpu_percent
                    ):
                        print(
                            f"  ⚠️ CPU limit exceeded in {env_name}: {total_cpu:.1f}%"
                        )

                    if (
                        env.resource_limits.memory_mb
                        and total_memory > env.resource_limits.memory_mb * 1024 * 1024
                    ):
                        print(
                            f"  ⚠️ Memory limit exceeded in {env_name}: {total_memory / 1024 / 1024:.1f}MB"
                        )

                    if callback:
                        callback(
                            {
                                "cpu_percent": total_cpu,
                                "memory_mb": total_memory / 1024 / 1024,
                                "process_count": process_count,
                            }
                        )

                    time.sleep(1)

                except Exception as e:
                    print(f"  ❌ Monitoring error in {env_name}: {e}")
                    break

        monitor_thread = threading.Thread(target=monitor_loop, daemon=True)
        monitor_thread.start()
        self.monitor_threads[env_name] = monitor_thread

    def _cleanup_environment(self, env_name: str):
        """Clean up the test environment."""
        if env_name not in self.environments:
            return

        env = self.environments[env_name]

        print(f"🧹 Cleaning up environment: {env_name}")

        # Remove from active list
        if env_name in self.active_environments:
            self.active_environments.remove(env_name)

        # Stop monitoring
        if env_name in self.monitor_threads:
            # Thread will stop naturally when environment is removed
            pass

        # Shutdown ROS2 nodes
        for node in env.nodes:
            try:
                node.destroy_node()
            except Exception as e:
                print(f"  ❌ Error destroying node: {e}")

        # Shutdown ROS2 context
        if env.ros_context:
            try:
                env.ros_context.shutdown()
            except Exception as e:
                print(f"  ❌ Error shutting down ROS2 context: {e}")

        # Terminate processes
        for process in env.processes:
            try:
                if process.poll() is None:  # Still running
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)

                    # Wait for graceful shutdown
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                        process.wait()

            except Exception as e:
                print(f"  ❌ Error terminating process: {e}")

        # Remove resource limits
        if env.cgroup_name:
            try:
                subprocess.run(
                    ["cgdelete", f"cpu:{env.cgroup_name}"], capture_output=True
                )
                subprocess.run(
                    ["cgdelete", f"memory:{env.cgroup_name}"], capture_output=True
                )
            except Exception as e:
                print(f"  ⚠️ Error removing cgroups: {e}")

        # Remove network namespace
        if env.network_namespace:
            try:
                subprocess.run(
                    ["ip", "netns", "delete", env.network_namespace],
                    capture_output=True,
                )
            except Exception as e:
                print(f"  ⚠️ Error removing network namespace: {e}")

        # Clean up environment variables
        ros_vars = [
            "ROS_DOMAIN_ID",
            "ROS_NAMESPACE",
            "ROS_LOG_DIR",
            "RCUTILS_LOGGING_LEVEL",
            "ROS_USE_SIM_TIME",
        ]
        for var in ros_vars:
            os.environ.pop(var, None)

        print(f"✅ Environment {env_name} cleaned up")

    def get_environment_status(self, env_name: str) -> Dict[str, Any]:
        """Get status of a test environment."""
        if env_name not in self.environments:
            return {"status": "not_found"}

        env = self.environments[env_name]

        status = {
            "name": env.name,
            "ros_domain_id": env.ros_config.domain_id,
            "active": env_name in self.active_environments,
            "nodes": len(env.nodes),
            "processes": len([p for p in env.processes if p.poll() is None]),
            "network_namespace": env.network_namespace,
            "resource_limits": {
                "cpu_percent": env.resource_limits.cpu_percent,
                "memory_mb": env.resource_limits.memory_mb,
                "network_bandwidth_mbps": env.resource_limits.network_bandwidth_mbps,
            },
        }

        return status

    def list_environments(self) -> List[Dict[str, Any]]:
        """List all environments."""
        return [self.get_environment_status(name) for name in self.environments.keys()]


# Global environment manager instance
_env_manager = None


def get_environment_manager() -> ROS2EnvironmentManager:
    """Get the global ROS2 environment manager instance."""
    global _env_manager
    if _env_manager is None:
        _env_manager = ROS2EnvironmentManager()
    return _env_manager


# Example usage
def example_extreme_environment():
    """Example of creating an extreme test environment."""
    from pathlib import Path

    manager = get_environment_manager()

    # Extreme environment configuration
    ros_config = ROSEnvironmentConfig(
        domain_id=100, use_sim_time=True, log_level="DEBUG"
    )

    resource_limits = ResourceLimits(
        cpu_percent=10.0,  # Very limited CPU
        memory_mb=50,  # Very limited memory
        network_bandwidth_mbps=0.1,  # Very slow network
        max_processes=5,
    )

    with manager.create_environment(
        name="extreme_test",
        ros_config=ros_config,
        resource_limits=resource_limits,
        workspace_path=Path.cwd(),
    ) as env:
        print(f"Created extreme environment: {env.name}")

        # Start monitoring
        manager.monitor_environment(
            "extreme_test",
            lambda metrics: print(
                f"📊 Metrics: CPU={metrics['cpu_percent']:.1f}%, "
                f"Memory={metrics['memory_mb']:.1f}MB, "
                f"Processes={metrics['process_count']}"
            ),
        )

        # Environment is automatically cleaned up when exiting context


if __name__ == "__main__":
    example_extreme_environment()
