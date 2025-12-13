"""Simulation time management.

Provides precise time control for simulation including real-time scaling,
time stepping, and synchronization.

Author: URC 2026 Autonomy Team
"""

import time
from typing import Any, Dict


class TimeManager:
    """Manages simulation time and stepping.

    Provides precise control over simulation time including:
    - Fixed time steps
    - Real-time scaling
    - Time synchronization
    - Performance monitoring
    """

    def __init__(self):
        """Initialize time manager."""
        self.current_time = 0.0
        self.real_time_start = 0.0
        self.simulation_time_start = 0.0
        self.real_time_factor = 1.0
        self.step_size = 0.01  # 100Hz default
        self.max_step_size = 0.1  # Maximum allowed step
        self.min_step_size = 0.001  # Minimum allowed step
        self.step_count = 0

        # Performance tracking
        self.last_step_time = 0.0
        self.step_times: list[float] = []
        self.max_step_time = 0.0

    def initialize(self, config: Dict[str, Any]):
        """Initialize time manager with configuration.

        Args:
            config: Time configuration dictionary
        """
        self.real_time_factor = config.get("real_time_factor", 1.0)
        self.step_size = config.get("step_size", 0.01)
        self.max_step_size = config.get("max_step_size", 0.1)
        self.min_step_size = config.get("min_step_size", 0.001)

        # Validate step size
        if not (self.min_step_size <= self.step_size <= self.max_step_size):
            raise ValueError(
                f"Step size {self.step_size} not in range "
                f"[{self.min_step_size}, {self.max_step_size}]"
            )

        # Reset state
        self.reset()

    def step(self, dt: float) -> float:
        """Advance simulation time by specified delta.

        Args:
            dt: Time step in seconds

        Returns:
            float: New simulation time
        """
        # Clamp dt to valid range
        dt = max(self.min_step_size, min(dt, self.max_step_size))

        # Update simulation time
        self.current_time += dt
        self.step_count += 1

        # Track performance
        current_real_time = time.time()
        if self.last_step_time > 0:
            step_duration = current_real_time - self.last_step_time
            self.step_times.append(step_duration)
            self.max_step_time = max(self.max_step_time, step_duration)

            # Keep only recent samples (last 100)
            if len(self.step_times) > 100:
                self.step_times.pop(0)

        self.last_step_time = current_real_time

        return self.current_time

    def reset(self):
        """Reset time manager to initial state."""
        self.current_time = 0.0
        self.real_time_start = time.time()
        self.simulation_time_start = 0.0
        self.step_count = 0
        self.last_step_time = 0.0
        self.step_times.clear()
        self.max_step_time = 0.0

    def get_real_time_elapsed(self) -> float:
        """Get elapsed real time since start.

        Returns:
            float: Real time elapsed in seconds
        """
        return time.time() - self.real_time_start

    def get_simulation_time_ratio(self) -> float:
        """Get ratio of simulation time to real time.

        Returns:
            float: Time ratio (>1 means simulation faster than real-time)
        """
        real_elapsed = self.get_real_time_elapsed()
        if real_elapsed > 0:
            return self.current_time / real_elapsed
        return 0.0

    def is_real_time(self, tolerance: float = 0.1) -> bool:
        """Check if simulation is running in real-time.

        Args:
            tolerance: Acceptable deviation from real-time (default 10%)

        Returns:
            bool: True if running approximately real-time
        """
        ratio = self.get_simulation_time_ratio()
        return abs(ratio - 1.0) <= tolerance

    def sleep_until_real_time(self):
        """Sleep to maintain real-time simulation pacing."""
        if self.real_time_factor <= 0:
            return

        target_real_time = self.current_time / self.real_time_factor
        current_real_time = time.time() - self.real_time_start
        sleep_time = max(0, target_real_time - current_real_time)

        if sleep_time > 0:
            time.sleep(sleep_time)

    def get_statistics(self) -> Dict[str, Any]:
        """Get time management statistics.

        Returns:
            Dict containing time statistics
        """
        return {
            "current_time": self.current_time,
            "step_count": self.step_count,
            "real_time_elapsed": self.get_real_time_elapsed(),
            "simulation_time_ratio": self.get_simulation_time_ratio(),
            "is_real_time": self.is_real_time(),
            "average_step_time": (
                sum(self.step_times) / len(self.step_times)
                if self.step_times
                else 0
            ),
            "max_step_time": self.max_step_time,
            "step_size": self.step_size,
            "real_time_factor": self.real_time_factor,
        }

    def set_real_time_factor(self, factor: float):
        """Set real-time scaling factor.

        Args:
            factor: Real-time factor (>1 = faster than real-time,
                   <1 = slower, 0 = as fast as possible)
        """
        if factor < 0:
            raise ValueError("Real-time factor must be non-negative")

        self.real_time_factor = factor

    def set_step_size(self, step_size: float):
        """Set simulation step size.

        Args:
            step_size: Step size in seconds
        """
        if not (self.min_step_size <= step_size <= self.max_step_size):
            raise ValueError(
                f"Step size {step_size} not in range "
                f"[{self.min_step_size}, {self.max_step_size}]"
            )

        self.step_size = step_size
