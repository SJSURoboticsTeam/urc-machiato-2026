"""RL-focused data recorder for training and evaluation.

Extends the base DataRecorder with RL-specific features including
episode tracking, reward signals, action history, and state transitions.

Author: URC 2026 Autonomy Team
"""

import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

from simulation.core.logging_config import get_simulation_logger
from simulation.tools.data_recorder import DataRecorder


@dataclass
class RLStep:
    """Single RL training step."""
    timestamp: float
    episode: int
    step: int
    state: Dict[str, Any]
    action: Dict[str, Any]
    reward: float
    next_state: Dict[str, Any]
    done: bool
    info: Dict[str, Any]
    simulation_time: float


@dataclass
class RLEpisode:
    """Complete RL training episode."""
    episode_id: int
    start_time: float
    end_time: float
    steps: List[RLStep]
    total_reward: float
    episode_length: int
    success: bool
    termination_reason: str
    metadata: Dict[str, Any]


class RewardFunction:
    """Configurable reward function for RL training."""

    def __init__(self, reward_type: str = "navigation_efficiency"):
        """Initialize reward function.

        Args:
            reward_type: Type of reward function
        """
        self.reward_type = reward_type
        self.logger = get_simulation_logger(__name__, "reward_function")

    def calculate_reward(self,
                        state: Dict[str, Any],
                        action: Dict[str, Any],
                        next_state: Dict[str, Any],
                        done: bool,
                        info: Dict[str, Any]) -> float:
        """Calculate reward for RL step.

        Args:
            state: Current state
            action: Action taken
            next_state: Next state
            done: Whether episode ended
            info: Additional information

        Returns:
            Reward value
        """
        if self.reward_type == "navigation_efficiency":
            return self._navigation_efficiency_reward(state, action, next_state, done, info)
        elif self.reward_type == "safety_focused":
            return self._safety_focused_reward(state, action, next_state, done, info)
        elif self.reward_type == "exploration":
            return self._exploration_reward(state, action, next_state, done, info)
        else:
            self.logger.warning(f"Unknown reward type: {self.reward_type}, using default")
            return self._default_reward(state, action, next_state, done, info)

    def _navigation_efficiency_reward(self,
                                    state: Dict[str, Any],
                                    action: Dict[str, Any],
                                    next_state: Dict[str, Any],
                                    done: bool,
                                    info: Dict[str, Any]) -> float:
        """Reward based on navigation efficiency."""
        reward = 0.0

        # Distance to goal (if available)
        if "goal_distance" in info:
            goal_distance = info["goal_distance"]
            prev_goal_distance = info.get("prev_goal_distance", goal_distance)
            reward += (prev_goal_distance - goal_distance) * 10.0  # Progress toward goal

        # Speed efficiency
        rover_state = next_state.get("rover", {})
        velocity = rover_state.get("velocity", [0, 0, 0])
        speed = np.linalg.norm(velocity[:2])  # XY plane speed

        if 0.5 <= speed <= 2.0:  # Optimal speed range
            reward += speed * 2.0
        elif speed > 2.0:  # Too fast
            reward -= (speed - 2.0) * 5.0

        # Network reliability
        network_state = next_state.get("network", {})
        packet_loss = network_state.get("packet_loss_percent", 0.0)
        if packet_loss > 5.0:
            reward -= packet_loss * 0.1

        # Terrain difficulty penalty
        env_state = next_state.get("environment", {})
        terrain_difficulty = env_state.get("terrain_difficulty", 0.0)
        reward -= terrain_difficulty * 2.0

        # Termination rewards
        if done:
            if info.get("success", False):
                reward += 100.0  # Goal reached
            elif info.get("collision", False):
                reward -= 50.0  # Collision penalty
            elif info.get("timeout", False):
                reward -= 25.0  # Timeout penalty

        return reward

    def _safety_focused_reward(self,
                              state: Dict[str, Any],
                              action: Dict[str, Any],
                              next_state: Dict[str, Any],
                              done: bool,
                              info: Dict[str, Any]) -> float:
        """Reward focused on safety and collision avoidance."""
        reward = 0.0

        # Collision avoidance
        if not info.get("collision", False):
            reward += 1.0  # Small reward for not crashing
        else:
            reward -= 100.0  # Large penalty for collision

        # Proximity to obstacles (if available)
        if "obstacle_distance" in info:
            obstacle_dist = info["obstacle_distance"]
            if obstacle_dist < 1.0:
                reward -= (1.0 - obstacle_dist) * 20.0  # Close obstacle penalty

        # Stability reward
        imu_data = next_state.get("sensors", {}).get("imu", {})
        angular_velocity = imu_data.get("angular_velocity", [0, 0, 0])
        stability = 1.0 - min(np.linalg.norm(angular_velocity), 5.0) / 5.0
        reward += stability * 2.0

        # GPS quality bonus
        gps_data = next_state.get("sensors", {}).get("gps", {})
        gps_quality = gps_data.get("fix_quality", 0)
        reward += gps_quality * 0.5

        return reward

    def _exploration_reward(self,
                           state: Dict[str, Any],
                           action: Dict[str, Any],
                           next_state: Dict[str, Any],
                           done: bool,
                           info: Dict[str, Any]) -> float:
        """Reward for exploration and coverage."""
        reward = 0.0

        # Coverage reward (area explored)
        if "coverage_increase" in info:
            reward += info["coverage_increase"] * 10.0

        # Novelty bonus (visiting new areas)
        if info.get("new_area_discovered", False):
            reward += 5.0

        # Information gain
        if "information_gain" in info:
            reward += info["information_gain"] * 2.0

        # Small movement penalty to encourage efficient exploration
        movement = np.linalg.norm([
            next_state.get("rover", {}).get("position", [0, 0, 0])[i] -
            state.get("rover", {}).get("position", [0, 0, 0])[i]
            for i in range(2)
        ])
        reward -= movement * 0.1

        return reward

    def _default_reward(self,
                       state: Dict[str, Any],
                       action: Dict[str, Any],
                       next_state: Dict[str, Any],
                       done: bool,
                       info: Dict[str, Any]) -> float:
        """Default reward function."""
        # Simple survival reward
        reward = 1.0  # Small reward for continuing

        if done:
            if info.get("success", False):
                reward += 10.0
            else:
                reward -= 10.0

        return reward


class RLDataRecorder(DataRecorder):
    """Enhanced data recorder for RL training and evaluation."""

    def __init__(self):
        """Initialize RL data recorder."""
        super().__init__()

        self.logger = get_simulation_logger(__name__, "rl_recorder")

        # RL-specific data
        self.episodes: List[RLEpisode] = []
        self.current_episode: Optional[RLEpisode] = None
        self.episode_counter = 0

        # RL configuration
        self.reward_function = RewardFunction()
        self.episode_timeout = 300  # 5 minutes max
        self.max_episodes = 1000

        # Training metrics
        self.training_metrics = {
            "total_episodes": 0,
            "total_steps": 0,
            "average_reward": 0.0,
            "average_episode_length": 0.0,
            "success_rate": 0.0,
            "best_reward": float('-inf'),
        }

    def initialize(self, config: Dict[str, Any]):
        """Initialize recorder with RL-specific configuration.

        Args:
            config: Recording configuration with RL parameters
        """
        super().initialize(config)

        # RL-specific configuration
        rl_config = config.get("rl", {})
        reward_type = rl_config.get("reward_function", "navigation_efficiency")
        self.reward_function = RewardFunction(reward_type)

        self.episode_timeout = rl_config.get("episode_timeout", 300)
        self.max_episodes = rl_config.get("max_episodes", 1000)

        self.logger.info("RL Data Recorder initialized",
                        reward_function=reward_type,
                        episode_timeout=self.episode_timeout)

    def start_episode(self, metadata: Optional[Dict[str, Any]] = None) -> int:
        """Start a new RL training episode.

        Args:
            metadata: Additional episode metadata

        Returns:
            Episode ID
        """
        if self.current_episode is not None:
            self.end_episode("interrupted", success=False)

        episode_id = self.episode_counter
        self.episode_counter += 1

        self.current_episode = RLEpisode(
            episode_id=episode_id,
            start_time=time.time(),
            end_time=0.0,
            steps=[],
            total_reward=0.0,
            episode_length=0,
            success=False,
            termination_reason="",
            metadata=metadata or {}
        )

        self.logger.info("Started RL episode", episode_id=episode_id)
        return episode_id

    def record_rl_step(self,
                      state: Dict[str, Any],
                      action: Dict[str, Any],
                      next_state: Dict[str, Any],
                      done: bool,
                      info: Optional[Dict[str, Any]] = None,
                      custom_reward: Optional[float] = None) -> RLStep:
        """Record a single RL training step.

        Args:
            state: Current state
            action: Action taken
            next_state: Next state
            done: Whether episode ended
            info: Additional information
            custom_reward: Override calculated reward

        Returns:
            RLStep object
        """
        if self.current_episode is None:
            raise RuntimeError("No active episode. Call start_episode() first.")

        info = info or {}

        # Calculate reward if not provided
        if custom_reward is not None:
            reward = custom_reward
        else:
            reward = self.reward_function.calculate_reward(
                state, action, next_state, done, info
            )

        # Create RL step
        step = RLStep(
            timestamp=time.time(),
            episode=self.current_episode.episode_id,
            step=self.current_episode.episode_length,
            state=self._extract_rl_state(state),
            action=action,
            reward=reward,
            next_state=self._extract_rl_state(next_state),
            done=done,
            info=info,
            simulation_time=state.get("timestamp", 0.0)
        )

        # Add to current episode
        self.current_episode.steps.append(step)
        self.current_episode.total_reward += reward
        self.current_episode.episode_length += 1

        # Check for episode termination
        if done:
            termination_reason = "unknown"
            success = False

            if info.get("success", False):
                termination_reason = "goal_reached"
                success = True
            elif info.get("collision", False):
                termination_reason = "collision"
            elif info.get("timeout", False):
                termination_reason = "timeout"
            elif self.current_episode.episode_length >= self.episode_timeout:
                termination_reason = "max_steps"

            self.end_episode(termination_reason, success)

        # Also record in base format for compatibility
        self.record({
            "rl_step": asdict(step),
            "simulation_state": next_state
        })

        self.logger.log_rl_step(
            episode=step.episode,
            step=step.step,
            reward=step.reward,
            action_type=action.get("type", "unknown"),
            done=done
        )

        return step

    def end_episode(self, termination_reason: str, success: bool = False):
        """End the current episode.

        Args:
            termination_reason: Reason for episode termination
            success: Whether episode was successful
        """
        if self.current_episode is None:
            return

        self.current_episode.end_time = time.time()
        self.current_episode.success = success
        self.current_episode.termination_reason = termination_reason

        # Move to completed episodes
        self.episodes.append(self.current_episode)
        episode = self.current_episode
        self.current_episode = None

        # Update training metrics
        self._update_training_metrics(episode)

        self.logger.info("Ended RL episode",
                        episode_id=episode.episode_id,
                        total_reward=episode.total_reward,
                        episode_length=episode.episode_length,
                        success=success,
                        termination_reason=termination_reason)

    def _extract_rl_state(self, simulation_state: Dict[str, Any]) -> Dict[str, Any]:
        """Extract RL-relevant state from simulation state.

        Args:
            simulation_state: Full simulation state

        Returns:
            RL-focused state representation
        """
        rl_state = {}

        # Position and orientation (core navigation state)
        rover = simulation_state.get("rover", {})
        rl_state["position"] = rover.get("position", [0.0, 0.0, 0.0])
        rl_state["orientation"] = rover.get("orientation", [0.0, 0.0, 0.0])
        rl_state["velocity"] = rover.get("velocity", [0.0, 0.0, 0.0])

        # Sensor data
        sensors = simulation_state.get("sensors", {})
        rl_state["gps"] = {
            "fix_quality": sensors.get("gps", {}).get("fix_quality", 0),
            "satellites_visible": sensors.get("gps", {}).get("satellites_visible", 0),
            "hdop": sensors.get("gps", {}).get("hdop", 50.0),
        }
        rl_state["imu"] = {
            "linear_acceleration": sensors.get("imu", {}).get("linear_acceleration", [0.0, 0.0, 0.0]),
            "angular_velocity": sensors.get("imu", {}).get("angular_velocity", [0.0, 0.0, 0.0]),
        }

        # Environmental factors
        environment = simulation_state.get("environment", {})
        rl_state["environment"] = {
            "terrain_difficulty": environment.get("terrain_difficulty", 0.0),
            "visibility": environment.get("visibility", 1.0),
            "wind_speed": environment.get("wind_speed", 0.0),
            "dust_density": environment.get("dust_density", 0.0),
        }

        # Network conditions
        network = simulation_state.get("network", {})
        rl_state["network"] = {
            "latency_ms": network.get("latency_ms", 0.0),
            "packet_loss_percent": network.get("packet_loss_percent", 0.0),
            "connection_status": network.get("connection_status", True),
        }

        # Time information
        rl_state["simulation_time"] = simulation_state.get("timestamp", 0.0)

        return rl_state

    def _update_training_metrics(self, episode: RLEpisode):
        """Update training metrics with completed episode.

        Args:
            episode: Completed episode
        """
        self.training_metrics["total_episodes"] += 1
        self.training_metrics["total_steps"] += episode.episode_length

        # Rolling averages
        total_episodes = self.training_metrics["total_episodes"]
        self.training_metrics["average_reward"] = (
            (self.training_metrics["average_reward"] * (total_episodes - 1)) +
            episode.total_reward
        ) / total_episodes

        self.training_metrics["average_episode_length"] = (
            (self.training_metrics["average_episode_length"] * (total_episodes - 1)) +
            episode.episode_length
        ) / total_episodes

        # Success rate
        success_count = sum(1 for ep in self.episodes if ep.success)
        self.training_metrics["success_rate"] = success_count / total_episodes

        # Best reward
        self.training_metrics["best_reward"] = max(
            self.training_metrics["best_reward"],
            episode.total_reward
        )

    def get_rl_statistics(self) -> Dict[str, Any]:
        """Get RL training statistics.

        Returns:
            Dictionary with RL training metrics
        """
        stats: Dict[str, Any] = self.training_metrics.copy()

        if self.episodes:
            # Recent episodes (last 10)
            recent_episodes = self.episodes[-10:]
            stats["recent_average_reward"] = np.mean([ep.total_reward for ep in recent_episodes])
            stats["recent_success_rate"] = np.mean([ep.success for ep in recent_episodes])

            # Episode length distribution
            episode_lengths = [ep.episode_length for ep in self.episodes]
            stats["episode_length_stats"] = {
                "mean": np.mean(episode_lengths),
                "std": np.std(episode_lengths),
                "min": min(episode_lengths),
                "max": max(episode_lengths),
            }

        return stats

    def save_rl_dataset(self, filepath: str, format: str = "json") -> bool:
        """Save RL training dataset.

        Args:
            filepath: Path to save dataset
            format: Dataset format ("json", "pickle")

        Returns:
            bool: True if saved successfully
        """
        try:
            dataset = {
                "metadata": {
                    "created_at": time.time(),
                    "total_episodes": len(self.episodes),
                    "reward_function": self.reward_function.reward_type,
                    "training_metrics": self.training_metrics,
                },
                "episodes": [asdict(episode) for episode in self.episodes],
            }

            path = Path(filepath)
            path.parent.mkdir(parents=True, exist_ok=True)

            if format == "json":
                import json
                with open(path, "w") as f:
                    json.dump(dataset, f, indent=2, default=str)
            elif format == "pickle":
                import pickle
                with open(path, "wb") as f:
                    pickle.dump(dataset, f)
            else:
                raise ValueError(f"Unsupported format: {format}")

            self.logger.info("RL dataset saved", filepath=str(path), format=format)
            return True

        except Exception as e:
            self.logger.error("Failed to save RL dataset", error=str(e))
            return False

    def get_episode_data(self, episode_id: int) -> Optional[RLEpisode]:
        """Get data for specific episode.

        Args:
            episode_id: Episode ID to retrieve

        Returns:
            RLEpisode if found, None otherwise
        """
        for episode in self.episodes:
            if episode.episode_id == episode_id:
                return episode
        return None
