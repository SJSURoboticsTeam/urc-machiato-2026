#!/usr/bin/env python3
"""
RL Training Example with Enhanced Logging, Monitoring, and Tracing

This example demonstrates how to use the enhanced simulation framework
for RL training with comprehensive logging, real-time monitoring,
and performance tracing.

Features demonstrated:
- Structured logging with RL metrics
- Real-time performance monitoring
- Comprehensive tracing for debugging
- RL-specific data recording and analysis
- Curriculum learning across environment tiers

Author: URC 2026 Autonomy Team
"""

import time
from pathlib import Path

import numpy as np

from simulation import SimulationManager
from simulation.core.logging_config import setup_simulation_logging
from simulation.tools.rl_data_recorder import RLDataRecorder


def create_rl_training_config():
    """Create comprehensive RL training configuration."""
    return {
        # Enhanced logging configuration
        "logging": {
            "enabled": True,
            "level": "INFO",
            "file": "rl_training.log",
            "structured": True,
            "json_format": False,
        },
        # Monitoring configuration
        "monitoring": {
            "enabled": True,
            "interval": 1.0,  # Monitor every second
        },
        # Tracing configuration
        "tracing": {
            "enabled": True,
            "auto_profile_threshold": 0.1,  # Log operations > 100ms
        },
        # Environment curriculum (starts easy, gets harder)
        "environment": {
            "tier": "real_life",  # Will be modified during curriculum
            "randomize_conditions": True,
        },
        # Sensor suite
        "sensors": [
            {"name": "gps", "type": "gps", "position_noise_std": 2.0},
            {"name": "imu", "type": "imu", "bias_drift": True},
        ],
        # Network conditions (affects communication reliability)
        "network": {
            "profile": "rural_wifi",  # Will vary during training
        },
        # Rover configuration
        "rover": {
            "model": "urc_rover",
            "physics_enabled": True,
            "failure_modes": ["wheel_stuck", "sensor_failure"],
        },
        # Time configuration
        "time": {
            "step_size": 0.1,  # 10Hz simulation
            "real_time_factor": 1.0,
        },
        # RL-specific recording
        "recording": {
            "format": "rl_dataset",
            "include_rewards": True,
            "state_compression": "minimal",
            "record_interval": 0.1,  # 10Hz recording
        },
        # RL training parameters
        "rl": {
            "reward_function": "navigation_efficiency",
            "episode_timeout": 300,  # 5 minutes max per episode
            "max_episodes": 100,
            "curriculum_learning": True,
        },
    }


class SimpleRLAgent:
    """Simple RL agent for demonstration."""

    def __init__(self):
        """Initialize RL agent."""
        self.logger = setup_simulation_logging()
        self.logger.info("Initializing RL agent")

        # Simple policy: random actions with learning
        self.action_space = ["forward", "turn_left", "turn_right", "stop"]
        self.learning_rate = 0.1
        self.epsilon = 0.1  # Exploration rate

        # Simple Q-table (state -> action values)
        self.q_table = {}
        self.episode_rewards = []

    def get_state_key(self, state):
        """Convert simulation state to discrete key for Q-learning."""
        # Extract relevant features for state representation
        rover = state.get("rover", {})
        position = rover.get("position", [0, 0, 0])

        # Discretize position (coarse grid)
        grid_size = 10.0
        x_grid = int(position[0] / grid_size)
        y_grid = int(position[1] / grid_size)

        # GPS quality
        gps_quality = state.get("sensors", {}).get("gps", {}).get("fix_quality", 0)

        # Network latency (binned)
        network_latency = state.get("network", {}).get("latency_ms", 0)
        latency_bin = "low" if network_latency < 100 else "high"

        return f"{x_grid}_{y_grid}_gps{gps_quality}_net{latency_bin}"

    def choose_action(self, state):
        """Choose action using epsilon-greedy policy."""
        state_key = self.get_state_key(state)

        # Exploration
        if np.random.random() < self.epsilon:
            action = np.random.choice(self.action_space)
            action_type = "explore"
        else:
            # Exploitation - choose best action
            if state_key in self.q_table:
                action = max(self.q_table[state_key], key=self.q_table[state_key].get)
            else:
                action = np.random.choice(self.action_space)
            action_type = "exploit"

        # Convert to action format expected by simulation
        action_data = {"type": action, "timestamp": time.time()}

        return action_data, action_type

    def update_q_value(self, state, action, reward, next_state):
        """Update Q-value using Q-learning."""
        state_key = self.get_state_key(state)
        next_state_key = self.get_state_key(next_state)
        action_name = action["type"]

        # Initialize Q-values if needed
        if state_key not in self.q_table:
            self.q_table[state_key] = {a: 0.0 for a in self.action_space}
        if next_state_key not in self.q_table:
            self.q_table[next_state_key] = {a: 0.0 for a in self.action_space}

        # Q-learning update
        current_q = self.q_table[state_key][action_name]
        max_next_q = max(self.q_table[next_state_key].values())

        # Q(s,a) = Q(s,a) + α[r + γ*max(Q(s',a')) - Q(s,a)]
        new_q = current_q + self.learning_rate * (reward + 0.9 * max_next_q - current_q)
        self.q_table[state_key][action_name] = new_q


def curriculum_schedule(episode: int) -> dict:
    """Define curriculum learning schedule."""
    if episode < 20:
        # Easy: Perfect conditions
        return {
            "environment": {"tier": "perfect"},
            "network": {"profile": "perfect"},
        }
    elif episode < 50:
        # Medium: Real-life conditions
        return {
            "environment": {"tier": "real_life"},
            "network": {"profile": "rural_wifi"},
        }
    else:
        # Hard: Extreme conditions
        return {
            "environment": {"tier": "extreme"},
            "network": {"profile": "extreme"},
        }


def run_rl_training():
    """Run RL training with enhanced monitoring and logging."""
    print("🚀 Starting RL Training with Enhanced Monitoring")
    print("=" * 60)

    # Setup logging
    setup_simulation_logging(
        log_level="INFO", log_file="rl_training.log", enable_structured=True
    )

    # Create simulation with RL configuration
    config = create_rl_training_config()
    sim_manager = SimulationManager()

    # Replace default recorder with RL recorder
    rl_recorder = RLDataRecorder()
    sim_manager.data_recorder = rl_recorder

    # Initialize simulation
    if not sim_manager.initialize(config):
        print("❌ Failed to initialize simulation")
        return

    # Create RL agent
    agent = SimpleRLAgent()

    print("🎯 Starting RL training loop...")
    print("- Episodes: Curriculum learning (Easy → Medium → Hard)")
    print("- Monitoring: Real-time performance tracking")
    print("- Tracing: Detailed operation profiling")
    print("- Logging: Structured logs with RL metrics")
    print()

    # Training loop
    max_episodes = config["rl"]["max_episodes"]
    episode_timeout = config["rl"]["episode_timeout"]

    for episode in range(max_episodes):
        print(f"\n📊 Episode {episode + 1}/{max_episodes}")

        # Curriculum learning: adjust difficulty
        curriculum_config = curriculum_schedule(episode)
        if curriculum_config:
            print(f"🎓 Curriculum: {curriculum_config}")

            # Update environment
            if "environment" in curriculum_config:
                env_config = curriculum_config["environment"]
                sim_manager.environment = sim_manager.environment.__class__(env_config)
                print(f"   Environment: {env_config.get('tier', 'default')}")

            # Update network
            if "network" in curriculum_config:
                net_config = curriculum_config["network"]
                sim_manager.network = sim_manager.network.__class__(net_config)
                print(f"   Network: {net_config.get('profile', 'default')}")

        # Start new episode
        episode_id = rl_recorder.start_episode(
            metadata={
                "curriculum_phase": "easy"
                if episode < 20
                else "medium"
                if episode < 50
                else "hard",
                "episode": episode,
            }
        )

        episode_reward = 0
        episode_steps = 0
        episode_start_time = time.time()

        # Reset simulation for new episode
        sim_manager.reset()

        # Episode loop
        while episode_steps < episode_timeout:
            try:
                # Get current state
                current_state = sim_manager.get_state()

                # Agent chooses action
                action, action_type = agent.choose_action(current_state)

                # Execute action in simulation
                dt = config["time"]["step_size"]
                next_state = sim_manager.step(dt)

                # Calculate reward and check termination
                reward = rl_recorder.reward_function.calculate_reward(
                    current_state, action, next_state, False, {}
                )

                # Check episode termination conditions
                done = False
                termination_reason = ""

                # Goal reached (simple distance check)
                rover_pos = next_state.get("rover", {}).get("position", [0, 0, 0])
                distance_from_start = np.linalg.norm(rover_pos[:2])
                if distance_from_start > 50.0:  # Moved 50m from start
                    done = True
                    termination_reason = "goal_reached"
                    reward += 100  # Goal bonus

                # Collision detection (simple bumper)
                elif (
                    next_state.get("environment", {}).get("terrain_difficulty", 0) > 0.8
                ):
                    done = True
                    termination_reason = "collision"
                    reward -= 50

                # Timeout
                elif episode_steps >= episode_timeout - 1:
                    done = True
                    termination_reason = "timeout"
                    reward -= 25

                # Record RL step
                rl_recorder.record_rl_step(
                    current_state,
                    action,
                    next_state,
                    done,
                    info={"termination_reason": termination_reason},
                    custom_reward=reward,
                )

                # Update agent
                if not done:
                    agent.update_q_value(current_state, action, reward, next_state)

                episode_reward += reward
                episode_steps += 1

                # Progress logging
                if episode_steps % 100 == 0:
                    print(".2f" ".3f")

                if done:
                    break

            except Exception as e:
                print(f"❌ Episode {episode + 1} error: {e}")
                rl_recorder.end_episode("error", success=False)
                break

        # End episode
        episode_duration = time.time() - episode_start_time
        success = episode_steps < episode_timeout and distance_from_start > 50.0
        rl_recorder.end_episode(termination_reason, success=success)

        # Episode summary
        print(".3f" ".1f" ".3f")

        # Save checkpoint every 10 episodes
        if (episode + 1) % 10 == 0:
            checkpoint_file = f"rl_training_checkpoint_episode_{episode + 1}.json"
            rl_recorder.save_rl_dataset(checkpoint_file)
            print(f"💾 Checkpoint saved: {checkpoint_file}")

        # Performance monitoring
        if episode > 0 and episode % 5 == 0:
            stats = sim_manager.get_statistics()
            monitoring = stats.get("monitoring", {})
            tracing = stats.get("tracing", {})

            print("📈 Performance Summary:")
            if monitoring:
                print(".1f")
            if tracing:
                slow_ops = tracing.get("slowest_operations", [])
                if slow_ops:
                    slowest = slow_ops[0]
                    print(".3f")

    print("\n🎉 RL Training Complete!")
    print("=" * 60)

    # Final statistics
    final_stats = rl_recorder.get_rl_statistics()
    print("📊 Final RL Statistics:")
    print(".1f")
    print(".2f")
    print(".1f")
    print(".3f")

    # Save final dataset
    final_dataset = "rl_training_final_dataset.json"
    rl_recorder.save_rl_dataset(final_dataset)
    print(f"💾 Final dataset saved: {final_dataset}")

    # Export monitoring data
    monitoring_data = "rl_training_monitoring.json"
    sim_manager.monitor.export_metrics(monitoring_data)
    print(f"📊 Monitoring data exported: {monitoring_data}")

    # Export traces
    trace_data = "rl_training_traces.json"
    sim_manager.tracer.export_traces(trace_data)
    print(f"🔍 Trace data exported: {trace_data}")

    # Cleanup
    sim_manager.monitor.stop_monitoring()
    sim_manager.stop()

    print("\n✅ Training complete! Check the log files and exported data for analysis.")


if __name__ == "__main__":
    run_rl_training()
