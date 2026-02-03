#!/usr/bin/env python3
"""
Simple Swerve Drive Simulator
Lightweight simulation for testing CAN bridge before real hardware.

Simulates 8 swerve motors (4 drive + 4 steer) with basic kinematics.
"""

import asyncio
import time
import json
import threading
from typing import Dict, List, Tuple, Any
from dataclasses import dataclass


@dataclass
class SwerveModuleState:
    """Individual swerve module state"""

    angle: float = 0.0  # Steering angle in radians
    speed: float = 0.0  # Drive speed in m/s
    target_angle: float = 0.0
    target_speed: float = 0.0


class SwerveDriveSimulator:
    def __init__(self):
        self.modules = {
            "fl_steer": SwerveModuleState(),
            "fl_drive": SwerveModuleState(),
            "fr_steer": SwerveModuleState(),
            "fr_drive": SwerveModuleState(),
            "rl_steer": SwerveModuleState(),
            "rl_drive": SwerveModuleState(),
            "rr_steer": SwerveModuleState(),
            "rr_drive": SwerveModuleState(),
        }

        self.wheelbase_width = 0.5  # meters
        self.update_rate = 50  # Hz
        self.running = True

    def update_swerve_kinematics(self):
        """Update module positions based on swerve kinematics"""
        # Simple first-order kinematics approximation
        for name, module in self.modules.items():
            # Update angle
            angle_error = module.target_angle - module.angle
            module.angle += angle_error * 0.1  # 10% update rate

            # Update speed
            speed_error = module.target_speed - module.speed
            module.speed += speed_error * 0.1  # 10% update rate

            # Add some simulated noise
            module.angle += (time.time() % 100) * 0.001
            module.speed += (time.time() % 100) * 0.01

    def get_robot_velocity(self) -> Tuple[float, float, float]:
        """Calculate overall robot velocity from module states"""
        # Simplified kinematics - average drive components
        vx = 0.0
        vy = 0.0
        omega = 0.0
        pitch = 0.0

        for drive_name in ["fl_drive", "fr_drive", "rl_drive", "rr_drive"]:
            if drive_name in self.modules:
                drive_module = self.modules[drive_name]

                # Calculate contribution to robot velocity
                # Front wheels: angle=0, contributes to forward motion
                # Rear wheels: angle=0, contributes to forward motion
                # Side components contribute to lateral motion and rotation

                if "fl" in drive_name:  # Front left
                    vx += drive_module.speed * 0.707  # cos(45°)
                    vy += drive_module.speed * 0.707  # sin(45°)
                elif "fr" in drive_name:  # Front right
                    vx += drive_module.speed * 0.707
                    vy -= drive_module.speed * 0.707
                elif "rl" in drive_name:  # Rear left
                    vx += drive_module.speed * 0.5  # cos(60°)
                    vy += drive_module.speed * 0.866  # sin(60°)
                elif "rr" in drive_name:  # Rear right
                    vx += drive_module.speed * 0.5  # cos(60°)
                    vy -= drive_module.speed * 0.866  # sin(60°)

        # Simplified rotation from steering (average of steering angles)
        steering_angles = [
            self.modules["fl_steer"].angle,
            self.modules["fr_steer"].angle,
            self.modules["rl_steer"].angle,
            self.modules["rr_steer"].angle,
        ]
        avg_steering = sum(steering_angles) / 4.0

        omega = avg_steering * 0.5  # Simple approximation

        return vx, vy, omega

    def simulate_step(self):
        """Single simulation step"""
        self.update_swerve_kinematics()

        # Update all modules toward targets
        for name, module in self.modules.items():
            if abs(module.angle - module.target_angle) > 0.01:
                print(
                    f"🔄 {name}: adjusting angle {module.angle:.3f}→{module.target_angle:.3f}"
                )
            if abs(module.speed - module.target_speed) > 0.05:
                print(
                    f"⚡ {name}: adjusting speed {module.speed:.2f}→{module.target_speed:.2f}"
                )

    async def run_simulation(self):
        """Run simulation loop"""
        print("🚗 Starting Swerve Drive Simulator...")
        print("   8 motors (4 drive + 4 steer)")
        print("   Update rate: 50Hz")
        print("   Press Ctrl+C to stop")
        print()

        step_count = 0

        while self.running:
            start_time = time.time()

            self.simulate_step()
            step_count += 1

            # Print status every second
            if step_count % 50 == 0:
                vx, vy, omega = self.get_robot_velocity()
                speed = (vx**2 + vy**2) ** 0.5

                print(
                    f"📊 Robot: vx={vx:.2f}m/s, vy={vy:.2f}m/s, ω={omega:.3f}rad/s, speed={speed:.2f}m/s"
                )
                print("   Modules: ", end="")
                for name, module in self.modules.items():
                    print(f"{name}({module.angle:.1f}°,{module.speed:.1f}) ", end="")
                print()
                print()

            # Maintain update rate
            elapsed = time.time() - start_time
            if elapsed < 0.02:  # 50Hz = 20ms period
                await asyncio.sleep(0.02 - elapsed)


def main():
    simulator = SwerveDriveSimulator()

    try:
        asyncio.run(simulator.run_simulation())
    except KeyboardInterrupt:
        print("\n👋 Simulator stopped")


if __name__ == "__main__":
    main()
