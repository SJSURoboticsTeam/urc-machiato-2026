#!/usr/bin/env python3
"""
Demo: Autonomy Response to Teleoperation Data
Shows how autonomy reacts to different teleoperation scenarios
"""

import time


def simulate_scenario(scenario_name, description):
    """Simulate a teleoperation data scenario and show autonomy response"""
    print(f"\n Scenario: {scenario_name}")
    print(f" {description}")
    print("-" * 50)

    # Simulate different data conditions
    if scenario_name == "Normal Operation":
        temp_data = [35.0, 36.0, 34.0, 35.5]  # Normal temperatures
        battery_pct = 78.0
        motor_velocities = [1.2, -0.8, 0.5, -1.1]

    elif scenario_name == "Thermal Stress":
        temp_data = [68.0, 72.0, 65.0, 70.0]  # High temperatures
        battery_pct = 75.0
        motor_velocities = [2.1, -1.5, 1.8, -2.0]

    elif scenario_name == "Battery Critical":
        temp_data = [42.0, 41.0, 43.0, 40.0]  # Normal temperatures
        battery_pct = 8.0  # Critical battery
        motor_velocities = [0.8, -0.6, 0.3, -0.9]

    elif scenario_name == "Motor Issues":
        temp_data = [38.0, 85.0, 37.0, 39.0]  # One motor overheating
        battery_pct = 65.0
        motor_velocities = [15.0, 0.0, 0.0, 0.0]  # Velocity spike on one motor

    # Show the data
    print("[GRAPH] Teleoperation Data Received:")
    print(".1f")
    print("   Motor Temperatures: [°C]")
    for i, temp in enumerate(temp_data):
        status = " HIGH" if temp > 60 else "[PASS] OK"
        print(f"     Motor {i}: {temp:.1f}°C {status}")
    print("   Motor Velocities: [rad/s]")
    for i, vel in enumerate(motor_velocities):
        status = "  SPIKE" if abs(vel) > 10 else "[PASS] OK"
        print(f"     Motor {i}: {vel:.2f} rad/s {status}")

    # Show autonomy response
    print("\n Autonomy Response:")

    # Temperature analysis
    max_temp = max(temp_data)
    avg_temp = sum(temp_data) / len(temp_data)

    if max_temp > 70:
        print("    CRITICAL: Emergency thermal shutdown initiated!")
        print("    Stopping all motors immediately")
        print("   [LIGHTNING] Reducing power consumption")
    elif max_temp > 55:
        print("    Thermal stress detected - applying speed limits")
        print("    Reducing velocity by 30% to prevent overheating")
        print(".1f")
    else:
        print("   [PASS] Temperatures within normal range")

    # Battery analysis
    if battery_pct < 10:
        print("    CRITICAL: Emergency battery level!")
        print("    Initiating immediate return to base")
        print("    Switching to power-saving navigation mode")
    elif battery_pct < 20:
        print("    Low battery - enabling conservation mode")
        print("    Simplifying mission path")
        print("    Reducing speed by 20%")
    else:
        print("   [PASS] Battery level acceptable")

    # Motor analysis
    velocity_issues = [i for i, vel in enumerate(motor_velocities) if abs(vel) > 10]
    if velocity_issues:
        print("    Motor velocity anomalies detected")
        for motor_idx in velocity_issues:
            print(
                f"   [TOOL] Motor {motor_idx}: velocity spike ({motor_velocities[motor_idx]:.1f} rad/s)"
            )
        print("   [MAGNIFY] Initiating motor health check")
        print("    Enabling redundant motor compensation")

    # Overall decision
    print("\n[OBJECTIVE] Overall Autonomy Decision:")
    critical_issues = max_temp > 70 or battery_pct < 10 or len(velocity_issues) > 0
    moderate_issues = max_temp > 55 or battery_pct < 20

    if critical_issues:
        print("    CRITICAL MODE: Safety protocols activated")
        print("    Mission suspended - emergency procedures")
        print("    Human operator notification sent")
    elif moderate_issues:
        print("    CAUTION MODE: Adaptive behavior activated")
        print("   [REFRESH] Mission continuing with modifications")
        print("   [GRAPH] Continuous monitoring enabled")
    else:
        print("   [PASS] NORMAL MODE: Full autonomy active")
        print("   [IGNITE] Mission proceeding normally")
        print("    Performance optimization active")


def main():
    """Run the autonomy response demo"""
    print(" Autonomy Response to Teleoperation Data Demo")
    print("=" * 60)
    print("This demo shows how autonomy reacts to different system conditions")
    print("based on teleoperation data (temperatures, battery, motor velocities)")
    print()

    scenarios = [
        ("Normal Operation", "All systems operating within normal parameters"),
        ("Thermal Stress", "Motors running hot due to continuous operation"),
        (
            "Battery Critical",
            "Battery level dangerously low, needs immediate attention",
        ),
        ("Motor Issues", "One motor showing velocity spikes and overheating"),
    ]

    for scenario_name, description in scenarios:
        simulate_scenario(scenario_name, description)
        time.sleep(2)  # Pause between scenarios

    print("\n" + "=" * 60)
    print("[PARTY] Demo Complete!")
    print()
    print("Key Takeaways:")
    print("• [PASS] Autonomy validates all incoming teleoperation data")
    print("•  Makes intelligent decisions based on system health")
    print("• [REFRESH] Adapts behavior dynamically to changing conditions")
    print("•  Prioritizes safety with emergency shutdown capabilities")
    print("• [GRAPH] Provides detailed logging for monitoring and debugging")
    print()
    print("Ready for real teleoperation integration! [IGNITE]")


if __name__ == "__main__":
    main()
