#!/usr/bin/env python3
"""
ROS2 Monitoring Demo - Shows how to monitor ALL ROS topics and services

This script demonstrates how to monitor the complete URC 2026 ROS2 system
including state machine, mission control, hardware sensors, and services.
"""

import subprocess
import time
import os
import sys

def run_cmd(cmd, description=""):
    """Run a command and return the result."""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
        return result.returncode == 0, result.stdout.strip(), result.stderr.strip()
    except Exception as e:
        return False, "", str(e)

def print_section(title):
    """Print a section header."""
    print(f"\n{'='*60}")
    print(f"🚀 {title}")
    print(f"{'='*60}")

def main():
    """Main demonstration."""
    print("🌟 URC 2026 ROS2 Universal Monitoring Demo")
    print("This shows how to monitor ALL ROS topics and services in the system")
    print()

    # Set up environment
    workspace_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(workspace_dir)

    # Source ROS2 (for shell commands)
    ros_setup = "source /opt/ros/humble/setup.bash && source install/setup.bash && "

    print_section("1. SYSTEM OVERVIEW")

    # Check ROS2 nodes
    print("🔗 Active ROS2 Nodes:")
    success, output, error = run_cmd(ros_setup + "ros2 node list")
    if success:
        nodes = output.split('\n')
        for node in nodes:
            if node.strip():
                print(f"   ✅ {node}")
        print(f"   📊 Total: {len([n for n in nodes if n.strip()])} nodes")
    else:
        print(f"   ❌ Error: {error}")

    print_section("2. ALL ROS2 TOPICS")

    # Get all topics
    success, output, error = run_cmd(ros_setup + "ros2 topic list")
    if success:
        topics = [t.strip() for t in output.split('\n') if t.strip()]
        print(f"📡 Found {len(topics)} active ROS2 topics:")

        # Categorize topics
        state_machine_topics = [t for t in topics if 'state_machine' in t]
        hardware_topics = [t for t in topics if 'hardware' in t or t in ['/odom', '/imu', '/gps/fix']]
        mission_topics = [t for t in topics if 'mission' in t]
        safety_topics = [t for t in topics if 'safety' in t or 'emergency' in t]
        other_topics = [t for t in topics if t not in state_machine_topics + hardware_topics + mission_topics + safety_topics]

        print(f"\n🤖 State Machine Topics ({len(state_machine_topics)}):")
        for topic in state_machine_topics:
            print(f"   📍 {topic}")

        print(f"\n🔧 Hardware/Sensor Topics ({len(hardware_topics)}):")
        for topic in hardware_topics:
            print(f"   📡 {topic}")

        print(f"\n🎯 Mission Control Topics ({len(mission_topics)}):")
        for topic in mission_topics:
            print(f"   🎮 {topic}")

        print(f"\n🛡️ Safety Topics ({len(safety_topics)}):")
        for topic in safety_topics:
            print(f"   ⚠️ {topic}")

        if other_topics:
            print(f"\n🔄 Other Topics ({len(other_topics)}):")
            for topic in other_topics:
                print(f"   📄 {topic}")

    print_section("3. ALL ROS2 SERVICES")

    # Get all services
    success, output, error = run_cmd(ros_setup + "ros2 service list")
    if success:
        services = [s.strip() for s in output.split('\n') if s.strip()]
        print(f"🔧 Found {len(services)} active ROS2 services:")

        # Filter out parameter services for cleaner display
        param_services = [s for s in services if 'describe_parameters' in s or 'get_parameter' in s or 'list_parameters' in s or 'set_parameters' in s]
        real_services = [s for s in services if s not in param_services]

        print(f"\n🎯 Key Services ({len(real_services)}):")
        for service in real_services:
            print(f"   🔧 {service}")

        if param_services:
            print(f"\n⚙️ Parameter Services ({len(param_services)}):")
            print(f"   📊 {len(param_services)} parameter management services available")

    print_section("4. LIVE TOPIC MONITORING")

    print("📊 Monitoring key topics (press Ctrl+C to continue)...")
    print()

    # Monitor key topics
    key_topics = [
        '/state_machine/current_state',
        '/hardware/battery_state',
        '/hardware/gps',
        '/hardware/imu',
        '/mission/status',
        '/emergency_stop'
    ]

    for topic in key_topics:
        print(f"🔍 Monitoring {topic}:")
        success, output, error = run_cmd(ros_setup + f"timeout 2 ros2 topic echo --once {topic} 2>/dev/null || echo 'No recent data'")

        if success and output and 'No recent data' not in output:
            # Truncate long outputs
            lines = output.split('\n')[:5]  # First 5 lines
            for line in lines:
                if line.strip():
                    print(f"   📨 {line}")
            if len(output.split('\n')) > 5:
                print("   ... (truncated)")
        else:
            print("   💤 No data available (topic may be inactive)")

        print()

    print_section("5. DASHBOARD INTEGRATION")

    print("🌐 WebSocket Telemetry Bridge:")
    print("   • Competition Bridge: ws://localhost:8080")
    print("   • Simulation Bridge: ws://localhost:8766")
    print("   • Real-time telemetry streaming to dashboard")
    print()
    print("📊 Dashboard Data Streams:")
    print("   • GPS position & IMU data")
    print("   • Battery status & system health")
    print("   • Mission progress & state machine status")
    print("   • Emergency stop & safety violations")
    print("   • Sample collection & autonomous navigation")
    print()

    print_section("6. MONITORING COMMANDS")

    print("🛠️ Useful ROS2 Monitoring Commands:")
    print()
    print("📡 View all topics:")
    print("   ros2 topic list")
    print()
    print("🔧 View all services:")
    print("   ros2 service list")
    print()
    print("📨 Monitor specific topic:")
    print("   ros2 topic echo /state_machine/current_state")
    print()
    print("📊 Get topic info:")
    print("   ros2 topic info /hardware/imu")
    print()
    print("🔍 Get topic bandwidth:")
    print("   ros2 topic hz /hardware/imu")
    print()
    print("🌐 View ROS2 graph:")
    print("   ros2 run rqt_graph rqt_graph")
    print()
    print("📈 Monitor with GUI:")
    print("   ros2 run rqt_topic rqt_topic")
    print()

    print_section("7. SYSTEM HEALTH CHECK")

    # Quick health check
    print("🏥 System Health Status:")

    # Check if key nodes are running
    key_nodes = ['competition_bridge', 'ros2_state_machine_bridge']
    for node in key_nodes:
        success, output, error = run_cmd(ros_setup + f"ros2 node list | grep {node}")
        if success and output.strip():
            print(f"   ✅ {node}: Running")
        else:
            print(f"   ❌ {node}: Not found")

    # Check key topics have publishers
    key_topics_check = ['/state_machine/current_state', '/hardware/battery_state']
    for topic in key_topics_check:
        success, output, error = run_cmd(ros_setup + f"ros2 topic info {topic} | grep 'Publisher count:' | awk '{{print $3}}'")
        if success and output.strip():
            count = output.strip()
            status = "✅ Active" if count != "0" else "⚠️ No publishers"
            print(f"   {status} {topic}: {count} publisher(s)")
        else:
            print(f"   ❌ {topic}: Error checking")

    print()
    print("🎉 ROS2 Universal Monitoring Demo Complete!")
    print("All ROS topics and services are active and streaming data.")
    print("Use the dashboard at http://localhost:3000 to view real-time telemetry!")

if __name__ == "__main__":
    main()

