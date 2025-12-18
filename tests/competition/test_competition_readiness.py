#!/usr/bin/env python3
"""
Competition Readiness Validation Script

Comprehensive testing of all URC competition components:
- Hardware interface integration
- Terrain intelligence
- Competition safety systems
- Autonomous missions
- Communication bridges

This script validates that the system is ready for competition deployment.
"""

import sys
import os
import time
import json
import subprocess
import signal
from typing import Dict, List, Any, Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import threading


class CompetitionReadinessTest(Node):
    """Test node for validating competition readiness."""

    def __init__(self):
        super().__init__('competition_readiness_test')

        # Test results
        self.test_results = {}
        self.test_start_time = time.time()

        # Subscribers for monitoring
        self.system_status_sub = self.create_subscription(
            String, '/hardware/system_status', self.system_status_callback, 10)

        self.mission_status_sub = self.create_subscription(
            String, '/mission/status', self.mission_status_callback, 10)

        self.safety_alert_sub = self.create_subscription(
            String, '/safety/alert', self.safety_alert_callback, 10)

        # Publishers for testing
        self.mission_cmd_pub = self.create_publisher(String, '/mission/commands', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)

        # Test state
        self.system_healthy = False
        self.mission_responsive = False
        self.safety_functional = False

    def system_status_callback(self, msg: String):
        """Monitor system health."""
        try:
            status = json.loads(msg.data)
            self.system_healthy = status.get('system_healthy', False)
        except:
            pass

    def mission_status_callback(self, msg: String):
        """Monitor mission system responsiveness."""
        self.mission_responsive = True

    def safety_alert_callback(self, msg: String):
        """Monitor safety system alerts."""
        self.safety_functional = True

    def run_test_suite(self) -> Dict[str, Any]:
        """Run the complete competition readiness test suite."""

        self.get_logger().info("🧪 Starting Competition Readiness Test Suite")

        # Test 1: System Startup
        self.test_results['system_startup'] = self.test_system_startup()

        # Test 2: Hardware Interface
        self.test_results['hardware_interface'] = self.test_hardware_interface()

        # Test 3: Terrain Intelligence
        self.test_results['terrain_intelligence'] = self.test_terrain_intelligence()

        # Test 4: Competition Safety
        self.test_results['competition_safety'] = self.test_competition_safety()

        # Test 5: Mission Systems
        self.test_results['mission_systems'] = self.test_mission_systems()

        # Test 6: Communication Bridge
        self.test_results['communication_bridge'] = self.test_communication_bridge()

        # Test 7: Emergency Stop
        self.test_results['emergency_stop'] = self.test_emergency_stop()

        # Test 8: Integration Test
        self.test_results['integration'] = self.test_integration()

        # Calculate overall score
        self.test_results['overall_score'] = self.calculate_overall_score()
        self.test_results['test_duration'] = time.time() - self.test_start_time

        return self.test_results

    def test_system_startup(self) -> Dict[str, Any]:
        """Test system startup and basic functionality."""
        self.get_logger().info("Testing system startup...")

        result = {
            'passed': False,
            'details': [],
            'score': 0
        }

        # Wait for system to stabilize
        time.sleep(5)

        # Check if nodes are running
        try:
            nodes = subprocess.run(['ros2', 'node', 'list'],
                                 capture_output=True, text=True, timeout=10)

            expected_nodes = [
                'hardware_interface',
                'terrain_analyzer',
                'competition_geofencing',
                'competition_bridge',
                'autonomous_keyboard_mission',
                'sample_collection_mission'
            ]

            running_nodes = nodes.stdout.strip().split('\n')
            found_nodes = [node for node in expected_nodes if any(node in rn for rn in running_nodes)]

            result['details'].append(f"Found {len(found_nodes)}/{len(expected_nodes)} expected nodes")
            result['details'].extend([f"✓ {node}" for node in found_nodes])

            if len(found_nodes) >= len(expected_nodes) * 0.8:  # 80% success rate
                result['passed'] = True
                result['score'] = 80 + (len(found_nodes) / len(expected_nodes)) * 20

        except Exception as e:
            result['details'].append(f"Node check failed: {e}")

        return result

    def test_hardware_interface(self) -> Dict[str, Any]:
        """Test hardware interface functionality."""
        self.get_logger().info("Testing hardware interface...")

        result = {
            'passed': False,
            'details': [],
            'score': 0
        }

        # Check if hardware topics are publishing
        topics_to_check = [
            '/hardware/joint_states',
            '/hardware/battery_state',
            '/hardware/system_status'
        ]

        try:
            topics = subprocess.run(['ros2', 'topic', 'list'],
                                  capture_output=True, text=True, timeout=5)

            available_topics = topics.stdout.strip().split('\n')
            found_topics = [topic for topic in topics_to_check if topic in available_topics]

            result['details'].append(f"Found {len(found_topics)}/{len(topics_to_check)} hardware topics")
            result['details'].extend([f"✓ {topic}" for topic in found_topics])

            if len(found_topics) == len(topics_to_check):
                result['passed'] = True
                result['score'] = 100
            elif len(found_topics) > 0:
                result['passed'] = True
                result['score'] = (len(found_topics) / len(topics_to_check)) * 100

        except Exception as e:
            result['details'].append(f"Topic check failed: {e}")

        return result

    def test_terrain_intelligence(self) -> Dict[str, Any]:
        """Test terrain intelligence system."""
        self.get_logger().info("Testing terrain intelligence...")

        result = {
            'passed': False,
            'details': [],
            'score': 0
        }

        # Check terrain topics
        terrain_topics = [
            '/terrain/traversability',
            '/terrain/classification'
        ]

        try:
            topics = subprocess.run(['ros2', 'topic', 'list'],
                                  capture_output=True, text=True, timeout=5)

            available_topics = topics.stdout.strip().split('\n')
            found_topics = [topic for topic in terrain_topics if topic in available_topics]

            result['details'].append(f"Found {len(found_topics)}/{len(terrain_topics)} terrain topics")

            if len(found_topics) > 0:
                result['passed'] = True
                result['score'] = (len(found_topics) / len(terrain_topics)) * 100
                result['details'].extend([f"✓ {topic}" for topic in found_topics])
            else:
                result['details'].append("No terrain topics found - may be using mock data")

        except Exception as e:
            result['details'].append(f"Terrain check failed: {e}")

        return result

    def test_competition_safety(self) -> Dict[str, Any]:
        """Test competition safety systems."""
        self.get_logger().info("Testing competition safety...")

        result = {
            'passed': False,
            'details': [],
            'score': 0
        }

        # Check safety topics
        safety_topics = [
            '/safety/boundary_violation',
            '/safety/alert'
        ]

        try:
            topics = subprocess.run(['ros2', 'topic', 'list'],
                                  capture_output=True, text=True, timeout=5)

            available_topics = topics.stdout.strip().split('\n')
            found_topics = [topic for topic in safety_topics if topic in available_topics]

            result['details'].append(f"Found {len(found_topics)}/{len(safety_topics)} safety topics")

            if len(found_topics) > 0:
                result['passed'] = True
                result['score'] = (len(found_topics) / len(safety_topics)) * 100
                result['details'].extend([f"✓ {topic}" for topic in found_topics])

                # Test emergency stop responsiveness
                self.emergency_stop_pub.publish(Bool(data=True))
                time.sleep(1)
                self.emergency_stop_pub.publish(Bool(data=False))

                result['details'].append("✓ Emergency stop commands sent")

        except Exception as e:
            result['details'].append(f"Safety check failed: {e}")

        return result

    def test_mission_systems(self) -> Dict[str, Any]:
        """Test mission execution systems."""
        self.get_logger().info("Testing mission systems...")

        result = {
            'passed': False,
            'details': [],
            'score': 0
        }

        # Test mission command responsiveness
        initial_responsive = self.mission_responsive

        # Send test commands
        test_commands = ['keyboard_typing', 'sample_collection']
        for cmd in test_commands:
            self.mission_cmd_pub.publish(String(data=f"start_{cmd}"))
            time.sleep(0.5)

        time.sleep(2)  # Wait for responses

        if self.mission_responsive or initial_responsive:
            result['passed'] = True
            result['score'] = 100
            result['details'].append("✓ Mission system responsive")
        else:
            result['details'].append("⚠ Mission system may not be fully responsive")

        return result

    def test_communication_bridge(self) -> Dict[str, Any]:
        """Test competition communication bridge."""
        self.get_logger().info("Testing communication bridge...")

        result = {
            'passed': False,
            'details': [],
            'score': 0
        }

        # Check WebSocket port
        try:
            import socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            result_ws = sock.connect_ex(('127.0.0.1', 8080))
            sock.close()

            if result_ws == 0:
                result['passed'] = True
                result['score'] = 100
                result['details'].append("✓ WebSocket server running on port 8080")
            else:
                result['details'].append("⚠ WebSocket server not accessible")

        except Exception as e:
            result['details'].append(f"WebSocket check failed: {e}")

        return result

    def test_emergency_stop(self) -> Dict[str, Any]:
        """Test emergency stop functionality."""
        self.get_logger().info("Testing emergency stop...")

        result = {
            'passed': False,
            'details': [],
            'score': 0
        }

        # Test emergency stop activation
        initial_safety = self.safety_functional

        self.emergency_stop_pub.publish(Bool(data=True))
        time.sleep(1)
        self.emergency_stop_pub.publish(Bool(data=False))
        time.sleep(1)

        if self.safety_functional or initial_safety:
            result['passed'] = True
            result['score'] = 100
            result['details'].append("✓ Emergency stop system functional")
        else:
            result['details'].append("⚠ Emergency stop system response unclear")

        return result

    def test_integration(self) -> Dict[str, Any]:
        """Test overall system integration."""
        self.get_logger().info("Testing system integration...")

        result = {
            'passed': False,
            'details': [],
            'score': 0
        }

        # Check overall system health
        if self.system_healthy:
            result['details'].append("✓ System reports healthy")
            result['score'] += 50

        if self.mission_responsive:
            result['details'].append("✓ Mission system integrated")
            result['score'] += 25

        if self.safety_functional:
            result['details'].append("✓ Safety system integrated")
            result['score'] += 25

        if result['score'] >= 75:
            result['passed'] = True

        return result

    def calculate_overall_score(self) -> float:
        """Calculate overall test score."""
        total_score = 0
        total_tests = 0

        for test_name, test_result in self.test_results.items():
            if test_name.startswith('test_'):
                continue
            if isinstance(test_result, dict) and 'score' in test_result:
                total_score += test_result['score']
                total_tests += 1

        return total_score / total_tests if total_tests > 0 else 0

    def print_results(self):
        """Print test results in a formatted way."""
        print("\n" + "="*60)
        print("🏆 COMPETITION READINESS TEST RESULTS")
        print("="*60)

        for test_name, result in self.test_results.items():
            if not isinstance(result, dict):
                continue

            status = "✅ PASS" if result.get('passed', False) else "❌ FAIL"
            score = result.get('score', 0)

            print(f"\n{test_name.upper().replace('_', ' ')}: {status} ({score:.1f}%)")

            for detail in result.get('details', []):
                print(f"  {detail}")

        overall_score = self.test_results.get('overall_score', 0)
        duration = self.test_results.get('test_duration', 0)

        print(f"\n{'='*60}")
        print(f"📊 OVERALL SCORE: {overall_score:.1f}%")
        print(f"⏱️  TEST DURATION: {duration:.1f} seconds")

        if overall_score >= 90:
            print("🎉 SYSTEM IS COMPETITION READY!")
        elif overall_score >= 75:
            print("⚠️  SYSTEM MOSTLY READY - MINOR ISSUES TO ADDRESS")
        else:
            print("🚨 SYSTEM NEEDS SIGNIFICANT WORK BEFORE COMPETITION")


def main():
    """Main test execution."""
    # Initialize ROS2
    rclpy.init()

    # Create test node
    test_node = CompetitionReadinessTest()

    try:
        # Give system time to start up
        time.sleep(10)

        # Run test suite
        results = test_node.run_test_suite()

        # Print results
        test_node.print_results()

        # Save results to file
        with open('competition_readiness_report.json', 'w') as f:
            json.dump(results, f, indent=2)

        print(f"\n📄 Detailed results saved to competition_readiness_report.json")

        # Return appropriate exit code
        overall_score = results.get('overall_score', 0)
        return 0 if overall_score >= 75 else 1

    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
        return 1
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
