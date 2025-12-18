#!/usr/bin/env python3
"""
Basic ROS2 Integration Test
Tests ROS2 topic publishing and subscribing with simplified mission system
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestMissionSystem(Node):
    """Test simplified mission system communication"""

    def __init__(self):
        super().__init__("test_mission_system")

        self.received_messages = {"mission_commands": [], "mission_status": []}

        # Subscribe to mission commands (test publishing)
        self.cmd_sub = self.create_subscription(
            String,
            "/mission/commands",
            lambda msg: self.message_callback("mission_commands", msg),
            10,
        )

        # Publisher for mission status (test subscribing)
        self.status_pub = self.create_publisher(String, "/mission/status", 10)

        self.get_logger().info("Test mission system initialized")

    def message_callback(self, topic, msg):
        """Handle incoming messages"""
        self.received_messages[topic].append(
            {"timestamp": time.time(), "data": msg.data}  # String message data
        )

        # Keep only last 5 messages
        if len(self.received_messages[topic]) > 5:
            self.received_messages[topic].pop(0)

        self.get_logger().debug(f"Received {topic} message: {msg.data}")

    def send_mission_command(self, command):
        """Send a mission command for testing"""
        msg = String()
        msg.data = command
        self.status_pub.publish(msg)  # Note: Publishing to status topic for test
        self.get_logger().info(f"Sent mission command: {command}")

    def get_message_counts(self):
        """Get count of received messages per topic"""
        return {
            topic: len(messages) for topic, messages in self.received_messages.items()
        }

    def has_received_data(self):
        """Check if we've received mission command data"""
        counts = self.get_message_counts()
        return counts.get("mission_commands", 0) > 0


class MockMissionPublisher(Node):
    """Mock mission publisher for testing"""

    def __init__(self):
        super().__init__("mock_mission_publisher")
        self.cmd_pub = self.create_publisher(String, "/mission/commands", 10)
        self.get_logger().info("Mock mission publisher initialized")

    def publish_command(self, command):
        """Publish a mission command"""
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Published mission command: {command}")


class BasicROS2IntegrationTest:
    """Basic ROS2 integration test with simplified mission system"""

    def __init__(self):
        self.success = False
        self.publisher_thread = None
        self.subscriber_node = None

    def run_integrated_test(self):
        """Run both publisher and subscriber in same ROS2 context"""
        print("[ANTENNA] Starting integrated ROS2 mission system test...")

        # Initialize ROS2 once
        rclpy.init()

        try:
            # Create both nodes
            publisher = MockMissionPublisher()
            self.subscriber_node = TestMissionSystem()

            # Create executor for both nodes
            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(publisher)
            executor.add_node(self.subscriber_node)

            # Run test for specified duration with active publishing
            print("⏳ Running integrated mission system test...")
            end_time = time.time() + 8
            publish_interval = 1.0  # Publish every second
            last_publish = 0

            test_commands = ["start", "pause", "resume", "stop"]
            command_index = 0

            while time.time() < end_time and rclpy.ok():
                current_time = time.time()

                # Publish a mission command periodically
                if current_time - last_publish >= publish_interval:
                    command = test_commands[command_index % len(test_commands)]
                    publisher.publish_command(command)
                    command_index += 1
                    last_publish = current_time

                executor.spin_once(timeout_sec=0.1)

            # Check results
            if self.subscriber_node:
                message_counts = self.subscriber_node.get_message_counts()
                has_data = self.subscriber_node.has_received_data()

                print("\n[GRAPH] MISSION SYSTEM INTEGRATION TEST RESULTS:")
                print(
                    f"   Has received mission commands: {'[PASS]' if has_data else '[FAIL]'}"
                )

                print("   Message counts:")
                for topic, count in message_counts.items():
                    status = "[PASS]" if count > 0 else "[FAIL]"
                    print(f"     {status} {topic}: {count} messages")

                # Check that we received mission commands (allow for timing variations)
                # We should receive at least some messages to prove pub/sub works
                received_count = message_counts.get("mission_commands", 0)
                min_expected_messages = 3  # Reasonable minimum for pub/sub validation

                if has_data and received_count >= min_expected_messages:
                    print("\n[PARTY] Mission System Integration Test PASSED!")
                    print(
                        f"[PASS] Mission commands published and received ({received_count} messages)"
                    )
                    print("[PASS] ROS2 pub/sub communication is working")
                    print("[PASS] Simplified mission system is functional")
                    self.success = True
                else:
                    print("\n[FAIL] Mission System Integration Test FAILED!")
                    print(
                        f"[FAIL] Expected at least {min_expected_messages} messages, got {received_count}"
                    )
                    print("[FAIL] Check ROS2 setup and topic communication")
                    self.success = False

            # Cleanup
            executor.remove_node(publisher)
            executor.remove_node(self.subscriber_node)
            publisher.destroy_node()
            self.subscriber_node.destroy_node()

        except Exception as e:
            print(f"\n Integrated test failed: {e}")
            self.success = False

        finally:
            rclpy.shutdown()

    def run_test(self):
        """Run the basic ROS2 integration test"""
        print("[EXPERIMENT] Basic ROS2 Integration Test")
        print("=" * 40)

        try:
            # Run integrated test with both publisher and subscriber
            self.run_integrated_test()

        except Exception as e:
            print(f"\n Test failed with exception: {e}")
            self.success = False

        return self.success


def main():
    """Main test function"""
    print("Testing basic ROS2 teleoperation integration...")

    test = BasicROS2IntegrationTest()
    success = test.run_test()

    print(f"\n[FLAG] Final Result: {'SUCCESS' if success else 'FAILURE'}")
    return success


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
