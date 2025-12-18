#!/usr/bin/env python3
"""
Simplified ROS2 Messaging Consistency Test
Tests basic publisher/subscriber patterns with our simplified mission system
"""

import time

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleMessagingTester(Node):
    """Test basic ROS2 messaging with simplified mission system."""

    def __init__(self):
        super().__init__("simple_messaging_tester")
        self.logger = self.get_logger()

        # Test results
        self.received_messages = []
        self.messages_sent = 0
        self.messages_received = 0

        # Subscribe to mission commands
        self.cmd_sub = self.create_subscription(
            String, "/mission/commands", self.command_callback, 10
        )

        # Publisher for testing
        self.cmd_pub = self.create_publisher(String, "/mission/commands", 10)

        self.logger.info("Simple messaging tester initialized")

    def command_callback(self, msg):
        """Handle received mission commands."""
        self.received_messages.append(msg.data)
        self.messages_received += 1
        self.logger.info(f"Received command: {msg.data}")

    def publish_command(self, command):
        """Publish a mission command."""
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self.messages_sent += 1
        self.logger.info(f"Published command: {command}")


@pytest.mark.integration
class TestSimplifiedMessaging:
    """Test simplified ROS2 messaging consistency."""

    def test_basic_pub_sub(self):
        """Test basic publish/subscribe functionality."""
        rclpy.init()

        try:
            # Create publisher and subscriber
            publisher = SimpleMessagingTester()
            subscriber = SimpleMessagingTester()

            # Create executor
            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(publisher)
            executor.add_node(subscriber)

            # Test message publishing
            test_commands = ["start", "pause", "resume", "stop"]

            for cmd in test_commands:
                publisher.publish_command(cmd)
                time.sleep(0.1)  # Allow message to propagate

            # Spin for a bit to process messages
            start_time = time.time()
            while time.time() - start_time < 2.0 and rclpy.ok():
                executor.spin_once(timeout_sec=0.1)

            # Verify messages were received
            assert subscriber.messages_received == len(
                test_commands
            ), f"Expected {len(test_commands)} messages, got {subscriber.messages_received}"

            assert (
                subscriber.received_messages == test_commands
            ), f"Message content mismatch: expected {test_commands}, got {subscriber.received_messages}"

            print("✅ Basic pub/sub test passed!")

        finally:
            rclpy.shutdown()

    def test_message_types(self):
        """Test that String messages work correctly."""
        rclpy.init()

        try:
            node = SimpleMessagingTester()

            # Test message creation
            msg = String()
            msg.data = "test_command"

            assert hasattr(msg, "data"), "String message should have data field"
            assert isinstance(msg.data, str), "data field should be string"
            assert msg.data == "test_command", "data field should contain correct value"

            print("✅ Message type validation passed!")

        finally:
            rclpy.shutdown()
