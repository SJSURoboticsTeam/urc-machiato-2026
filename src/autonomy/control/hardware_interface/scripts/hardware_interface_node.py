#!/usr/bin/env python3
"""
Hardware Interface Node - ROS2 to STM32 CAN Bus Bridge
Connects ROS2 autonomy commands to physical STM32 controllers.

Based on teleoperation system CAN serial communication protocol.
Integrates with vendor/control-systems STM32 firmware.
"""

import json
import math
import threading
import time
from typing import Any, Dict, Optional

import rclpy
import serial
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu, JointState, NavSatFix
from std_msgs.msg import Bool, Float32MultiArray, String


class HardwareInterfaceNode(Node):
    """
    Hardware Interface Node - ROS2 ↔ STM32 Controller Bridge

    Connects the ROS2 autonomy stack to the deployed STM32 control systems
    via CAN serial communication, following the teleoperation integration guide.
    """

    def __init__(self):
        super().__init__("hardware_interface")

        # Declare parameters
        self.declare_parameter("can_port", "/dev/ttyACM0")
        self.declare_parameter("can_baudrate", 115200)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("telemetry_rate_hz", 10.0)

        # Get parameters
        self.can_port = self.get_parameter("can_port").value
        self.can_baudrate = self.get_parameter("can_baudrate").value
        self.control_rate = self.get_parameter("control_rate_hz").value
        self.telemetry_rate = self.get_parameter("telemetry_rate_hz").value

        # CAN serial connection (based on teleoperation can_serial.py)
        self.can_serial = None
        self.can_connected = False

        # ROS2 QoS profiles for real-time control
        control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        # Publishers (data FROM hardware TO autonomy)
        self.joint_state_pub = self.create_publisher(
            JointState, "/hardware/joint_states", sensor_qos
        )

        self.chassis_velocity_pub = self.create_publisher(
            TwistStamped, "/hardware/chassis_velocity", sensor_qos
        )

        self.motor_temperatures_pub = self.create_publisher(
            Float32MultiArray, "/hardware/motor_temperatures", sensor_qos
        )

        self.battery_state_pub = self.create_publisher(
            BatteryState, "/hardware/battery_state", sensor_qos
        )

        self.imu_pub = self.create_publisher(Imu, "/hardware/imu", sensor_qos)

        self.gps_pub = self.create_publisher(NavSatFix, "/hardware/gps", sensor_qos)

        self.system_status_pub = self.create_publisher(
            String, "/hardware/system_status", sensor_qos
        )

        # Subscribers (commands FROM autonomy TO hardware)
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, control_qos
        )

        self.arm_cmd_sub = self.create_subscription(
            String, "/hardware/arm_command", self.arm_cmd_callback, control_qos
        )

        self.excavate_cmd_sub = self.create_subscription(
            String,
            "/hardware/excavate_command",
            self.excavate_cmd_callback,
            control_qos,
        )

        # Emergency stop subscriber
        self.emergency_stop_sub = self.create_subscription(
            Bool, "/emergency_stop", self.emergency_stop_callback, control_qos
        )

        # Control loop timer
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self.control_loop
        )

        # Telemetry timer
        self.telemetry_timer = self.create_timer(
            1.0 / self.telemetry_rate, self.telemetry_loop
        )

        # Connect to CAN serial on startup
        self.connect_can_serial()

        # Hardware state tracking
        self.last_cmd_vel = Twist()
        self.emergency_stop_active = False
        self.system_healthy = False

        self.get_logger().info("Hardware Interface Node initialized")

    def connect_can_serial(self):
        """Connect to CAN serial interface using teleoperation protocol."""
        try:
            # Import the CAN serial class from teleoperation system
            import sys

            sys.path.append(
                "/home/ubuntu/urc-machiato-2026/vendor/teleoperation/server"
            )
            from can_serial import CanSerial

            self.can_serial = CanSerial(self.can_port)
            self.can_connected = True
            self.system_healthy = True
            self.get_logger().info(f"Connected to CAN serial on {self.can_port}")

        except Exception as e:
            self.can_connected = False
            self.system_healthy = False
            self.get_logger().error(f"Failed to connect to CAN serial: {e}")

            # Fallback: create mock interface for testing
            self.can_serial = None
            self.get_logger().warn("Using mock CAN interface for testing")

    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands from autonomy stack."""
        if self.emergency_stop_active:
            self.get_logger().warn("Emergency stop active - ignoring velocity command")
            return

        self.last_cmd_vel = msg

        if self.can_connected and self.can_serial:
            self.send_velocity_command(msg)
        else:
            self.get_logger().debug("CAN not connected - velocity command buffered")

    def arm_cmd_callback(self, msg: String):
        """Handle arm control commands."""
        if self.can_connected and self.can_serial:
            self.send_arm_command(msg.data)
        else:
            self.get_logger().debug("CAN not connected - arm command buffered")

    def excavate_cmd_callback(self, msg: String):
        """Handle excavation commands."""
        if self.can_connected and self.can_serial:
            self.send_excavate_command(msg.data)
        else:
            self.get_logger().debug("CAN not connected - excavate command buffered")

    def emergency_stop_callback(self, msg: Bool):
        """Handle emergency stop commands."""
        self.emergency_stop_active = msg.data

        if self.can_connected and self.can_serial:
            if msg.data:
                self.send_emergency_stop()
                self.get_logger().error("EMERGENCY STOP ACTIVATED")
            else:
                self.send_emergency_resume()
                self.get_logger().info("Emergency stop deactivated")

    def send_velocity_command(self, twist: Twist):
        """Send velocity command to STM32 controllers."""
        try:
            # Convert ROS2 Twist to CAN message format
            # Based on drivetrain interface in vendor/control-systems/drive/
            linear_x = twist.linear.x
            angular_z = twist.angular.z

            # Calculate wheel velocities for differential drive
            # wheel_base = distance between wheels
            wheel_base = 0.5  # meters (adjust based on rover design)

            left_velocity = linear_x - (angular_z * wheel_base / 2.0)
            right_velocity = linear_x + (angular_z * wheel_base / 2.0)

            # Send CAN commands (format based on control-systems CAN protocol)
            left_cmd = f"SET_LEFT_VEL:{left_velocity:.3f}\r"
            right_cmd = f"SET_RIGHT_VEL:{right_velocity:.3f}\r"

            if self.can_serial:
                self.can_serial.write(left_cmd.encode())
                time.sleep(0.001)  # Small delay between commands
                self.can_serial.write(right_cmd.encode())

        except Exception as e:
            self.get_logger().error(f"Failed to send velocity command: {e}")

    def send_arm_command(self, command: str):
        """Send arm control commands to STM32."""
        try:
            # Parse and send arm commands
            # Format: ARM_CMD:joint1_pos,joint2_pos,joint3_pos,gripper_state
            arm_cmd = f"ARM_CMD:{command}\r"
            if self.can_serial:
                self.can_serial.write(arm_cmd.encode())
        except Exception as e:
            self.get_logger().error(f"Failed to send arm command: {e}")

    def send_excavate_command(self, command: str):
        """Send excavation commands to science payload."""
        try:
            # Send excavation commands to science STM32
            excavate_cmd = f"EXCAVATE:{command}\r"
            if self.can_serial:
                self.can_serial.write(excavate_cmd.encode())
        except Exception as e:
            self.get_logger().error(f"Failed to send excavate command: {e}")

    def send_emergency_stop(self):
        """Send emergency stop command."""
        try:
            stop_cmd = "EMERGENCY_STOP\r"
            if self.can_serial:
                self.can_serial.write(stop_cmd.encode())
        except Exception as e:
            self.get_logger().error(f"Failed to send emergency stop: {e}")

    def send_emergency_resume(self):
        """Send emergency resume command."""
        try:
            resume_cmd = "EMERGENCY_RESUME\r"
            if self.can_serial:
                self.can_serial.write(resume_cmd.encode())
        except Exception as e:
            self.get_logger().error(f"Failed to send emergency resume: {e}")

    def control_loop(self):
        """Main control loop - send commands at control rate."""
        # This is called by the timer, commands are sent in callbacks
        # Could add additional control logic here if needed
        pass

    def telemetry_loop(self):
        """Telemetry loop - read sensor data from hardware."""
        if not self.can_connected:
            # Publish mock data for testing when CAN not connected
            self.publish_mock_telemetry()
            return

        try:
            # Read sensor data from CAN serial
            # Based on teleoperation ROS2 publisher format
            self.read_joint_states()
            self.read_chassis_velocity()
            self.read_motor_temperatures()
            self.read_battery_state()
            self.read_imu_data()
            self.read_gps_data()

        except Exception as e:
            self.get_logger().error(f"Telemetry read error: {e}")
            self.system_healthy = False

    def read_joint_states(self):
        """Read joint states from CAN."""
        try:
            if self.can_serial:
                # Request joint state data
                self.can_serial.write("GET_JOINT_STATES\r".encode())

                # Read response (would need to implement parsing based on CAN protocol)
                # For now, publish mock data
                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.header.frame_id = "base_link"
                joint_state.name = [
                    "left_wheel",
                    "right_wheel",
                    "arm_joint1",
                    "arm_joint2",
                ]
                joint_state.position = [0.0, 0.0, 0.0, 0.0]  # Mock positions
                joint_state.velocity = [0.0, 0.0, 0.0, 0.0]  # Mock velocities

                self.joint_state_pub.publish(joint_state)

        except Exception as e:
            self.get_logger().error(f"Joint state read error: {e}")

    def read_chassis_velocity(self):
        """Read chassis velocity from CAN."""
        try:
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            twist_stamped.header.frame_id = "odom"

            # Mock velocity data (would read from CAN)
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = 0.0

            self.chassis_velocity_pub.publish(twist_stamped)

        except Exception as e:
            self.get_logger().error(f"Chassis velocity read error: {e}")

    def read_motor_temperatures(self):
        """Read motor temperatures."""
        try:
            temp_array = Float32MultiArray()
            temp_array.data = [25.0, 25.0, 25.0, 25.0]  # Mock temperatures in Celsius

            self.motor_temperatures_pub.publish(temp_array)

        except Exception as e:
            self.get_logger().error(f"Motor temperature read error: {e}")

    def read_battery_state(self):
        """Read battery state."""
        try:
            battery_state = BatteryState()
            battery_state.header.stamp = self.get_clock().now().to_msg()
            battery_state.voltage = 24.0  # Mock voltage
            battery_state.current = -5.0  # Negative = discharging
            battery_state.percentage = 85.0  # 85% charged

            self.battery_state_pub.publish(battery_state)

        except Exception as e:
            self.get_logger().error(f"Battery state read error: {e}")

    def read_imu_data(self):
        """Read IMU data."""
        try:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            # Mock IMU data
            imu_msg.linear_acceleration.x = 0.0
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 9.81

            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = 0.0

            self.imu_pub.publish(imu_msg)

        except Exception as e:
            self.get_logger().error(f"IMU read error: {e}")

    def read_gps_data(self):
        """Read GPS data."""
        try:
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.latitude = 33.0  # Mock coordinates
            gps_msg.longitude = -117.0
            gps_msg.altitude = 100.0

            self.gps_pub.publish(gps_msg)

        except Exception as e:
            self.get_logger().error(f"GPS read error: {e}")

    def publish_mock_telemetry(self):
        """Publish mock telemetry data for testing without hardware."""
        self.read_joint_states()
        self.read_chassis_velocity()
        self.read_motor_temperatures()
        self.read_battery_state()
        self.read_imu_data()
        self.read_gps_data()

        # Publish system status
        status_msg = String()
        status_msg.data = json.dumps(
            {
                "hardware_connected": self.can_connected,
                "system_healthy": self.system_healthy,
                "emergency_stop": self.emergency_stop_active,
                "timestamp": self.get_clock().now().nanoseconds / 1e9,
            }
        )
        self.system_status_pub.publish(status_msg)

    def destroy_node(self):
        """Clean shutdown."""
        if self.can_serial:
            try:
                # Send shutdown command
                self.can_serial.write("SHUTDOWN\r".encode())
                time.sleep(0.1)
                self.can_serial.close()
            except:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = HardwareInterfaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
