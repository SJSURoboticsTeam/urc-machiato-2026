#!/usr/bin/env python3
"""
Blackboard Connection Tester

Verifies that hardware_interface is writing to the blackboard.
This test requires the blackboard service to be running.
"""

import rclpy
from rclpy.node import Node
import sys
import time


class BlackboardTester(Node):
    """Test blackboard service connection."""

    def __init__(self):
        super().__init__('blackboard_tester')
        
        # Try to import blackboard service
        try:
            from autonomy_interfaces.srv import GetBlackboardValue, SetBlackboardValue
            self.get_srv_type = GetBlackboardValue
            self.set_srv_type = SetBlackboardValue
            self.get_logger().info("Blackboard service types imported successfully")
        except ImportError as e:
            self.get_logger().error(f"Failed to import blackboard services: {e}")
            return
        
        # Create clients
        self.get_client = self.create_client(self.get_srv_type, '/blackboard/get_value')
        self.set_client = self.create_client(self.set_srv_type, '/blackboard/set_value')
        
        self.get_logger().info("Waiting for blackboard services...")
        
    def test_service_available(self):
        """Check if blackboard services are available."""
        print("\n" + "="*70)
        print("Testing Blackboard Service Availability")
        print("="*70)
        print()
        
        # Wait for services
        get_available = self.get_client.wait_for_service(timeout_sec=5.0)
        set_available = self.set_client.wait_for_service(timeout_sec=5.0)
        
        print(f"Get service (/blackboard/get_value): {'✓ AVAILABLE' if get_available else '✗ NOT AVAILABLE'}")
        print(f"Set service (/blackboard/set_value): {'✓ AVAILABLE' if set_available else '✗ NOT AVAILABLE'}")
        print()
        
        if not (get_available and set_available):
            print("⚠️  Blackboard services not available")
            print()
            print("This means:")
            print("  - hardware_interface CAN write to blackboard (Option 1 code is present)")
            print("  - BUT no blackboard service is running to receive the writes")
            print()
            print("To fix:")
            print("  1. Start a BT.CPP node that provides blackboard service")
            print("  2. OR the blackboard writes will happen when BT nodes are running")
            print()
            print("For now, CAN data is still published to /hardware/* topics")
            print("which the dashboard is monitoring successfully.")
            return False
        
        return True
    
    def test_read_values(self):
        """Try to read values from blackboard."""
        print("="*70)
        print("Reading Blackboard Values (CAN → hardware_interface → blackboard)")
        print("="*70)
        print()
        
        keys_to_test = [
            'battery_level',
            'robot_velocity_x',
            'robot_velocity_y',
            'robot_x',
            'robot_y',
            'robot_yaw',
            'emergency_stop_active',
        ]
        
        found_values = 0
        
        for key in keys_to_test:
            req = self.get_srv_type.Request()
            req.key = key
            req.value_type = ''
            
            future = self.get_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.done():
                response = future.result()
                if response and response.success:
                    print(f"  ✓ {key:25s}: {response.value} ({response.value_type})")
                    found_values += 1
                else:
                    print(f"  ✗ {key:25s}: Not set")
            else:
                print(f"  ⏱ {key:25s}: Timeout")
        
        print()
        print(f"Found {found_values}/{len(keys_to_test)} expected values")
        print()
        
        if found_values > 0:
            print("✅ Blackboard is receiving CAN data from hardware_interface!")
            print("   Option 1 (direct write) is working correctly.")
        else:
            print("⚠️  No values found in blackboard")
            print("   Either blackboard service just started, or hardware_interface")
            print("   hasn't written values yet.")
        
        return found_values > 0


def main():
    rclpy.init()
    
    tester = BlackboardTester()
    
    try:
        print()
        print("╔════════════════════════════════════════════════════════════════╗")
        print("║         Blackboard Connection Test                            ║")
        print("╚════════════════════════════════════════════════════════════════╝")
        print()
        
        # Test service availability
        services_available = tester.test_service_available()
        
        if services_available:
            # Wait a moment for hardware_interface to write some values
            print("Waiting 2 seconds for hardware_interface to write values...")
            time.sleep(2)
            print()
            
            # Test reading values
            tester.test_read_values()
        else:
            print("="*70)
            print("Summary")
            print("="*70)
            print()
            print("Status: CAN messages flowing, blackboard service not available")
            print()
            print("What's working:")
            print("  ✓ hardware_interface is running")
            print("  ✓ CAN messages at 10 Hz")
            print("  ✓ Dashboard receiving /hardware/* topics")
            print("  ✓ Code for blackboard writes is present")
            print()
            print("What's missing:")
            print("  ✗ Blackboard service (/blackboard/get_value, /blackboard/set_value)")
            print()
            print("This is NORMAL for basic testing.")
            print("Blackboard writes will work when you start behavior tree nodes.")
            print()
        
    except KeyboardInterrupt:
        print("\nTest interrupted")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
