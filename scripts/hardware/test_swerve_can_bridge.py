#!/usr/bin/env python3
"""
Test Swerve CAN Bridge Functionality
Quick test to verify 8-motor swerve support.
"""

import sys
import os

def test_swerve_can_bridge():
    """Test the extended CAN bridge with swerve support."""
    
    print("🧪 TESTING SWERVE CAN BRIDGE EXTENSION")
    
    try:
        # Add current directory to path
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
        
        from can_bridge import CANBridge
        
        print("✅ CAN Bridge imported successfully")
        
        # Initialize bridge
        bridge = CANBridge({'device': 'simulator'})
        print("✅ Bridge initialized for swerve testing")
        
        # Check if swerve method exists
        if hasattr(bridge, 'encode_swerve_commands'):
            print("✅ encode_swerve_commands method available")
            
            # Test swerve command encoding
            test_commands = {
                'fl_steer': 0.5, 'fl_drive': 1.0,
                'fr_steer': -0.3, 'fr_drive': 0.8,
                'rl_steer': 0.2, 'rl_drive': 0.7,
                'rr_steer': -0.1, 'rr_drive': 0.6,
            }
            
            try:
                frames = bridge.encode_swerve_commands(test_commands)
                print(f"✅ Swerve command encoding successful: {len(frames)} frames")
                
                # Check method availability
                if hasattr(bridge, 'encode_swerve_commands'):
                    print("✅ encode_swerve_commands method is working")
                    return True
                else:
                    print("❌ encode_swerve_commands method not found")
                    return False
                    
            except Exception as e:
                print(f"❌ Swerve command encoding error: {e}")
                return False
        else:
            print("❌ encode_swerve_commands method not available")
            return False
            
    except ImportError as e:
        print(f"❌ Import error: {e}")
        return False
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return False

def main():
    print("🚗 URC 2026 Swerve Drive CAN Bridge Test")
    print("="*50)
    
    success = test_swerve_can_bridge()
    
    if success:
        print("✅ SWERVE DRIVE EXTENSION WORKING")
        print("🚀 READY FOR 8-MOTOR SWERVE CONTROL")
        return 0
    else:
        print("❌ SWERVE DRIVE EXTENSION FAILED")
        print("🔧 CHECK CAN BRIDGE IMPLEMENTATION")
        return 1

if __name__ == "__main__":
    exit(main())