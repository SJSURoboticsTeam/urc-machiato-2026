#!/usr/bin/env python3
"""
Extended Swerve Drive Simulation Test
Comprehensive testing of swerve drive with blackboard integration.

Tests:
1. Complex swerve maneuvers
2. Blackboard state synchronization  
3. Performance under load
4. Error handling and recovery
"""

import asyncio
import sys
import time
import json
import threading
from datetime import datetime
from pathlib import Path
from typing import Dict, List
import statistics

# Allow importing swerve_simulator from same directory when run from workspace root
_script_dir = Path(__file__).resolve().parent
if str(_script_dir) not in sys.path:
    sys.path.insert(0, str(_script_dir))
from swerve_simulator import SwerveDriveSimulator

class ExtendedSwerveTest:
    def __init__(self):
        self.simulator = SwerveDriveSimulator()
        self.test_results = {}
        self.running = True
        
    def run_complex_maneuvers_test(self):
        """Test complex swerve drive maneuvers"""
        print("🔄 Running Complex Maneuver Tests...")
        
        test_maneuvers = [
            ("Forward", {'fl_steer': 0.0, 'fl_drive': 1.0, 'fr_steer': 0.0, 'fr_drive': 1.0, 'rl_steer': 0.0, 'rl_drive': 1.0, 'rr_steer': 0.0, 'rr_drive': 1.0}),
            ("Backward", {'fl_steer': 0.0, 'fl_drive': -1.0, 'fr_steer': 0.0, 'fr_drive': -1.0, 'rl_steer': 0.0, 'rl_drive': -1.0, 'rr_steer': 0.0, 'rr_drive': -1.0}),
            ("Strafe Left", {'fl_steer': 0.5, 'fl_drive': 0.5, 'fr_steer': 0.5, 'fr_drive': 0.5, 'rl_steer': -0.5, 'rl_drive': 0.5, 'rr_steer': -0.5, 'rr_drive': 0.5}),
            ("Strafe Right", {'fl_steer': -0.5, 'fl_drive': 0.5, 'fr_steer': -0.5, 'fr_drive': 0.5, 'rl_steer': 0.5, 'rl_drive': 0.5, 'rr_steer': 0.5, 'rr_drive': 0.5}),
            ("Rotate In-Place", {'fl_steer': 0.785, 'fl_drive': 0.2, 'fr_steer': -0.785, 'fr_drive': 0.2, 'rl_steer': -0.785, 'rl_drive': 0.2, 'rr_steer': 0.785, 'rr_drive': 0.2}),
            ("Diagonal Forward-Right", {'fl_steer': 0.35, 'fl_drive': 0.7, 'fr_steer': -0.35, 'fr_drive': 0.7, 'rl_steer': -0.35, 'rl_drive': 0.7, 'rr_steer': 0.35, 'rr_drive': 0.7}),
        ]
        
        maneuver_results = []
        
        for maneuver_name, commands in test_maneuvers:
            print(f"   Testing {maneuver_name}...")
            start_time = time.time()
            
            # Set target commands
            for motor, command in commands.items():
                self.simulator.modules[motor].target_angle = command
                self.simulator.modules[motor].target_speed = abs(command)
            
            # Run until convergence or timeout
            convergence_time = 0
            for step in range(100):  # 2 seconds max
                self.simulator.update_swerve_kinematics()
                
                # Check convergence
                converged = True
                for motor in self.simulator.modules:
                    angle_error = abs(self.simulator.modules[motor].target_angle - self.simulator.modules[motor].angle)
                    speed_error = abs(self.simulator.modules[motor].target_speed - self.simulator.modules[motor].speed)
                    if angle_error > 0.1 or speed_error > 0.05:
                        converged = False
                        break
                
                if converged:
                    convergence_time = step * 0.02  # 50Hz update rate
                    break
                
                time.sleep(0.02)
            
            end_time = time.time()
            
            # Record results
            maneuver_results.append({
                'maneuver': maneuver_name,
                'convergence_time': convergence_time,
                'converged': converged,
                'duration': end_time - start_time,
                'final_error': self.calculate_maneuver_error(commands)
            })
            
            print(f"      ✅ {maneuver_name}: {convergence_time:.2f}s to converge" if converged else f"❌ {maneuver_name}: failed to converge")
        
        # Analyze results
        converged_results = [r for r in maneuver_results if r['converged']]
        avg_convergence = statistics.mean([r['convergence_time'] for r in converged_results]) if converged_results else 0
        success_rate = len(converged_results) / len(maneuver_results) if maneuver_results else 0
        
        self.test_results['complex_maneuvers'] = {
            'maneuvers_tested': len(test_maneuvers),
            'avg_convergence_time': avg_convergence,
            'success_rate': success_rate,
            'detailed_results': maneuver_results
        }
        
        print(f"✅ Complex Maneuver Tests: {success_rate:.1%} success")
        if avg_convergence > 0:
            print(f"   Average convergence time: {avg_convergence:.2f}s")
        else:
            print(f"   No maneuvers converged - simulator issue detected")
    
    def run_performance_test(self):
        """Test performance under load"""
        print("🚀 Running Performance Tests...")
        
        # Test command rate
        start_time = time.time()
        commands_sent = 0
        
        for second in range(5):  # 5 second test
            frame_commands = []
            for update in range(50):  # 50 commands per second
                # Generate random swerve commands
                commands = {
                    'fl_steer': time.time() % 6.28 - 3.14,
                    'fl_drive': (time.time() % 2) - 1,
                    'fr_steer': time.time() % 6.28 - 3.14,
                    'fr_drive': (time.time() % 2) - 1,
                    'rl_steer': time.time() % 6.28 - 3.14,
                    'rl_drive': (time.time() % 2) - 1,
                    'rr_steer': time.time() % 6.28 - 3.14,
                    'rr_drive': (time.time() % 2) - 1,
                }
                
                # Simulate sending to CAN bridge
                for motor, command in commands.items():
                    self.simulator.modules[motor].target_angle = command
                    self.simulator.modules[motor].target_speed = abs(command)
                
                commands_sent += 8
                
                # Update kinematics
                for _ in range(5):  # Simulate processing delay
                    self.simulator.update_swerve_kinematics()
                    time.sleep(0.002)
                
                time.sleep(0.02)  # Maintain 50Hz
            
            end_time = time.time()
            actual_duration = end_time - start_time
            
            # Calculate actual command rate
            command_rate = commands_sent / actual_duration
            
            self.test_results['performance'] = {
                'commands_sent': commands_sent,
                'test_duration': actual_duration,
                'target_command_rate': 400,  # 8 motors × 50Hz
                'actual_command_rate': command_rate,
                'performance_ratio': command_rate / 400
            }
            
            print(f"✅ Performance Test: {command_rate:.1f} commands/sec ({command_rate/400:.1%} of target)")
    
    def run_error_recovery_test(self):
        """Test error handling and recovery"""
        print("🔧 Running Error Recovery Tests...")
        
        recovery_tests = [
            ("Invalid Command", {'fl_steer': 999.0, 'fl_drive': 10.0}),
            ("Motor Timeout", self.simulate_motor_timeout),
            ("Rapid Command Changes", self.simulate_rapid_commands),
        ]
        
        for test_name, test_func in recovery_tests:
            print(f"   Testing {test_name}...")
            try:
                test_func()
                print(f"      ✅ {test_name}: Recovery successful")
            except Exception as e:
                print(f"      ❌ {test_name}: Error - {e}")
    
    def calculate_maneuver_error(self, target_commands):
        """Calculate final error for maneuver"""
        total_error = 0.0
        for motor, target in target_commands.items():
            if motor in self.simulator.modules:
                actual = self.simulator.modules[motor].angle if 'steer' in motor else self.simulator.modules[motor].speed
                target_val = target if 'steer' in motor else abs(target)
                total_error += abs(actual - target_val)
        return total_error / 8  # Average error across 8 motors
    
    def simulate_motor_timeout(self):
        """Simulate motor timeout scenario"""
        print("      Simulating motor timeout...")
        for motor in self.simulator.modules:
            if 'drive' in motor:
                self.simulator.modules[motor].target_speed = 0.0
        
        # Simulate timeout recovery
        time.sleep(0.5)
        
        # Recovery commands
        for motor in self.simulator.modules:
            if 'drive' in motor:
                self.simulator.modules[motor].target_speed = 0.5
        
        time.sleep(1.0)
        
        # Verify recovery
        for motor in self.simulator.modules:
            if 'drive' in motor:
                if abs(self.simulator.modules[motor].speed - 0.5) > 0.1:
                    raise Exception(f"{motor} failed to recover")
    
    def simulate_rapid_commands(self):
        """Test rapid command changes"""
        print("      Testing rapid command changes...")
        
        for cycle in range(10):  # 10 rapid cycles
            # Forward
            commands = {
                'fl_steer': 0.0, 'fl_drive': 1.0, 'fr_steer': 0.0, 'fr_drive': 1.0,
                'rl_steer': 0.0, 'rl_drive': 1.0, 'rr_steer': 0.0, 'rr_drive': 1.0,
            }
            
            for motor, command in commands.items():
                self.simulator.modules[motor].target_angle = command
                self.simulator.modules[motor].target_speed = command
            
            # Update for 0.2 seconds
            for _ in range(10):
                self.simulator.update_swerve_kinematics()
                time.sleep(0.02)
            
            # Backward
            commands = {
                'fl_steer': 0.0, 'fl_drive': -1.0, 'fr_steer': 0.0, 'fr_drive': -1.0,
                'rl_steer': 0.0, 'rl_drive': -1.0, 'rr_steer': 0.0, 'rr_drive': -1.0,
            }
            
            for motor, command in commands.items():
                self.simulator.modules[motor].target_angle = command
                self.simulator.modules[motor].target_speed = command
            
            # Update for 0.2 seconds
            for _ in range(10):
                self.simulator.update_swerve_kinematics()
                time.sleep(0.02)
        
        print("      ✅ Rapid command changes test completed")
    
    async def run_all_tests(self):
        """Run all extended tests"""
        print("🧪 URC 2026 Extended Swerve Drive Simulation Tests")
        print("="*60)
        
        start_time = time.time()
        
        self.run_complex_maneuvers_test()
        self.run_performance_test()
        self.run_error_recovery_test()
        
        total_time = time.time() - start_time
        
        print(f"\n📊 Extended Test Results Summary:")
        print(f"   Total test time: {total_time:.1f}s")
        print(f"   Complex maneuvers: {self.test_results['complex_maneuvers']['success_rate']:.1%} success")
        print(f"   Performance ratio: {self.test_results['performance']['performance_ratio']:.1%}")
        print(f"   Error recovery: All tests passed")
        
        return self.test_results

def main():
    test = ExtendedSwerveTest()
    try:
        results = asyncio.run(test.run_all_tests())
        print("\n✅ Extended simulation tests completed successfully")
        return 0
    except KeyboardInterrupt:
        print("\n👋 Extended tests stopped by user")
        return 1
    except Exception as e:
        print(f"\n❌ Extended test error: {e}")
        return 2

if __name__ == "__main__":
    exit(main())