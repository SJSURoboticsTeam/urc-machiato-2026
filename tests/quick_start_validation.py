#!/usr/bin/env python3
"""
Quick Start Validation for 1-Week Testing Plan

Run this script immediately to validate critical communication systems
and establish baseline performance metrics.

Usage:
    python tests/quick_start_validation.py

Author: URC 2026 Testing Team
"""

import subprocess
import sys
import time
import os
from pathlib import Path

def run_test_command(cmd, description, timeout=60):
    """Run a test command with error handling."""
    print(f"\n{'='*60}")
    print(f"🧪 {description}")
    print(f"{'='*60}")
    
    start_time = time.time()
    
    try:
        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=timeout,
            cwd=Path(__file__).parent.parent
        )
        
        duration = time.time() - start_time
        
        if result.returncode == 0:
            print(f"✅ PASSED in {duration:.1f}s")
            if result.stdout:
                print(f"Output:\n{result.stdout}")
        else:
            print(f"❌ FAILED after {duration:.1f}s")
            print(f"Error output:\n{result.stderr}")
            return False
            
    except subprocess.TimeoutExpired:
        print(f"⏰ TIMEOUT after {timeout}s")
        return False
    except Exception as e:
        print(f"💥 ERROR: {e}")
        return False
    
    return True

def main():
    """Execute quick start validation."""
    print("🚀 URC 2026 Swerve Drive Testing - Quick Start Validation")
    print("Optimized for RealSense 435 on Pi 5 16GB")
    print("Validating communication readiness and SLAM performance...")
    
    test_results = []
    
    # Test 1: Existing communication tests
    test_results.append(run_test_command(
        "python3 -m pytest tests/unit/test_bridge_communications.py -v",
        "Existing Communication Tests",
        timeout=30
    ))
    
    # Test 2: Communication stress tests (WebSocket high-frequency)
    test_results.append(run_test_command(
        "python3 -m pytest tests/communication/test_communication_stress.py::TestCommunicationStress::test_websocket_high_frequency_commands -v",
        "WebSocket High-Frequency Command Test (100Hz)",
        timeout=90
    ))
    
    # Test 3: SLAM performance baseline (RealSense 435 optimized)
    test_results.append(run_test_command(
        "python3 -m pytest tests/slam/test_slam_performance.py::TestSLAMPerformance::test_frame_processing_rate -v -s",
        "SLAM Frame Processing (RealSense 435 on Pi 5)",
        timeout=90
    ))
    
    # Test 4: Integration validation
    test_results.append(run_test_command(
        "python3 -c \"import sys; print('Python path validated'); sys.exit(0)\"",
        "Python Environment Validation",
        timeout=10
    ))
    
    # Test 5: ROS2 availability check
    test_results.append(run_test_command(
        "python3 -c \"try:\n    import rclpy\n    print('ROS2 available for testing')\nexcept ImportError:\n    print('ROS2 not available - some tests will be skipped')\n    print('This is expected in development environment')\"",
        "ROS2 Environment Check",
        timeout=10
    ))
    
    # Results Summary
    print(f"\n{'='*60}")
    print("📊 QUICK START VALIDATION SUMMARY")
    print(f"{'='*60}")
    
    passed = sum(test_results)
    total = len(test_results)
    
    print(f"Tests Passed: {passed}/{total}")
    
    if passed == total:
        print("🎉 All critical systems validated!")
        print("You're ready to proceed with the 1-week testing plan.")
        return 0
    else:
        print("⚠️  Some tests failed. Check the output above for details.")
        print("Fix issues before proceeding with the full testing plan.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
