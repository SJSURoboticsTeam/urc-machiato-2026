#!/usr/bin/env python3
"""
Hardware-in-the-Loop (HIL) Testing Demo

Demonstrates mixing real hardware with simulated components
for flexible testing configurations.

Author: URC 2026 Examples Team
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from simulation.hil.hil_manager import HILManager, ComponentMode
from simulation.hil.device_discovery import DeviceDiscovery


def device_discovery_demo():
    """Demonstrate hardware device discovery."""
    print("\n=== Device Discovery Demo ===")
    
    discovery = DeviceDiscovery()
    
    print("\n🔍 Scanning for hardware devices...")
    devices = discovery.scan()
    
    if devices:
        print(f"\n✅ Found {len(devices)} device(s):")
        for device in devices:
            print(f"  • {device['device_type']}: {device['port']}")
            print(f"    Description: {device['description']}")
    else:
        print("  No hardware devices found (running in pure simulation mode)")
    
    print("\n✅ Discovery complete!\n")


def hil_modes_demo():
    """Demonstrate different HIL modes."""
    print("\n=== HIL Modes Demo ===")
    
    manager = HILManager()
    
    # Try real mode (falls back to simulation if no hardware)
    print("\n🔄 Mode: AUTO (try real, fallback to simulation)")
    
    firmware_component = manager.initialize_component(
        'firmware',
        mode=ComponentMode.AUTO
    )
    
    if firmware_component:
        mode = "REAL" if hasattr(firmware_component, 'serial_port') else "SIMULATED"
        print(f"  Firmware initialized in {mode} mode")
    
    # Force simulation mode
    print("\n💻 Mode: SIMULATED (always use simulation)")
    
    slcan_component = manager.initialize_component(
        'slcan',
        mode=ComponentMode.SIMULATED
    )
    
    if slcan_component:
        print("  SLCAN initialized in SIMULATED mode")
    
    print("\n✅ Modes demo complete!\n")


def mixed_testing_demo():
    """Demonstrate mixed real/simulated testing."""
    print("\n=== Mixed Real/Simulated Demo ===")
    
    manager = HILManager()
    
    print("\n🔧 Setting up mixed environment...")
    
    # Initialize components (will use simulation if no hardware)
    components = {
        'firmware': manager.initialize_component('firmware', ComponentMode.AUTO),
        'slcan': manager.initialize_component('slcan', ComponentMode.SIMULATED),
        'websocket': manager.initialize_component('websocket', ComponentMode.SIMULATED)
    }
    
    print("\n📊 Component Status:")
    for name, component in components.items():
        if component:
            mode = "REAL" if hasattr(component, 'serial_port') or hasattr(component, 'device') else "SIMULATED"
            print(f"  {name}: {mode}")
    
    # Run test with mixed components
    print("\n🧪 Running test with mixed components...")
    
    if components['firmware']:
        # Start firmware (simulation or real)
        if hasattr(components['firmware'], 'start'):
            components['firmware'].start()
            print("  ✅ Firmware started")
        
        # Send command
        if hasattr(components['firmware'], 'set_velocity_command'):
            components['firmware'].set_velocity_command(0, 3.0)
            print("  ✅ Command sent")
            
            import time
            time.sleep(0.5)
            
            # Check status
            status = components['firmware'].get_motor_status(0)
            if status:
                print(f"  ✅ Motor velocity: {status['velocity_actual']:.2f} rad/s")
        
        # Stop firmware
        if hasattr(components['firmware'], 'stop'):
            components['firmware'].stop()
    
    print("\n✅ Mixed testing complete!\n")


def comparison_demo():
    """Demonstrate real vs. simulated comparison."""
    print("\n=== Real vs. Simulated Comparison Demo ===")
    
    manager = HILManager()
    
    print("\n📊 Initializing for comparison...")
    
    # Initialize both if possible
    real_firmware = manager.initialize_component('firmware', ComponentMode.REAL)
    sim_firmware = manager.initialize_component('firmware', ComponentMode.SIMULATED)
    
    if real_firmware and sim_firmware:
        print("  ✅ Both real and simulated firmware available")
        
        # Run same test on both
        print("\n🧪 Running identical test on both...")
        
        # Start both
        if hasattr(real_firmware, 'start'):
            real_firmware.start()
        sim_firmware.start()
        
        # Send same command
        velocity = 5.0
        if hasattr(real_firmware, 'set_velocity_command'):
            real_firmware.set_velocity_command(0, velocity)
        sim_firmware.set_velocity_command(0, velocity)
        
        import time
        time.sleep(1.0)
        
        # Compare results
        if hasattr(real_firmware, 'get_motor_status'):
            real_status = real_firmware.get_motor_status(0)
            sim_status = sim_firmware.get_motor_status(0)
            
            print("\n📈 Comparison Results:")
            print(f"  Real: {real_status['velocity_actual']:.3f} rad/s")
            print(f"  Sim:  {sim_status['velocity_actual']:.3f} rad/s")
            print(f"  Diff: {abs(real_status['velocity_actual'] - sim_status['velocity_actual']):.3f} rad/s")
        
        # Stop both
        if hasattr(real_firmware, 'stop'):
            real_firmware.stop()
        sim_firmware.stop()
        
        # Record comparison
        comparison = manager.record_comparison('motor_velocity', {
            'real': real_status['velocity_actual'] if 'real_status' in locals() else 0,
            'simulated': sim_status['velocity_actual'],
            'test': 'velocity_response'
        })
        
        print(f"\n💾 Comparison recorded")
        
    elif sim_firmware:
        print("  ℹ️  Only simulation available (no hardware detected)")
        print("  Running in pure simulation mode")
    else:
        print("  ⚠️  Could not initialize components")
    
    print("\n✅ Comparison demo complete!\n")


def performance_profiling_demo():
    """Demonstrate performance profiling."""
    print("\n=== Performance Profiling Demo ===")
    
    from simulation.firmware.stm32_firmware_simulator import create_firmware_simulator
    import time
    
    sim = create_firmware_simulator('default')
    sim.start()
    
    print("\n⏱️  Measuring control loop performance...")
    
    initial_cycles = sim.stats['control_cycles']
    start_time = time.time()
    
    time.sleep(2.0)
    
    elapsed = time.time() - start_time
    cycles = sim.stats['control_cycles'] - initial_cycles
    
    frequency = cycles / elapsed
    
    print(f"  Control cycles: {cycles}")
    print(f"  Duration: {elapsed:.2f}s")
    print(f"  Frequency: {frequency:.1f} Hz")
    print(f"  Target: 100 Hz")
    
    if 90 < frequency < 110:
        print(f"  ✅ Within tolerance")
    else:
        print(f"  ⚠️  Outside tolerance")
    
    sim.stop()
    print("\n✅ Performance profiling complete!\n")


def main():
    """Run all HIL demos."""
    print("=" * 60)
    print("Hardware-in-the-Loop (HIL) Testing Demo")
    print("=" * 60)
    print("\nNote: These demos will use simulation if no hardware is detected.")
    print("Connect hardware devices to see HIL capabilities in action.\n")
    
    try:
        device_discovery_demo()
        hil_modes_demo()
        mixed_testing_demo()
        comparison_demo()
        performance_profiling_demo()
        
        print("=" * 60)
        print("✅ All HIL demos completed!")
        print("=" * 60)
        print("\nNext steps:")
        print("  • Connect real CAN adapter to test with hardware")
        print("  • Review comparison data in output/hil_comparisons/")
        print("  • Calibrate simulation parameters based on real data")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
