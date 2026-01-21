#!/usr/bin/env python3
"""
Simulation Latency Performance Tests

Measures latency at various points in the communication stack.

Author: URC 2026 Performance Testing Team
"""

import pytest
import asyncio
import time
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from simulation.integration.full_stack_simulator import create_full_stack_simulator


# Latency targets (milliseconds)
LATENCY_TARGETS = {
    'perfect_e2e': 50.0,      # End-to-end in perfect mode
    'default_e2e': 150.0,     # End-to-end in default mode
    'network_delay': 50.0,    # Network simulation delay
    'slcan_encode': 1.0,      # SLCAN encoding
    'firmware_response': 20.0, # Firmware response time
}


class TestEndToEndLatency:
    """Test end-to-end latency."""
    
    @pytest.mark.asyncio
    async def test_perfect_mode_latency(self):
        """Measure latency in perfect mode (no delays)."""
        sim = create_full_stack_simulator('perfect')
        client_id = await sim.websocket_sim.connect()
        
        latencies = []
        iterations = 20
        
        for i in range(iterations):
            start = time.time()
            
            await sim.websocket_sim.receive('driveCommands', {
                'linear': 0.5,
                'angular': 0.0
            }, client_id)
            
            # Minimal delay for processing
            await asyncio.sleep(0.001)
            
            latency_ms = (time.time() - start) * 1000
            latencies.append(latency_ms)
        
        sim.shutdown()
        
        avg_latency = statistics.mean(latencies)
        p50_latency = statistics.median(latencies)
        p95_latency = statistics.quantiles(latencies, n=20)[18]  # 95th percentile
        
        print(f"\n📊 Perfect Mode Latency:")
        print(f"   Average: {avg_latency:.2f}ms")
        print(f"   P50: {p50_latency:.2f}ms")
        print(f"   P95: {p95_latency:.2f}ms")
        print(f"   Target: <{LATENCY_TARGETS['perfect_e2e']}ms")
        
        assert avg_latency < LATENCY_TARGETS['perfect_e2e'], \
            f"Average latency too high: {avg_latency:.2f}ms > {LATENCY_TARGETS['perfect_e2e']}ms"
    
    @pytest.mark.asyncio
    async def test_default_mode_latency(self):
        """Measure latency with realistic network conditions."""
        sim = create_full_stack_simulator('default')
        client_id = await sim.websocket_sim.connect()
        
        latencies = []
        iterations = 10
        
        for i in range(iterations):
            start = time.time()
            
            await sim.websocket_sim.receive('driveCommands', {
                'linear': 0.5,
                'angular': 0.0
            }, client_id)
            
            await asyncio.sleep(0.01)
            
            latency_ms = (time.time() - start) * 1000
            latencies.append(latency_ms)
        
        sim.shutdown()
        
        avg_latency = statistics.mean(latencies)
        
        print(f"\n📊 Default Mode Latency:")
        print(f"   Average: {avg_latency:.2f}ms")
        print(f"   Target: <{LATENCY_TARGETS['default_e2e']}ms")
        
        assert avg_latency < LATENCY_TARGETS['default_e2e'], \
            f"Average latency too high: {avg_latency:.2f}ms > {LATENCY_TARGETS['default_e2e']}ms"


class TestComponentLatency:
    """Test individual component latencies."""
    
    def test_slcan_encoding_latency(self):
        """Measure SLCAN encoding latency."""
        from simulation.can.slcan_protocol_simulator import create_slcan_simulator
        
        sim = create_slcan_simulator('perfect')
        
        latencies = []
        iterations = 100
        
        for i in range(iterations):
            start = time.time()
            sim.encode_velocity_command(0.5, 0.0, 0.1)
            latency_ms = (time.time() - start) * 1000
            latencies.append(latency_ms)
        
        avg_latency = statistics.mean(latencies)
        
        print(f"\n📊 SLCAN Encoding Latency:")
        print(f"   Average: {avg_latency:.3f}ms")
        print(f"   Target: <{LATENCY_TARGETS['slcan_encode']}ms")
        
        assert avg_latency < LATENCY_TARGETS['slcan_encode'], \
            f"Encoding latency too high: {avg_latency:.3f}ms"
    
    def test_firmware_response_latency(self):
        """Measure firmware response latency."""
        from simulation.firmware.stm32_firmware_simulator import create_firmware_simulator
        
        sim = create_firmware_simulator('default')
        sim.start()
        
        # Send command and measure time to see response
        start = time.time()
        sim.set_velocity_command(0, 5.0)
        
        # Wait for velocity to start changing
        for _ in range(100):  # Max 1 second
            status = sim.get_motor_status(0)
            if status['velocity_actual'] > 0.1:
                break
            time.sleep(0.01)
        
        response_time_ms = (time.time() - start) * 1000
        
        sim.stop()
        
        print(f"\n📊 Firmware Response Latency:")
        print(f"   Time: {response_time_ms:.1f}ms")
        print(f"   Target: <{LATENCY_TARGETS['firmware_response']}ms")
        
        assert response_time_ms < LATENCY_TARGETS['firmware_response'], \
            f"Response time too high: {response_time_ms:.1f}ms"
    
    @pytest.mark.asyncio
    async def test_network_delay_accuracy(self):
        """Measure network delay simulation accuracy."""
        from simulation.network.websocket_server_simulator import create_websocket_simulator
        
        sim = create_websocket_simulator('default')
        
        # Get configured latency
        configured_latency = sim.network_conditions.latency_ms
        
        latencies = []
        iterations = 10
        
        for i in range(iterations):
            start = time.time()
            await sim._apply_network_delay()
            actual_latency = (time.time() - start) * 1000
            latencies.append(actual_latency)
        
        avg_latency = statistics.mean(latencies)
        
        print(f"\n📊 Network Delay Simulation:")
        print(f"   Configured: {configured_latency}ms")
        print(f"   Actual: {avg_latency:.2f}ms")
        
        # Allow 20% tolerance
        tolerance = configured_latency * 0.2
        assert abs(avg_latency - configured_latency) < tolerance, \
            f"Network delay inaccurate: {avg_latency:.2f}ms vs {configured_latency}ms"


class TestLatencyPercentiles:
    """Test latency percentiles and distribution."""
    
    @pytest.mark.asyncio
    async def test_latency_distribution(self):
        """Measure complete latency distribution."""
        sim = create_full_stack_simulator('perfect')
        client_id = await sim.websocket_sim.connect()
        
        latencies = []
        iterations = 100
        
        for i in range(iterations):
            start = time.time()
            await sim.websocket_sim.receive('driveCommands', {
                'linear': 0.5,
                'angular': 0.0
            }, client_id)
            await asyncio.sleep(0.001)
            latency_ms = (time.time() - start) * 1000
            latencies.append(latency_ms)
        
        sim.shutdown()
        
        # Calculate percentiles
        p50 = statistics.median(latencies)
        p90 = statistics.quantiles(latencies, n=10)[8]
        p95 = statistics.quantiles(latencies, n=20)[18]
        p99 = statistics.quantiles(latencies, n=100)[98]
        
        print(f"\n📊 Latency Distribution:")
        print(f"   P50: {p50:.2f}ms")
        print(f"   P90: {p90:.2f}ms")
        print(f"   P95: {p95:.2f}ms")
        print(f"   P99: {p99:.2f}ms")
        
        # P95 should still be under target
        assert p95 < LATENCY_TARGETS['perfect_e2e'], \
            f"P95 latency too high: {p95:.2f}ms"


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
