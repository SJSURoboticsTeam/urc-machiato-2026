#!/usr/bin/env python3
"""
Simulation Throughput Performance Tests

Measures message processing throughput for all simulation components.

Author: URC 2026 Performance Testing Team
"""

import pytest
import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from simulation.can.slcan_protocol_simulator import create_slcan_simulator
from simulation.firmware.stm32_firmware_simulator import create_firmware_simulator
from simulation.network.websocket_server_simulator import create_websocket_simulator


# Performance targets (from baseline)
TARGETS = {
    'slcan_encoding': 10000,  # frames/sec
    'slcan_decoding': 10000,  # frames/sec
    'websocket_messages': 500,  # messages/sec
    'firmware_updates': 100,  # updates/sec (control loop)
}


class TestSLCANThroughput:
    """Test SLCAN protocol throughput."""
    
    def test_encoding_throughput(self):
        """Measure SLCAN encoding throughput."""
        sim = create_slcan_simulator('perfect')
        
        count = 10000
        start = time.time()
        
        for i in range(count):
            sim.encode_velocity_command(0.5, 0.0, 0.1)
        
        duration = time.time() - start
        throughput = count / duration
        
        print(f"\n📊 SLCAN Encoding: {throughput:.0f} frames/sec")
        print(f"   Target: {TARGETS['slcan_encoding']} frames/sec")
        
        assert throughput > TARGETS['slcan_encoding'], \
            f"Encoding too slow: {throughput:.0f} < {TARGETS['slcan_encoding']}"
    
    def test_decoding_throughput(self):
        """Measure SLCAN decoding throughput."""
        sim = create_slcan_simulator('perfect')
        
        # Pre-encode frames
        frame = sim.encode_velocity_command(0.5, 0.0, 0.1)
        
        count = 10000
        start = time.time()
        
        for i in range(count):
            sim.decode_velocity_command(frame)
        
        duration = time.time() - start
        throughput = count / duration
        
        print(f"\n📊 SLCAN Decoding: {throughput:.0f} frames/sec")
        print(f"   Target: {TARGETS['slcan_decoding']} frames/sec")
        
        assert throughput > TARGETS['slcan_decoding'], \
            f"Decoding too slow: {throughput:.0f} < {TARGETS['slcan_decoding']}"
    
    def test_roundtrip_throughput(self):
        """Measure complete encode/decode cycle throughput."""
        sim = create_slcan_simulator('perfect')
        
        count = 5000
        start = time.time()
        
        for i in range(count):
            frame = sim.encode_velocity_command(0.5, 0.0, 0.1)
            cmd = sim.decode_velocity_command(frame)
        
        duration = time.time() - start
        throughput = count / duration
        
        print(f"\n📊 SLCAN Roundtrip: {throughput:.0f} cycles/sec")
        
        # Target should be at least half of encoding rate
        min_target = TARGETS['slcan_encoding'] / 2
        assert throughput > min_target, \
            f"Roundtrip too slow: {throughput:.0f} < {min_target}"


class TestWebSocketThroughput:
    """Test WebSocket server throughput."""
    
    @pytest.mark.asyncio
    async def test_message_receiving_throughput(self):
        """Measure WebSocket message receiving throughput."""
        sim = create_websocket_simulator('perfect')
        
        received = []
        sim.on('test', lambda data: received.append(data))
        
        client_id = await sim.connect()
        
        count = 1000
        start = time.time()
        
        for i in range(count):
            await sim.receive('test', {'id': i}, client_id)
        
        duration = time.time() - start
        throughput = count / duration
        
        print(f"\n📊 WebSocket Receiving: {throughput:.0f} msg/sec")
        print(f"   Target: {TARGETS['websocket_messages']} msg/sec")
        
        assert throughput > TARGETS['websocket_messages'], \
            f"Receiving too slow: {throughput:.0f} < {TARGETS['websocket_messages']}"
    
    @pytest.mark.asyncio
    async def test_message_emitting_throughput(self):
        """Measure WebSocket message emitting throughput."""
        sim = create_websocket_simulator('perfect')
        
        client_id = await sim.connect()
        
        count = 1000
        start = time.time()
        
        for i in range(count):
            await sim.emit('test', {'id': i}, client_id)
        
        duration = time.time() - start
        throughput = count / duration
        
        print(f"\n📊 WebSocket Emitting: {throughput:.0f} msg/sec")
        print(f"   Target: {TARGETS['websocket_messages']} msg/sec")
        
        assert throughput > TARGETS['websocket_messages'], \
            f"Emitting too slow: {throughput:.0f} < {TARGETS['websocket_messages']}"


class TestFirmwareThroughput:
    """Test firmware simulator throughput."""
    
    def test_control_loop_frequency(self):
        """Measure firmware control loop frequency."""
        sim = create_firmware_simulator('default')
        sim.start()
        
        initial_cycles = sim.stats['control_cycles']
        start = time.time()
        
        time.sleep(2.0)
        
        elapsed = time.time() - start
        cycles = sim.stats['control_cycles'] - initial_cycles
        
        frequency = cycles / elapsed
        
        sim.stop()
        
        print(f"\n📊 Control Loop Frequency: {frequency:.1f} Hz")
        print(f"   Target: {TARGETS['firmware_updates']} Hz")
        
        # Allow 10% tolerance
        min_freq = TARGETS['firmware_updates'] * 0.9
        max_freq = TARGETS['firmware_updates'] * 1.1
        
        assert min_freq < frequency < max_freq, \
            f"Frequency out of range: {frequency:.1f} not in [{min_freq}, {max_freq}]"
    
    def test_command_processing_rate(self):
        """Measure command processing rate."""
        sim = create_firmware_simulator('default')
        
        count = 1000
        start = time.time()
        
        for i in range(count):
            sim.set_velocity_command(0, 5.0)
        
        duration = time.time() - start
        rate = count / duration
        
        print(f"\n📊 Command Processing: {rate:.0f} commands/sec")
        
        # Should process commands much faster than control loop
        assert rate > 1000, f"Command processing too slow: {rate:.0f} < 1000"


@pytest.mark.benchmark
class TestIntegratedThroughput:
    """Test integrated system throughput."""
    
    @pytest.mark.asyncio
    async def test_end_to_end_message_rate(self):
        """Measure end-to-end message processing rate."""
        from simulation.integration.full_stack_simulator import create_full_stack_simulator
        
        sim = create_full_stack_simulator('perfect')
        client_id = await sim.websocket_sim.connect()
        
        count = 500
        start = time.time()
        
        for i in range(count):
            await sim.websocket_sim.receive('driveCommands', {
                'linear': 0.5,
                'angular': 0.0
            }, client_id)
        
        duration = time.time() - start
        throughput = count / duration
        
        sim.shutdown()
        
        print(f"\n📊 End-to-End Throughput: {throughput:.0f} msg/sec")
        print(f"   Target: {TARGETS['websocket_messages'] * 0.5:.0f} msg/sec (50% of WebSocket)")
        
        # E2E should be at least 50% of WebSocket rate
        min_target = TARGETS['websocket_messages'] * 0.5
        assert throughput > min_target, \
            f"E2E throughput too low: {throughput:.0f} < {min_target}"


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
