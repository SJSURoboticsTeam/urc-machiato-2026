#!/usr/bin/env python3
"""
WebSocket Bridge Demo

Demonstrates WebSocket server simulator usage for testing
teleoperation interface without hardware.

Author: URC 2026 Examples Team
"""

import asyncio
import sys
from pathlib import Path

# Add simulation to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from simulation.network.websocket_server_simulator import (
    create_websocket_simulator, ConnectionState
)


async def basic_communication_demo():
    """Demonstrate basic WebSocket communication."""
    print("\n=== Basic Communication Demo ===")
    
    # Create simulator
    sim = create_websocket_simulator('perfect')
    
    # Track received messages
    received_commands = []
    
    async def handle_drive_command(data):
        received_commands.append(data)
        print(f"  Received: linear={data.get('linear'):.2f}, angular={data.get('angular'):.2f}")
    
    # Register handler
    sim.on('driveCommands', handle_drive_command)
    
    # Simulate client connection
    client_id = await sim.connect()
    print(f"✅ Client connected: {client_id[:8]}...")
    
    # Send commands
    print("\n📤 Sending commands...")
    test_commands = [
        {'linear': 0.5, 'angular': 0.0},  # Forward
        {'linear': 0.0, 'angular': 0.2},  # Rotate
        {'linear': 0.3, 'angular': 0.1},  # Combined
    ]
    
    for cmd in test_commands:
        await sim.receive('driveCommands', cmd, client_id)
    
    await asyncio.sleep(0.1)
    
    # Statistics
    stats = sim.get_statistics()
    print(f"\n📊 Statistics:")
    print(f"  Messages received: {stats['stats']['messages_received']}")
    print(f"  Commands processed: {len(received_commands)}")
    
    # Cleanup
    await sim.disconnect(client_id)
    print("✅ Demo complete!\n")


async def network_conditions_demo():
    """Demonstrate network condition simulation."""
    print("\n=== Network Conditions Demo ===")
    
    # Create with realistic network
    sim = create_websocket_simulator('default')
    print(f"Network latency: {sim.network_conditions.latency_ms}ms")
    print(f"Packet loss: {sim.network_conditions.packet_loss_rate*100:.1f}%")
    
    # Connect client
    client_id = await sim.connect()
    
    # Send messages and measure latency
    print("\n⏱️  Measuring latency...")
    latencies = []
    
    for i in range(10):
        import time
        start = time.time()
        await sim.emit('test', {'id': i}, client_id)
        latency = (time.time() - start) * 1000
        latencies.append(latency)
    
    avg_latency = sum(latencies) / len(latencies)
    print(f"Average latency: {avg_latency:.1f}ms")
    
    # Test with degraded network
    print("\n🌐 Simulating poor network...")
    sim.set_network_conditions({
        'latency_ms': 200.0,
        'packet_loss_rate': 0.1  # 10% loss
    })
    
    # Send more messages
    sent = 0
    for i in range(100):
        await sim.emit('test', {'id': i}, client_id)
        sent += 1
    
    stats = sim.get_statistics()
    dropped = stats['stats'].get('messages_dropped', 0)
    print(f"Sent: {sent}, Dropped: {dropped}")
    print(f"Delivery rate: {(1 - dropped/sent)*100:.1f}%")
    
    print("✅ Network demo complete!\n")


async def multiple_clients_demo():
    """Demonstrate multi-client handling."""
    print("\n=== Multiple Clients Demo ===")
    
    sim = create_websocket_simulator('perfect')
    
    # Connect multiple clients
    clients = []
    print("👥 Connecting clients...")
    for i in range(3):
        client_id = await sim.connect()
        clients.append(client_id)
        print(f"  Client {i+1}: {client_id[:8]}...")
    
    # Broadcast to all
    print("\n📢 Broadcasting message...")
    await sim.emit('broadcast', {'message': 'Hello everyone!'})
    
    stats = sim.get_statistics()
    print(f"Messages sent: {stats['stats']['messages_sent']}")  # Should be 3
    
    # Send to specific client
    print("\n📧 Sending to specific client...")
    await sim.emit('private', {'message': 'Just for you!'}, clients[0])
    
    # Disconnect clients
    print("\n👋 Disconnecting clients...")
    for client_id in clients:
        await sim.disconnect(client_id)
    
    print(f"Connected clients: {stats['connected_clients']}")
    print("✅ Multi-client demo complete!\n")


async def message_recording_demo():
    """Demonstrate message history recording."""
    print("\n=== Message Recording Demo ===")
    
    sim = create_websocket_simulator('perfect')
    client_id = await sim.connect()
    
    # Send various messages
    print("📝 Sending messages...")
    events = [
        ('driveCommands', {'linear': 0.5, 'angular': 0.0}),
        ('systemStatus', {'battery': 95.0}),
        ('cameraFeed', {'frame_id': 1234}),
    ]
    
    for event_name, data in events:
        await sim.receive(event_name, data, client_id)
    
    # Retrieve history
    print("\n📚 Message history:")
    history = sim.get_message_history(count=10)
    
    for i, msg in enumerate(history, 1):
        print(f"  {i}. {msg['event_name']}: {msg['direction']}")
    
    # Export history
    import tempfile
    import json
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        export_path = f.name
    
    sim.export_history(export_path)
    print(f"\n💾 History exported to: {export_path}")
    
    # Load and display
    with open(export_path, 'r') as f:
        data = json.load(f)
    print(f"Total messages in export: {len(data['message_history'])}")
    
    print("✅ Recording demo complete!\n")


async def main():
    """Run all demos."""
    print("=" * 60)
    print("WebSocket Server Simulator Demo")
    print("=" * 60)
    
    try:
        await basic_communication_demo()
        await network_conditions_demo()
        await multiple_clients_demo()
        await message_recording_demo()
        
        print("=" * 60)
        print("✅ All demos completed successfully!")
        print("=" * 60)
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    asyncio.run(main())
