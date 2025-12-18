#!/usr/bin/env python3
"""
Test script to verify WebSocket connection to simulation bridge.
"""

import asyncio
import json
import time

import websockets


async def test_websocket_connection():
    """Test WebSocket connection to simulation bridge."""
    uri = "ws://localhost:8766"

    print(f"🔌 Connecting to {uri}...")

    try:
        async with websockets.connect(uri) as websocket:
            print("✅ Connected to simulation bridge")

            # Wait for initial connection message
            response = await websocket.recv()
            data = json.loads(response)
            print(f"📨 Received: {data.get('type', 'unknown')}")

            if data.get("type") == "simulation_connected":
                print("✅ Simulation bridge confirmed connection")

                # Request current state
                request = {"type": "request_state"}
                await websocket.send(json.dumps(request))
                print("📤 Sent state request")

                # Listen for data updates
                start_time = time.time()
                message_count = 0

                while time.time() - start_time < 10:  # Test for 10 seconds
                    try:
                        response = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                        data = json.loads(response)

                        if data.get("type") == "simulation_update":
                            message_count += 1
                            sim_data = data.get("simulation_data", {})

                            # Print sample data
                            if message_count == 1:
                                print("📊 Sample simulation data received:")
                                if "gps" in sim_data:
                                    gps = sim_data["gps"]
                                    print(
                                        f"  GPS: {gps.get('latitude', 'N/A'):.6f}, {gps.get('longitude', 'N/A'):.6f}"
                                    )
                                if "imu" in sim_data:
                                    imu = sim_data["imu"]
                                    print(
                                        f"  IMU: accel_x={imu.get('accel_x', 'N/A'):.3f}"
                                    )

                        elif data.get("type") == "simulation_state":
                            print("📊 Received simulation state response")

                    except asyncio.TimeoutError:
                        # No message received, continue
                        pass

                print(f"📈 Received {message_count} simulation updates in 10 seconds")
                return message_count > 0

            else:
                print(f"❌ Unexpected initial message: {data}")
                return False

    except Exception as e:
        print(f"❌ WebSocket connection failed: {e}")
        return False


def main():
    """Main test function."""
    print("🧪 Testing WebSocket Connection to Simulation Bridge")
    print("=" * 55)

    # Run the async test
    success = asyncio.run(test_websocket_connection())

    print("\n📊 Test Results:")
    if success:
        print("✅ PASS: WebSocket connection successful, receiving simulation data")
    else:
        print("❌ FAIL: WebSocket connection failed or no data received")


if __name__ == "__main__":
    main()
