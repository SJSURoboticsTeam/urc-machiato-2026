# 👁️ VISUAL HUMAN TESTING DASHBOARD - Quick Guide

## Purpose

This dashboard is designed for **HUMAN visual verification** of system integration. Instead of automated tests, YOU can manually interact with the system and visually confirm everything works correctly.

## 🎯 What You Can Test

### 1. **Send Test Messages**
- Choose target: WebSocket, ROS2, or CAN
- Type any message
- Click Send
- **Watch**: The target node will flash when it receives your message
- **Verify**: Message appears in history with delivery confirmation

### 2. **Test State Transitions**
- Click any state button (BOOT, READY, TELEOPERATION, etc.)
- **Watch**: Transition result appears immediately
- **Verify**: Current state updates in the header
- **Confirm**: Success/failure status is clearly shown

### 3. **Monitor Topic Data**
- Select any ROS2 topic from dropdown
- **Watch**: Live data updates every 100ms
- **Verify**: Values make sense (GPS coordinates, IMU readings, battery voltage)
- Toggle "Raw Data" to see JSON format

### 4. **Communication Flow Visualization**
- **Watch**: Nodes pulse when active
- **Verify**: Message counters increase (sent/received)
- **Confirm**: All three channels (WebSocket → ROS2 → CAN) are communicating

### 5. **System Telemetry**
- **Monitor**: Battery, GPS, Speed, Temperature
- **Verify**: Values update in real-time
- **Confirm**: No error states (red indicators)

---

## 🚀 Quick Start

```bash
cd /home/ubuntu/urc-machiato-2026
./scripts/testing/start_dashboard.sh
```

Then:
1. Open **http://localhost:5173**
2. Go to **Testing** tab
3. Click **"Integrated Dashboard"**
4. Start testing!

---

## 📋 Manual Testing Checklist

### Communication Tests
- [ ] Send message to WebSocket → See node flash
- [ ] Send message to ROS2 → See node flash
- [ ] Send message to CAN → See node flash (mock)
- [ ] Verify all messages get "Delivered" status
- [ ] Check sent/received counters increase

### State Machine Tests
- [ ] Transition BOOT → READY → Works?
- [ ] Transition READY → TELEOPERATION → Works?
- [ ] Transition TELEOPERATION → AUTONOMOUS → Works?
- [ ] Try invalid transition → Gets rejected?
- [ ] Current state updates in header?

### Topic Monitoring
- [ ] Subscribe to /gps/fix → See lat/lon values?
- [ ] Subscribe to /imu/data → See accel/gyro values?
- [ ] Subscribe to /battery/status → See voltage/current?
- [ ] Values update continuously?
- [ ] Raw data JSON looks correct?

### Visual Indicators
- [ ] Connected indicator is green?
- [ ] Active nodes have green pulse dot?
- [ ] Flashing works when sending messages?
- [ ] Metrics counters increment?
- [ ] Telemetry cards show data?

---

## 🎨 Visual Feedback Guide

| Visual Cue | Meaning |
|------------|---------|
| 🟢 Green dot pulsing | Node is active |
| ⚡ Node flashes briefly | Message received |
| ✓ Green checkmark | Test passed |
| ✗ Red X | Test failed |
| ⏳ Yellow spinner | Test in progress |
| 📊 Numbers increasing | Messages flowing |

---

## 🔧 What's Mocked vs Real

### MOCKED (for testing without hardware):
- ❌ CAN Bus data (clearly labeled as MOCK)
- ❌ Sensor readings (simulated realistic values)
- ❌ Topic data (generated values)

### REAL (connects to actual system):
- ✅ WebSocket connection status
- ✅ ROS2 topic list (when ROS2 running)
- ✅ State machine (when state director running)
- ✅ Message sending/receiving

---

## 🐛 Troubleshooting

### "Disconnected" status shows
- Check backend is running: `ps aux | grep test_dashboard_backend`
- Restart: `./scripts/testing/start_dashboard.sh`

### Messages not sending
- Verify you typed a message
- Check target channel is selected
- Ensure Connected indicator is green

### State transitions fail
- Check current state allows this transition
- Verify state machine director is running
- Check for error messages in transition result

### No topic data showing
- Ensure you selected a topic from dropdown
- Wait a few seconds for data to accumulate
- Check "Live" indicator is green

---

## 💡 Testing Tips

1. **Test the happy path first**: Send simple messages, do basic state transitions
2. **Then test edge cases**: Try invalid states, send malformed messages
3. **Watch for visual feedback**: Nodes should flash, counters should increase
4. **Check both directions**: Send AND receive on each channel
5. **Verify mock labels**: Ensure CAN data is clearly marked as MOCK

---

## 📊 What Success Looks Like

✅ **Communication**
- All nodes show green active dot
- Messages send and get delivered
- Counters increase on both sent/received
- Nodes flash when messages flow

✅ **State Machine**
- Transitions complete successfully
- Current state updates immediately
- Invalid transitions are rejected
- Success status shows green checkmark

✅ **Topics**
- Data updates continuously
- Values are reasonable (not NaN or errors)
- Raw JSON is well-formed
- Different topics show different data

✅ **Overall System**
- No red error indicators
- Connection stays green
- Telemetry updates
- Everything feels responsive

---

## 🎓 Understanding the Interface

```
┌─────────────────────────────────────────────────────────┐
│ Header: Connection Status + Current State              │
├──────────────┬──────────────────────────────────────────┤
│              │                                          │
│ LEFT PANEL   │         CENTER PANEL                    │
│              │                                          │
│ • Send Msg   │   ┌──────────┐                          │
│ • Test State │   │ WebSocket│→┌─────┐→┌──────────┐   │
│ • Subscribe  │   └──────────┘ │ROS2 │ │ CAN Bus  │   │
│              │                └─────┘ └──────────┘   │
│              │                                          │
│              │   [Metrics] [Topic Data] [Telemetry]   │
└──────────────┴──────────────────────────────────────────┘
```

### Left Panel (Manual Controls)
- **Message Sender**: Type and send test messages
- **State Tester**: Click buttons to change states
- **Topic Subscriber**: Select topics to monitor

### Center Panel (Visual Feedback)
- **Communication Flow**: See nodes flash and data flow
- **Metrics**: Counters show message activity
- **Topic Data**: Live values from subscribed topics
- **Telemetry**: System health indicators

---

## ✅ Ready to Test!

The dashboard is designed to be **intuitive and visual**. Just start clicking buttons, sending messages, and watch what happens!

**Key Philosophy**: If you can SEE it working, you can TRUST it working.

---

**Quick Commands**:
```bash
# Start everything
./scripts/testing/start_dashboard.sh

# Stop everything
Ctrl+C in the terminal

# Check logs
tail -f /tmp/test_backend.log
tail -f /tmp/frontend.log
```

**Access**: http://localhost:5173 → Testing Tab → Integrated Dashboard

🎉 **Happy Visual Testing!** 🎉
