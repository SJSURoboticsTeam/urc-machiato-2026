# Mock Test Scenarios

Deterministic mock scenarios for testing the dashboard without ROS or hardware.

## Available Scenarios

### idle

**State:** IDLE  
**Battery:** 85% (fixed)  
**GPS:** Fixed at (37.7749, -122.4194)  
**CAN:** All sensors normal  
**Mission:** None  

**Expected Dashboard State:**
- TopBar: Green "IDLE" badge
- Overview: Battery shows 85% (within 1%)
- Debug: All sensors show values, no errors
- Mission: Empty state
- Network: CAN status "operational"

**Usage:** `VITE_USE_MOCK=1 npm run dev` or open `http://localhost:5173?mock=idle` with mock mode.

---

### mission_active

**State:** EXECUTING  
**Battery:** 70% down to 65% over time (drains ~0.1%/sec)  
**GPS:** Moving along a path (latitude varies with time)  
**CAN:** All sensors normal  
**Mission:** mission_001, progress increases ~2% per second, waypoints completed every 10 seconds  

**Expected Dashboard State:**
- TopBar: Blue "EXECUTING" badge
- Overview: Battery decreases over time, GPS updates
- Mission: Shows mission_001, progress bar advances
- Debug: Waypoints completed increases over time

**Usage:** `http://localhost:5173?mock=mission_active` (with `VITE_USE_MOCK=1 npm run dev`).

---

### sensor_degraded

**State:** PAUSED  
**Battery:** 60%  
**GPS:** Drifting (simulates loss of fix; latitude drifts with time)  
**CAN:** IMU and Compass offline (degraded)  
**Mission:** mission_002 paused with error "Sensor timeout: IMU, Compass"  

**Expected Dashboard State:**
- TopBar: Yellow "PAUSED" badge
- Overview: Battery 60%, alerts possible for sensors
- Network: CAN status "degraded"
- Mission: Shows error message and paused mission

**Usage:** `http://localhost:5173?mock=sensor_degraded` (with mock mode).

---

## Running with Mock Data

### Quick start

```bash
cd src/dashboard
VITE_USE_MOCK=1 npm run dev
```

Open http://localhost:5173. Default scenario is `idle`.

### Switching scenarios via URL

Add `?mock=<scenario>` to the URL:

- http://localhost:5173?mock=idle
- http://localhost:5173?mock=mission_active
- http://localhost:5173?mock=sensor_degraded

### NPM scripts

From `src/dashboard`:

- `npm run dev:mock` – same as `VITE_USE_MOCK=1 npm run dev` (if script is configured)
- `VITE_USE_MOCK=1 npm run test:run` – run all tests with mock mode enabled

### Full mock test suite

From project root:

```bash
./scripts/run_full_mock_test.sh
```

Runs frontend tests in mock mode, scenario/data-source unit tests, and mock data flow integration test.

---

## Debugging

**Check mock mode is active:**  
Console should show: `[MockDataSource] Connecting to scenario: <name>`

**"WebSocket connection failed" with mock expected:**  
Ensure `VITE_USE_MOCK=1` when starting the dev server.

**Unknown scenario:**  
Use one of: `idle`, `mission_active`, `sensor_degraded`.

**Stale data:**  
Hard refresh (Ctrl+Shift+R) or reload after changing `?mock=`.

**Data source type in UI:**  
When mock is active, `useTelemetryContext().dataSource?.getSourceType()` is `'mock'`.

---

## How to view the dashboard with mock data

1. **Start the dev server with mock mode:**
   ```bash
   cd src/dashboard
   npm run dev:mock
   ```
   Or: `VITE_USE_MOCK=1 npm run dev`

2. **Open the dashboard in the browser:**  
   Go to **http://localhost:5173** (not the coverage report or a file:// URL).  
   The dashboard uses a dark theme (dark grey background). If you see a completely white page, you may be on the wrong URL (e.g. `coverage/index.html`) or there is a JavaScript error; open DevTools (F12) and check the Console.

3. **Switch scenario:**  
   Change the URL to include `?mock=<scenario>`:
   - http://localhost:5173?mock=idle
   - http://localhost:5173?mock=mission_active
   - http://localhost:5173?mock=sensor_degraded

4. **Confirm mock is active:**  
   In the browser console you should see: `[MockDataSource] Connecting to scenario: <name>`.  
   No ROS or WebSocket connection to 9090/8766 is required.

---

## How to view test coverage

1. **Generate the coverage report:**
   ```bash
   cd src/dashboard
   npm run test:coverage
   ```

2. **Open the HTML report:**  
   Open in a browser:
   ```bash
   src/dashboard/coverage/index.html
   ```
   Or from the dashboard directory: `coverage/index.html`  
   (e.g. `file:///path/to/urc-machiato-2026/src/dashboard/coverage/index.html`)

   The report shows per-file statement, branch, function, and line coverage and highlights uncovered lines.
