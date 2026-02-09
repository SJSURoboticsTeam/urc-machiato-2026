# Full Testing with Mock Data and Dashboard Visualization – Development Plan

Plan to run **software systems on mock data** with **dashboard visualization**, without requiring live ROS or hardware for the mock path. **ROS integration is in scope**: the dashboard's connection to ROS (rosbridge, topics, services), connection state, error handling, and topic-driven updates must be maintained and verified; mock mode is an alternative data path when ROS is unavailable.

**Principles:** (1) Keep the dashboard **lean** (fewer tabs, no duplicate test/debug surfaces). (2) Integrate with existing systems only; extend `mockDataUtils`, WebSocket 8766 contract, and `TelemetryContext`; do not add new servers or protocols. (3) **ROS integration remains the primary path** when connected; mock mode only supplements or replaces it when unconnected.

**Cross-references:** `dashboard_and_bt_improvement_plan.md`, `BUILD_AND_TEST.md`, `AGENTS.md`.

---

## 1. Dashboard scope: lean and unbloated

The plan assumes a **consolidated dashboard**. Today there are 8 tabs with overlap (Monitor, Testing, and Debug all do diagnostics). Target:

| Target tabs | Contents | Rationale |
|-------------|----------|-----------|
| **Overview** | State badge, telemetry summary, mission summary, alerts | Single home view. |
| **Mission** | Mission list, progress, execution | Mission-only. |
| **Network** | CAN/ROS nodes, connections, traffic | Network-only. |
| **Debug** | State strip, sensors, blackboard, BT, message tester, monitoring health | One place for all debugging and test-style views (merge current Monitor + Testing + Debug). |
| **Config** | Settings, optional analytics summary | Single settings/analytics surface. |

**Consolidation (do once, then the rest of the plan applies):**

- Merge **Monitor** and **Testing** into **Debug**: move MonitoringDashboard and TestingTab content (e.g. MessageTester, ThreeColumnTestingDashboard, component tests) into DebuggingDashboard as sub-panels or sub-tabs. Remove the separate "Monitor" and "Testing" top-level tabs.
- Fold **Analytics** into **Debug** (e.g. one "Analytics" panel) or into **Config**; remove the standalone Analytics tab.
- Result: **5 tabs** (Overview, Mission, Network, Debug, Config). No new features; only reorganization and removal of redundant tabs.

This consolidation is a **prerequisite** (or Phase 0) so the mock-data plan does not have to verify 8 tabs or maintain three separate test surfaces.

---

## 2. Goal and scope (mock data plan)

### 2.1 Objective

- Run state machine, telemetry, CAN, mission (and optionally BT/blackboard) in **mock-data-only** mode when ROS/8766 are unavailable.
- **Keep ROS integration working**: connection (rosbridge), topics, services, connection state, retry/error UI; when ROS is available, it remains the primary data source.
- Visualize in the **lean dashboard** (5 tabs above).
- Support a **minimal set of scenarios** and one command to run tests.

### 2.2 In scope

| System | Dashboard surface (lean) | Data path |
|--------|---------------------------|-----------|
| **ROS integration** | TopBar (connection/retry), all tabs that subscribe to ROS | ROSContext, useROS, rosbridge, topic/service subscriptions; validation of ROS messages (Zod); connection state and error handling. |
| State machine | Overview, TopBar, Debug | ROS topics + useStateMachine; context |
| Telemetry / sensors | Overview, Debug | WebSocket 8766, TelemetryContext, mockDataUtils; ROS when connected |
| CAN | Network, Debug | 8766 simulation_data, useNetworkData; ROS `/can/sensor_data` when connected |
| Mission | Mission tab, TopBar | Context / useMissionRos (ROS topics when connected) |
| BT / blackboard | Debug (one panel) | ROS `/bt/telemetry` or inline mock |

### 2.3 Out of scope

- Requiring **live ROS or hardware** to run the mock test suite (mock path must work without them).
- Gazebo simulation.
- New WebSocket server or new protocol.
- More than 5 tabs; new standalone Monitor/Analytics/Testing tabs.
- Teleop frontend migration.

### 2.4 Existing systems to reuse

- **ROS:** `ROSContext`, `useROS`, `rosbridge`, validation schemas for ROS messages – keep as primary path when connected; mock mode is alternative when disconnected.
- **8766 and schemas:** `simulationMessageSchema`, `TelemetryContext`, `useNetworkData` – same message shape for mock.
- **Mock data:** `mockDataUtils.js` – extend, do not duplicate.
- **Backend (optional):** `tests/testing/test_dashboard_backend.py` – extend with scenario mode only if you want server-driven mock; otherwise in-dashboard only.

---

## 3. Minimal path (do this first)

If you only need "dashboard runs on mock data, one scenario, one command":

| Step | Action |
|------|--------|
| 1 | In `TelemetryContext.tsx`, when `import.meta.env.VITE_USE_MOCK === '1'`, do not open `ws://localhost:8766`. Use `setInterval` (e.g. 500 ms) and call `setTelemetry` / `setSystemStatus` with one `simulation_data`-shaped object from `mockDataUtils` (extend it with one helper that returns that shape if needed). |
| 2 | In `BUILD_AND_TEST.md`, add: "Dashboard with mock data (no server): `VITE_USE_MOCK=1 npm run dev` (in `src/dashboard`)." |
| 3 | Optional: one Vitest test that enables mock mode and asserts e.g. state badge or battery in the DOM. |

Stop here unless you need repeatable scenarios and CI.

---

## 4. Extended plan (3 milestones, lean dashboard only)

Only if you need **repeatable scenarios**, **documented expectations**, and **one script + runbook**.

### Milestone A: Mock data and injection (1–2 weeks)

**Goal:** One scenario module and in-dashboard mock mode; no new server.

| Deliverable | What to do |
|-------------|------------|
| A.1 Scenario module | Add `src/dashboard/src/utils/mockScenarios.js`: export `getSimulationDataForScenario(name, t)` returning a `simulation_data`-shaped object (use `mockDataUtils` + existing schemas). Support **3 scenarios**: `idle`, `mission_active`, `sensor_degraded`. Deterministic for given `t` or seed. |
| A.2 Mock mode | In `TelemetryContext`, when `VITE_USE_MOCK=1`, skip 8766; `setInterval` and call `setTelemetry`/`setSystemStatus` from `getSimulationDataForScenario('idle', Date.now()/1000)` (or from a single scenario chosen by URL param `?mock=idle`). Optional: when mock, skip ROS connect in useROS so TopBar shows "disconnected" but data still flows. |
| A.3 useNetworkData in mock | When mock mode, either skip 8766 in useNetworkData and feed network state from the same scenario (e.g. derived in TelemetryContext or a small shared mock updater), or accept "disconnected" on Network tab until backend scenario mode exists. Prefer in-dashboard only (no backend change). |

**Do not:** Add a new server, new protocol, or more than 3 scenarios.

---

### Milestone B: Verify lean dashboard and document (1 week)

**Goal:** Confirm all **5 tabs** show mock data where relevant; one short doc with expectations.

| Deliverable | What to do |
|-------------|------------|
| B.1 Per-tab check | With mock mode on and scenario `idle` or `mission_active`: Overview shows state and telemetry; Mission shows mission data if provided by scenario/context; Network shows state (or "no connection" if not fed); Debug shows state strip, sensors, no crashes; Config loads. Fix only guards that hide content when `!isConnected` so mock data still shows. When mock mode is **off**, ROS integration must still work: connection to 9090, TopBar connection state and Retry, topic-driven updates in tabs that use ROS. |
| B.2 Scenario doc | Add `docs/development/mock_test_scenarios.md`: table of 3 scenarios (name, description, what to expect on Overview/Debug). One runbook paragraph in `BUILD_AND_TEST.md`: how to run `VITE_USE_MOCK=1 npm run dev`, open dashboard, switch scenario via `?mock=`, verify. |

**Do not:** Add new tabs or new panels; only verify and document.

---

### Milestone C: Automate and single command (1 week)

**Goal:** Vitest scenario tests and one script; optional CI.

| Deliverable | What to do |
|-------------|------------|
| C.1 Vitest scenario tests | Add tests (e.g. in `context/__tests__/` or `mock/__tests__/`) that enable mock mode and scenario (e.g. provider or env), render App or Overview+TopBar, assert state badge text and at least one telemetry value. Cover at least 2 of the 3 scenarios. |
| C.2 Single script | Add `scripts/run_full_mock_test.sh`: run backend pytest (tests that don’t need ROS), then `cd src/dashboard && npm run test:run`. Document in BUILD_AND_TEST.md. |
| C.3 Optional CI | One CI job that runs the script. |

**Do not:** New test framework or E2E; use Vitest + jsdom only.

---

## 5. Timeline (extended plan)

| Phase | Milestone | Duration |
|-------|-----------|----------|
| 0 (prerequisite) | Consolidate dashboard to 5 tabs (merge Monitor + Testing + Debug; fold Analytics) | 1–2 weeks |
| A | Mock data + injection | 1–2 weeks |
| B | Verify + document | 1 week |
| C | Automate + script | 1 week |

**Total (after consolidation):** about **3–4 weeks** for the mock-data plan. Consolidation is one-time; afterward the dashboard stays lean.

---

## 6. Success criteria

1. **Dashboard is lean:** 5 tabs (Overview, Mission, Network, Debug, Config); no separate Monitor, Testing, or Analytics tabs.
2. **ROS integration intact:** With mock mode off, dashboard connects to rosbridge (9090), shows connection state and Retry on error, and tabs that subscribe to ROS topics receive and display data when ROS is available.
3. **Mock mode works:** `VITE_USE_MOCK=1 npm run dev` runs the dashboard with mock data in at least Overview and Debug without 8766/9090.
4. **Scenarios:** 3 named scenarios (idle, mission_active, sensor_degraded) with documented expected state.
5. **Single command:** One script runs backend + dashboard tests; runbook updated in BUILD_AND_TEST.md.

---

## 7. References

- **Dashboard plan:** `docs/dashboard_and_bt_improvement_plan.md`
- **Build and test:** `docs/development/BUILD_AND_TEST.md`
- **ROS integration:** `src/dashboard/src/context/ROSContext.tsx`, `src/dashboard/src/hooks/useROS.js`, `src/dashboard/src/utils/rosbridge.js`; validation in `validationSchemas.js` for ROS message types.
- **Mock utils:** `src/dashboard/src/utils/mockDataUtils.js`
- **Validation:** `src/dashboard/src/utils/validationSchemas.js`
- **8766 consumers:** `TelemetryContext.tsx`, `useNetworkData.ts`
- **Backend (optional):** `tests/testing/test_dashboard_backend.py`
