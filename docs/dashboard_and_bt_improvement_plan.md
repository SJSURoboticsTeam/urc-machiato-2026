# Dashboard Overhaul and Behavior Tree Testing – Comprehensive Plan

Unified improvement plan for the URC 2026 dashboard (performance, architecture, quality, testing) and behavior tree testing (unit, integration, performance, mission scenarios). Cross-references: `docs/debugging_interface_plan.md`, `docs/behavior_tree_visualization_and_testing_plan.md`, `AGENTS.md`.

---

## Part A: Dashboard Overhaul

### Phase 1: Foundation (Week 1–2) – Performance & Reliability

| Item | Description |
|------|-------------|
| **Split SystemContext** | Replace monolithic `SystemContext` with four focused contexts: **ROSContext** (connection, URL, reconnect), **TelemetryContext** (sensor/telemetry state, debounced updates), **StateMachineContext** (current state, substate, transitions, `requestStateTransition`), **UIContext** (alerts, mission UI state, offline cache). |
| **Error boundaries** | Ensure every tab content is wrapped in an error boundary (e.g. `SectionErrorBoundary` in debugging; add or reuse for Overview, Mission, Network, Testing, Analytics, Config) so one tab failure does not crash the app. |
| **Debounced telemetry** | Throttle/debounce telemetry updates to the UI (e.g. 100 ms) so high-frequency ROS data does not cause excessive re-renders. |
| **ROS message validation** | Validate incoming ROS message shape (and optionally types) before updating state; reject or sanitize invalid payloads to avoid crashes and bad UI state. |

**Phase 1 complete.** Context split (ROS, StateMachine, Telemetry, UIContext in AppProviders). Every tab wrapped in SectionErrorBoundary. TelemetryContext uses `useDebouncedCallback(..., 100)` for setTelemetry (100ms debounce). All ROS message consumers validate with Zod before updating state: `useStateMachine` (stateMachineCurrentStateSchema, stateMachineTransitionSchema), `useSafetySystem` (safetyPayloadSchema), `SensorDashboardContainer` (sensorAnalyticsSchema), `MonitoringDashboard` (communicationHealthSchema), `MapVisualization` (mapPathSchema, waypointsSchema), `AOITesting` (aoiStatusSchema, aoiMetricsSchema), `ThreeColumnTestingDashboard` (simulationMessageSchema); mission/network already used parseAndValidate. Invalid payloads are rejected and not applied.

---

### Phase 2: Architecture (Week 3–4) – Component Restructuring

| Item | Description |
|------|-------------|
| **Custom hooks** | Extract 15+ focused hooks, e.g.: `useTelemetry()`, `useStateMachine()` (already exists), `useAlerts()`, `useROSConnection()`, `useMissionControl()`, plus hooks for topics/subscriptions to keep components thin. |
| **Break down large components** | Split **OverviewTab** (~319 lines) into 3 focused components; split **TestingTab** (~360 lines) into 4 specialized components. Identify other tabs or panels >200 lines and split by responsibility. |
| **Virtualization** | Use list virtualization (e.g. `react-window` or similar) for logs, message history, and long lists to keep 60fps with large datasets. |
| **React.memo** | Apply `React.memo` to expensive presentational components (charts, grids, long lists) where props are stable to reduce unnecessary re-renders. |

**Phase 2 progress (with Phase 3 in mind):** DebugTab split into `tabs/debug/` (DebugLogsPanel with virtualized list, DebugStatePanel, DebugTopicsPanel, DebugPerformancePanel, DebugNetworkPanel); NetworkTab split into `tabs/network/` with `useNetworkData` hook and memoized NetworkNode, NetworkConnection, NodeInspector, NetworkHeader, DataDisplay; MissionTab split into `tabs/mission/` with `useMissionRos`, MissionExecutionView, MissionPlannerView, MissionTemplateGrid. Added `react-window` and `VirtualizedLogList` for logs. Extended `statusUtils` with `getLogLevelColor` and `getAlertTypeColor` (consolidation for Phase 3). React.memo applied to cards (TelemetryCard, SystemStatusCard, MissionProgressCard) and all new presentational panels. Clear JSDoc and prop interfaces for future TypeScript migration.

**Phase 1 (verified):** Context already split (ROSContext, StateMachineContext, TelemetryContext, UIContext in AppProviders). Every tab content in App.jsx is wrapped in SectionErrorBoundary.

**Phase 3 (done):** Shared `.d.ts`: `types/ros.d.ts` (ROS message payloads), `types/network.d.ts` (NetworkNode, CanDataState, etc.), `types/stateMachine.d.ts` extended (ActiveMission.id, MissionTemplate). Zod validation: `validationSchemas.js` extended with `missionProgressSchema`, `missionStatusSchema`, `canSensorDataSchema`, `commandUpdateMessageSchema`, `parseAndValidate()`; used in `useMissionRos` and `useNetworkData`. Hooks migrated to TypeScript: `useMissionRos.ts`, `useNetworkData.ts`.

**Virtualization:** `VirtualizedList` (generic) added in `components/ui/`; MessageTester message history uses it. Tests mock VirtualizedList so react-window is not required in jsdom.

---

### Phase 3: Code Quality (Week 5–6) – TypeScript & Validation

| Item | Description |
|------|-------------|
| **TypeScript migration** | Gradual migration of core dashboard components and hooks to TypeScript (e.g. context providers, hooks, then high-traffic components). |
| **Consolidate utilities** | Centralize duplicate logic: status colors, number/date formatting, connection state labels in one place (e.g. `utils/statusUtils.js` or `utils/formatting.js`). |
| **ROS message validation** | Add schema validation (e.g. Joi or Zod) for all ROS message types used by the dashboard; validate on receive and log or reject invalid data. |
| **Shared types** | Create shared type definitions (`.d.ts` or `.ts`) for ROS payloads, telemetry, state machine states, and mission data. |
| **Error recovery** | Define retry/backoff for ROS connection and services; clear user-visible error state when recovery succeeds. |

**Phase 3 complete.** TypeScript: all four context providers and AppProviders migrated to `.tsx` with `types/context.d.ts`; TopBar and TabNavigation migrated to `.tsx`; useMissionRos and useNetworkData already in `.ts`. Consolidate utilities: `getStatusTextColor` extended with `healthy`/`unhealthy`; MonitoringDashboard uses `getStatusTextColor` and `formatRelativeTime` from `utils/formatting.js`; TopBar uses `getConnectionStateLabel`. `formatRelativeTime(timestamp)` added for relative time (Never, Just now, Xs ago, Xm ago, Xh ago). Error recovery: useROS uses exponential backoff for reconnection (delay = min(interval * 2^attempt, 30s)); on successful connection `lastError` and reconnect count are cleared. ROSContext now exposes `lastError`, `reconnectAttempts`, `resetReconnection` for UI. ROS message validation and shared types were completed in Phase 1 and earlier Phase 3 work.

---

### Phase 4: Testing (Week 7) – Comprehensive Testing

| Item | Description |
|------|-------------|
| **Integration tests** | Add integration tests for ROS flows (connect, subscribe, receive message, update UI) using test bridge or mock ROS. |
| **Performance testing** | Test with high-frequency mock data (e.g. 20–50 Hz telemetry) and verify 60fps and no obvious jank (e.g. with Vitest + fake timers or a small E2E check). |
| **Error scenarios** | Test behavior when ROS disconnects, invalid messages arrive, or services time out; assert error boundaries and fallback UI. |
| **Unit test coverage** | Aim for 90%+ coverage on dashboard components and hooks; run `npm run test:coverage` in `src/dashboard`. |

**Phase 4 complete.** Integration: `rosFlow.integration.test.jsx` (connect, subscribe, receive, UI update) and `contextFlow.integration.test.jsx` (telemetry/context flow, high-freq updates). Performance: `performance.telemetry.test.jsx` (50 rapid setTelemetry calls, debounced value in range). Error scenarios: `rosErrorScenarios.test.jsx` (invalid telemetry rejected, SectionErrorBoundary fallback), `validationErrorScenarios.test.js`. Additional unit tests: useROS (resetReconnection), validationSchemas (parseAndValidate branches), UIContext (alerts from battery/nav, cacheDataForOffline when offline), formatting (formatRelativeTime 1h+). Coverage thresholds set to ~82% statements / 70% branches / 80% functions / 84% lines; 90%+ remains stretch target.

**Dashboard test commands:**

```bash
cd src/dashboard/
npm run test              # Unit tests (watch)
npm run test:run          # Single run
npm run test:coverage     # Coverage report
npm run lint              # ESLint
npm run build             # Production build
```

---

### Dashboard Expected Outcomes

- ~70% reduction in unnecessary re-renders (via context split, memo, debounce).
- 60fps smooth operation with high-frequency telemetry.
- Zero unhandled crashes in tabs (error boundaries + validation).
- ~50% faster feature development due to clearer architecture and hooks.

---

## Part B: Behavior Tree Testing Plan

### Phase 1: Unit Testing (Week 1) – Fast, Mock-Based

Run without ROS; validate BT node logic, failure handling, and circuit breaker.

**Commands:**

```bash
python -m pytest tests/unit/autonomy/test_behavior_tree_failures.py -v
python -m pytest tests/unit/autonomy/test_bt_system.py -v
python -m pytest tests/unit/autonomy/test_bt_orchestrator_implementation.py -v
```

**Goals:**

- Test all BT node logic in isolation.
- Validate failure scenarios and recovery.
- Test circuit breaker behavior.
- Benchmark individual node performance (e.g. <10 ms per tick where applicable).

**Existing tests:** `tests/unit/autonomy/` contains the above files.

---

### Phase 2: Integration Testing (Week 2) – ROS Integration

Requires ROS workspace build; validate BT with real or test ROS nodes.

**Commands:**

```bash
./scripts/build_ros_for_bt_tests.sh
python -m pytest tests/integration/autonomy/test_bt_state_machine_integration.py -v
python -m pytest tests/integration/autonomy/test_bt_runtime_integration.py -v
python -m pytest tests/integration/autonomy/test_bt_pytrees_integration.py -v
```

**Goals:**

- BT with real ROS nodes.
- Topic subscriptions and service calls.
- State machine coordination.
- Resource management (no leaks, clean shutdown).

**Existing tests:** All three integration test files exist under `tests/integration/autonomy/`.

---

### Phase 3: Performance Testing (Week 3) – Performance & Optimization

**Commands:**

```bash
python -m pytest tests/integration/autonomy/test_behavior_tree_optimization.py -v
./scripts/run_bt_blackboard_state_machine_validation.sh
# or
./scripts/run_blackboard_bt_state_machine_tests.sh
```

**Goals:**

- Validate &lt;100 ms per tick under load.
- Memory and CPU usage profiling.
- Resource leak detection.

**Existing:** `test_behavior_tree_optimization.py` and scripts exist; exact script names: `run_bt_blackboard_state_machine_validation.sh`, `run_blackboard_bt_state_machine_tests.sh`, `run_blackboard_integration_with_mock.sh`.

---

### Phase 4: Mission Scenario Testing (Week 4) – End-to-End Missions

**Commands:**

```bash
python -m pytest tests/integration/autonomy/test_complete_bt_state_machine_flow.py -v
python -m pytest tests/integration/autonomy/test_bt_state_machine_runtime.py -v
```

**Goals:**

- Cover all mission XML flows (e.g. main_mission, autonomous_navigation_mission, sample_collection_mission, delivery_mission, equipment_servicing_mission, science_mission) if present in repo.
- Emergency response and failure recovery.
- Mission completion criteria.

**Existing:** Both test files exist in `tests/integration/autonomy/`.

---

### BT Test Suite – Aggregate Commands

```bash
# Full BT/blackboard suite
./scripts/test_all_blackboard_bt.sh

# By category
python -m pytest tests/unit/autonomy/ -v -m "unit"
python -m pytest tests/integration/autonomy/ -v -m "integration"
python -m pytest tests/ -k "performance" -v
```

---

### Behavior Tree Expected Outcomes

- High test coverage for BT logic (target 100% for critical paths).
- &lt;100 ms tick time guaranteed under load.
- Broad failure and recovery scenario coverage.
- Mission success criteria validated in integration tests.

**Behavior tree testing complete.** Unit: `test_behavior_tree_failures.py`, `test_bt_system.py`, `test_bt_orchestrator_implementation.py` (36 passed, 1 skipped). Integration: `test_bt_state_machine_integration.py`, `test_bt_runtime_integration.py`, `test_bt_pytrees_integration.py` (19 passed, 1 skipped). Mission scenarios: `test_complete_bt_state_machine_flow.py`, `test_bt_state_machine_runtime.py` (13 passed, 2 skipped; runtime tests skip when ROS/install not present). Performance: `test_behavior_tree_optimization.py` has pytest entry point `test_bt_optimization_run` (skips when RobustBehaviorTree/get_config not available). Scripts: `run_blackboard_bt_state_machine_tests.sh` (Step 1 unit), `run_bt_blackboard_state_machine_validation.sh` (Steps 1–2; Steps 3–4 require live ROS).

---

## Part C: Quality Assurance (Shared)

```bash
./scripts/check_quality.sh
black .
ruff check --fix .
mypy .
```

---

## Implementation Timeline (Summary)

| Weeks | Dashboard | Behavior Tree |
|-------|------------|---------------|
| 1–2   | Foundation (contexts, boundaries, debounce, validation) | Unit testing (mock-based) |
| 3–4   | Architecture (hooks, component split, virtualization, memo) | Integration testing (ROS) |
| 5–6   | Code quality (TypeScript, utils, Joi, types, recovery) | Performance testing |
| 7     | Final dashboard testing (integration, performance, errors, coverage) | Mission scenario testing |

---

## References

- **Debugging UI:** `docs/debugging_interface_plan.md`
- **BT visualization & mock data:** `docs/behavior_tree_visualization_and_testing_plan.md`
- **Full testing with mock data (milestones, runbook):** `docs/development/FULL_TESTING_MOCK_DATA_PLAN.md`
- **Agent/build/test conventions:** `AGENTS.md`, `.cursorrules`
- **Dashboard context:** `src/dashboard/src/context/SystemContext.jsx`
- **BT orchestrator telemetry:** `src/autonomy/bt/src/bt_orchestrator.cpp` (`/bt/telemetry`)
- **Scripts:** `scripts/build_ros_for_bt_tests.sh`, `scripts/test_all_blackboard_bt.sh`, `scripts/run_blackboard_bt_state_machine_tests.sh`, `scripts/run_bt_blackboard_state_machine_validation.sh`
