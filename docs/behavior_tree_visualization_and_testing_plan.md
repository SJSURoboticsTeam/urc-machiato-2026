# Behavior Tree Visualization and Testing Plan

Comprehensive plan for BT visualization in the debugging dashboard, mock data strategy, and automated testing. Aligns with existing components (`BTExecutionView`, `BTPerformanceChart`, `BTBlackboardLink`) and backend telemetry (`/bt/telemetry`).

---

## 1. Current State

### 1.1 Dashboard Components (existing)

| Component | Purpose | Data shape |
|-----------|---------|------------|
| `BTExecutionView` | Tree of nodes with status and duration | `nodes[]` (tree: `id`, `name`, `status`, `durationMs`, `children[]`) |
| `BTPerformanceChart` | Bar chart of node duration (bottlenecks) | `nodeTimings[]` (`id`, `name`, `durationMs`, `count`) |
| `BTBlackboardLink` | Node-to-blackboard read/write mapping | `nodeKeys[]` (`nodeId`, `reads[]`, `writes[]`) |

### 1.2 Backend Telemetry

- **Topic:** `/bt/telemetry` (BT orchestrator C++; not `/debugging/bt_performance`).
- **Message:** `std_msgs/msg/String` with JSON.
- **Content today:** Root-level only: `tree_status`, `mission_progress`, `robot_state`, `navigation_state`, `safety_state`, `system_health`. No per-node tree or per-node timing.

### 1.3 Gaps

- No per-node execution tree or per-node timing from ROS.
- Mock data is inline in `DebuggingDashboard.jsx`; no shared mock factory or scenarios.
- No Vitest tests for BT components.
- No clear contract for a future `/debugging/bt_performance` (or extended `/bt/telemetry`) payload.

---

## 2. Data Model

### 2.1 Execution Tree (for BTExecutionView)

```js
// Node: recursive tree
{
  id: string,           // unique (e.g. "root", "Navigate", "CheckBattery")
  name: string,        // display (e.g. "NavigateToPose")
  status: 'idle' | 'running' | 'success' | 'failure',
  durationMs?: number, // last or current run
  children?: Node[]    // ordered child nodes
}
```

- **Root:** Single root or array of top-level nodes (current code: array of roots).
- **Status:** `idle` (not run this tick), `running` (current), `success`, `failure`.

### 2.2 Node Timings / Performance (for BTPerformanceChart)

```js
{
  id: string,
  name: string,
  durationMs: number,
  count: number   // number of executions in window (e.g. last N ticks)
}
```

### 2.3 Node–Blackboard Links (for BTBlackboardLink)

```js
{
  nodeId: string,
  reads: string[],   // blackboard keys read
  writes: string[]  // blackboard keys written
}
```

### 2.4 Future ROS Payload (target)

If backend adds per-node data (e.g. `/debugging/bt_performance` or extended `/bt/telemetry`), aim for:

```json
{
  "timestamp": 1234567890.123,
  "event_type": "tick",
  "tree_status": "running",
  "nodes": [
    { "id": "root", "name": "Root", "status": "running", "duration_ms": 0, "children": ["Sequence"] },
    { "id": "Navigate", "name": "NavigateToPose", "status": "success", "duration_ms": 85 }
  ],
  "node_timings": [
    { "id": "Navigate", "duration_ms": 85, "count": 1 }
  ],
  "node_blackboard": [
    { "node_id": "Navigate", "reads": ["robot_x", "robot_y"], "writes": ["navigation_status"] }
  ]
}
```

---

## 3. Mock Data Strategy

### 3.1 Location

- **File:** `src/dashboard/src/components/debugging/mocks/btMockData.js` (or `data/btMockData.js` under `debugging`).
- **Exports:** Tree factories, timing factories, link definitions, and predefined scenarios.

### 3.2 Tree Factory

```js
// buildTreeNode({ id, name, status, durationMs, children })
// buildTreeFromFlat(flatList)  // optional: build hierarchy from flat list with parentId
```

- Defaults: `status: 'success'`, `durationMs: 0`, `children: []`.
- Use to build small (3–5 node) and large (10+ node) trees for layout and scrolling tests.

### 3.3 Scenarios (predefined mock data)

| Scenario | Purpose |
|----------|---------|
| **All success** | Happy path; all nodes success, short durations. |
| **One failure** | Single node failure (e.g. Navigate); rest success; test status colors and layout. |
| **Bottleneck** | One node with high `durationMs` (e.g. 150 ms); test performance chart ordering and tooltip. |
| **Deep tree** | 3–4 levels, many children; test expand/collapse and scroll. |
| **Running state** | One node `status: 'running'`, others success/failure; test “current” highlight. |

### 3.4 Node–Blackboard Links Mock

- **Static map** per known BT (e.g. from BT XML or design doc): list of `{ nodeId, reads, writes }`.
- Dashboard and tests import this; when backend provides it, replace with ROS-driven data.

### 3.5 Usage in Dashboard

- **DebuggingDashboard:** Import scenarios (e.g. `MOCK_BT_SCENARIOS.allSuccess`) or use factory in a `useMemo`/`useState` and pass to `BTExecutionView`, `BTPerformanceChart`, `BTBlackboardLink`.
- **Storybook (optional):** One story per scenario for visual QA.

---

## 4. Visualization Enhancements (roadmap)

### 4.1 Already in place

- Tree with depth indent, status color (success/failure), duration.
- Bar chart for duration (bottlenecks), truncated to 15 nodes.
- Node–blackboard list (reads/writes).

### 4.2 Short-term (high value)

| Feature | Description |
|---------|-------------|
| **Running state** | Distinct style for `status === 'running'` (e.g. pulse or “Running” label). |
| **Expand/collapse** | Per-node or “collapse all below depth N” to reduce clutter on deep trees. |
| **Selected node detail** | When a node is selected, show a small panel: name, id, last status, last duration, reads/writes. |
| **Empty/loading** | Explicit “No data” and “Loading…” states (already partially there). |

### 4.3 Medium-term

| Feature | Description |
|---------|-------------|
| **Time window** | Filter or aggregate node timings over last N seconds (if backend sends history or tick stream). |
| **Success/failure rate** | Per node: e.g. “8/10 success” from last N ticks (requires backend or replay). |
| **Tooltips** | On hover: full name, id, duration, status. |

### 4.4 Long-term (backend-dependent)

- Execution path history (which branches were taken).
- Heat map of failure conditions (e.g. by battery level or sensor).
- Live subscription to `/bt/telemetry` or `/debugging/bt_performance` and mapping of JSON into the above data model.

---

## 5. Testing Plan

### 5.1 Unit Tests (Vitest) – BT components

| Component | Test file | What to test |
|------------|-----------|----------------|
| **BTExecutionView** | `BehaviorTreeAnalytics/__tests__/BTExecutionView.test.jsx` | Renders empty state; renders tree (root + children); status colors (success/failure/running); duration shown; `onSelectNode` called with correct id when node clicked; selected node has distinct class; deep tree (e.g. 2 levels) renders without error. |
| **BTPerformanceChart** | `BehaviorTreeAnalytics/__tests__/BTPerformanceChart.test.jsx` | Renders empty state; renders bars from `nodeTimings`; sorts by duration (desc); truncates to 15; tooltip/formatter (if testable); handles missing `durationMs`/`count` (default 0/1). |
| **BTBlackboardLink** | `BehaviorTreeAnalytics/__tests__/BTBlackboardLink.test.jsx` | Renders empty state; renders list of nodeId + reads + writes; handles empty reads/writes; multiple nodes. |

### 5.2 Mock data in tests

- **Source:** Same `btMockData.js` (or a test-only subset) so UI and tests share one contract.
- **Pattern:** Import `buildTreeNode`, `MOCK_BT_SCENARIOS`, `MOCK_NODE_LINKS` in tests; pass into components; assert on visible text, roles, and (if needed) structure.

### 5.3 Integration-style test (optional)

- **DebuggingDashboard** (already tested): “Behavior Tree” tab shows execution tree and “Node duration” (and optionally “Node – Blackboard links”) using mock data. No new test file strictly required if tab + content are covered.

### 5.4 Snapshot testing (optional)

- One snapshot per scenario for `BTExecutionView` to catch unintended layout/class changes. Use sparingly; prefer behavior assertions.

---

## 6. Implementation Order

1. **Mock data module** – Add `debugging/mocks/btMockData.js`: tree factory, `nodeTimings` factory, static `nodeKeys`, and 3–5 scenarios (all success, one failure, bottleneck, deep, running).
2. **Wire dashboard to mocks** – In `DebuggingDashboard.jsx`, replace inline `MOCK_BT_NODES` / `MOCK_NODE_TIMINGS` with imports from `btMockData.js` (e.g. scenario selector or default scenario).
3. **BT component tests** – Add `BTExecutionView.test.jsx`, `BTPerformanceChart.test.jsx`, `BTBlackboardLink.test.jsx` using mock data; run with `npm run test:run -- src/components/debugging/BehaviorTreeAnalytics`.
4. **Visualization tweaks** – Add “running” state style and (if needed) expand/collapse or selected-node detail panel.
5. **Backend alignment** – Document desired JSON for `/debugging/bt_performance` or extended `/bt/telemetry`; when backend supports it, add a subscriber in the dashboard and map payload to the same data model used by mocks.

---

## 7. File Layout (proposed)

```
src/dashboard/src/components/debugging/
  mocks/
    btMockData.js       # tree + timings + nodeKeys + scenarios
  BehaviorTreeAnalytics/
    BTExecutionView.jsx
    BTPerformanceChart.jsx
    BTBlackboardLink.jsx
    __tests__/
      BTExecutionView.test.jsx
      BTPerformanceChart.test.jsx
      BTBlackboardLink.test.jsx
```

---

## 8. Success Criteria

- All BT components render correctly with mock data in the Debug tab.
- At least three scenarios (all success, one failure, bottleneck) available from a single mock module.
- Vitest tests for `BTExecutionView`, `BTPerformanceChart`, and `BTBlackboardLink` pass using shared mock data.
- Data model and target ROS JSON documented so backend can add per-node telemetry without changing dashboard contracts.

---

## 9. References

- Existing plan: `docs/debugging_interface_plan.md` (Phase 3).
- Backend telemetry: `src/autonomy/bt/src/bt_orchestrator.cpp` (`publish_bt_telemetry`, `/bt/telemetry`).
- Dashboard config: `src/dashboard/src/config/rosTopics.js` (`DEBUGGING_TOPICS.BT_PERFORMANCE`).
