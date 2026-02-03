"""
Path planning performance benchmarks.

Measures path plan time and replan rate for regression detection.
Can be run in CI or nightly; fail or alert on regressions.
"""

import sys
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))
nav_path = str(
    REPO / "src" / "autonomy" / "autonomy_core" / "autonomy_core" / "navigation"
)
if nav_path not in sys.path:
    sys.path.insert(0, nav_path)


# Max acceptable plan time (seconds) for a single plan_path call
MAX_PLAN_TIME_SEC = 0.5
# Min replans per second when costmap updates (replan rate)
MIN_REPLAN_RATE = 0.5


class TestPathPlanningBenchmark:
    """Benchmark path planner and D* Lite replan."""

    @pytest.fixture
    def path_planner(self):
        try:
            from autonomy.autonomy_core.autonomy_core.navigation.path_planner import (
                PathPlanner,
            )
        except ImportError:
            try:
                from autonomy_core.autonomy_core.navigation.path_planner import (
                    PathPlanner,
                )
            except ImportError:
                from path_planner import PathPlanner
        return PathPlanner()

    def test_plan_path_latency(self, path_planner):
        """Single plan_path call completes within MAX_PLAN_TIME_SEC."""
        start = (0.0, 0.0)
        goal = (10.0, 10.0)
        t0 = time.perf_counter()
        path = path_planner.plan_path(start, goal, None)
        elapsed = time.perf_counter() - t0
        assert path is not None
        assert (
            elapsed < MAX_PLAN_TIME_SEC
        ), f"plan_path took {elapsed:.3f}s (max {MAX_PLAN_TIME_SEC}s)"

    def test_replan_rate(self, path_planner):
        """D* Lite replan completes within acceptable time for dynamic obstacles."""
        try:
            from autonomy.autonomy_core.autonomy_core.navigation.path_planner import (
                DStarLite,
            )
        except ImportError:
            try:
                from autonomy_core.autonomy_core.navigation.path_planner import (
                    DStarLite,
                )
            except ImportError:
                from path_planner import DStarLite
        grid_res = 0.5
        dstar = DStarLite(grid_resolution=grid_res)
        start = (0.0, 0.0)
        goal = (5.0, 5.0)
        dstar.initialize(start, goal)
        # Build minimal costmap (empty = free)
        costmap = {}
        t0 = time.perf_counter()
        path = dstar.replan(costmap)
        elapsed = time.perf_counter() - t0
        # Replan should be fast (incremental)
        max_replan_time = 1.0 / MIN_REPLAN_RATE if MIN_REPLAN_RATE else 2.0
        assert (
            elapsed < max_replan_time
        ), f"replan took {elapsed:.3f}s (max {max_replan_time}s)"
        assert isinstance(path, list)
