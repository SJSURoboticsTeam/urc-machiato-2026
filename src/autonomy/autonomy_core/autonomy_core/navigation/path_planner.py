#!/usr/bin/env python3
"""
Path Planner - Global and local path planning for navigation.

Implements:
- A* global path planning
- Local obstacle avoidance
- Terrain-aware routing
- Multi-objective optimization

Author: URC 2026 Autonomy Team
"""

import logging
import math
from dataclasses import dataclass
from enum import Enum
from heapq import heappop, heappush
from typing import Any, Dict, List, Optional, Tuple

try:
    import networkx as nx
except ImportError:
    nx = None  # type: ignore[assignment]

logger = logging.getLogger(__name__)


class PathPlanningAlgorithm(Enum):
    """Available path planning algorithms"""

    ASTAR = "astar"
    DIJKSTRA = "dijkstra"
    RRT = "rrt"


@dataclass
class Node:
    """A* search node"""

    position: Tuple[int, int]
    g_cost: float  # Cost from start
    h_cost: float  # Heuristic cost to goal
    f_cost: float  # Total cost
    parent: Optional["Node"] = None

    @property
    def x(self) -> int:
        return self.position[0]

    @property
    def y(self) -> int:
        return self.position[1]


@dataclass
class PathSegment:
    """Path segment with properties"""

    start: Tuple[float, float]
    end: Tuple[float, float]
    length: float
    terrain_cost: float
    safety_margin: float


class PathPlanner:
    """
    Path planning system for autonomous navigation.

    Features:
    - Global path planning with A*
    - Local obstacle avoidance
    - Terrain cost integration
    - Multi-objective optimization
    """

    def __init__(self):
        self.algorithm = PathPlanningAlgorithm.ASTAR
        self.grid_resolution = 0.5  # meters
        self.max_search_iterations = 10000
        self.obstacle_inflation_radius = 1.0  # meters

        # Cost weights
        self.distance_weight = 1.0
        self.terrain_weight = 2.0
        self.safety_weight = 1.5

        # Cost/occupancy from obstacles (grid cell -> cost; 0 = free, inf = obstacle)
        self._costmap: Dict[Tuple[int, int], float] = {}
        self._grid_resolution = 0.5  # meters per cell

    def _world_to_grid(self, world_pos: Tuple[float, float]) -> Tuple[int, int]:
        """World (x, y) to grid cell indices."""
        return (
            int(world_pos[0] / self._grid_resolution),
            int(world_pos[1] / self._grid_resolution),
        )

    def initialize(self):
        """Initialize path planner"""
        # TODO: Load map data
        # Initialize cost maps
        # Set up algorithm parameters

    def plan_path(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        constraints: Optional[dict] = None,
    ) -> List[Tuple[float, float]]:
        """
        Plan path from start to goal

        Args:
            start: Starting position (x, y)
            goal: Goal position (x, y)
            constraints: Optional planning constraints

        Returns:
            List of waypoints forming the path
        """
        if self.algorithm == PathPlanningAlgorithm.ASTAR:
            return self._plan_astar(start, goal, constraints)
        elif self.algorithm == PathPlanningAlgorithm.DIJKSTRA:
            return self._plan_dijkstra(start, goal, constraints)
        else:
            # TODO: Implement other algorithms
            return []

    def _plan_astar(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        constraints: Optional[dict] = None,
    ) -> List[Tuple[float, float]]:
        """A* path planning implementation using NetworkX"""
        if nx is None:
            logger.warning("NetworkX not available, falling back to simple path")
            return self._simple_path(start, goal)

        # Create grid-based graph for path planning
        graph = self._create_navigation_graph(start, goal, constraints)

        if graph is None or len(graph.nodes) == 0:
            return self._simple_path(start, goal)

        # Convert coordinates to node IDs
        start_node = self._coord_to_node(start)
        goal_node = self._coord_to_node(goal)

        if start_node not in graph or goal_node not in graph:
            return self._simple_path(start, goal)

        try:
            # Use NetworkX A* implementation
            path = nx.astar_path(
                graph,
                start_node,
                goal_node,
                heuristic=self._euclidean_heuristic,
                weight="weight",
            )

            # Convert back to coordinates
            return [self._node_to_coord(node) for node in path]

        except (nx.NetworkXNoPath, nx.NodeNotFound):
            logger.warning(f"No path found from {start} to {goal}")
            return self._simple_path(start, goal)

    def _plan_dijkstra(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        constraints: Optional[dict] = None,
    ) -> List[Tuple[float, float]]:
        """Dijkstra path planning implementation using NetworkX"""
        if nx is None:
            logger.warning("NetworkX not available, falling back to simple path")
            return self._simple_path(start, goal)

        # Create grid-based graph for path planning
        graph = self._create_navigation_graph(start, goal, constraints)

        if graph is None or len(graph.nodes) == 0:
            return self._simple_path(start, goal)

        # Convert coordinates to node IDs
        start_node = self._coord_to_node(start)
        goal_node = self._coord_to_node(goal)

        if start_node not in graph or goal_node not in graph:
            return self._simple_path(start, goal)

        try:
            # Use NetworkX Dijkstra implementation
            path = nx.dijkstra_path(graph, start_node, goal_node, weight="weight")

            # Convert back to coordinates
            return [self._node_to_coord(node) for node in path]

        except (nx.NetworkXNoPath, nx.NodeNotFound):
            logger.warning(f"No path found from {start} to {goal}")
            return self._simple_path(start, goal)

    def get_terrain_cost(self, position: Tuple[float, float]) -> float:
        """Get terrain cost at position from costmap (1.0 free, higher near obstacles)."""
        cell = self._world_to_grid(position)
        c = self._costmap.get(cell, 0.0)
        if c >= 1e9:  # obstacle
            return float("inf")
        return 1.0 + c * 0.1  # scale for weight

    def is_position_valid(self, position: Tuple[float, float]) -> bool:
        """Check if position is valid (no obstacles within inflation radius)."""
        cell = self._world_to_grid(position)
        # Check this cell and neighbors within inflation radius (in grid cells)
        r = max(1, int(self.obstacle_inflation_radius / self._grid_resolution))
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                c = (cell[0] + dx, cell[1] + dy)
                if self._costmap.get(c, 0.0) >= 1e9:
                    return False
        return True

    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Smooth path to reduce sharp turns using SciPy B-splines"""
        if len(path) < 3:
            return path

        try:
            import numpy as np
            from scipy.interpolate import splprep, splev
        except ImportError:
            logger.warning("SciPy not available, skipping path smoothing")
            return path

        try:
            # Convert path to numpy array
            points = np.array(path)

            # B-spline smoothing with safety margin consideration
            smoothing_factor = min(0.1, len(path) * 0.01)  # Adaptive smoothing

            # Interpolate x and y coordinates separately
            tck, u = splprep([points[:, 0], points[:, 1]], s=smoothing_factor)

            # Generate more points for smoother path
            u_new = np.linspace(u.min(), u.max(), max(50, len(path) * 2))
            x_smooth, y_smooth = splev(u_new, tck)

            # Convert back to list of tuples
            smoothed_path = list(zip(x_smooth, y_smooth))

            # Ensure start and end points are preserved
            smoothed_path[0] = path[0]
            smoothed_path[-1] = path[-1]

            logger.debug(
                f"Smoothed path from {len(path)} to {len(smoothed_path)} points"
            )
            return smoothed_path

        except Exception as e:
            logger.warning(f"Path smoothing failed: {e}, returning original path")
            return path

    def optimize_path(
        self, path: List[Tuple[float, float]], criteria: Optional[List[str]] = None
    ) -> List[Tuple[float, float]]:
        """Optimize path based on multiple criteria"""
        if not criteria:
            criteria = ["distance", "terrain", "safety"]

        # TODO: Implement multi-objective optimization
        # - Distance minimization
        # - Terrain cost reduction
        # - Safety margin maximization
        return path  # Placeholder

    def get_path_length(self, path: List[Tuple[float, float]]) -> float:
        """Calculate total path length"""
        if len(path) < 2:
            return 0.0

        total_length = 0.0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            total_length += math.sqrt(dx * dx + dy * dy)

        return total_length

    def estimate_traversal_time(
        self,
        path: List[Tuple[float, float]],
        speed_profile: Optional[Dict[str, Any]] = None,
    ) -> float:
        """Estimate time to traverse path"""
        # TODO: Implement time estimation
        # - Consider terrain types
        # - Account for speed limits
        # - Factor in acceleration/deceleration
        length = self.get_path_length(path)
        avg_speed = speed_profile.get("average", 1.0) if speed_profile else 1.0
        return length / avg_speed

    def update_costmap(self, new_obstacles: List[Tuple[float, float, float]]) -> None:
        """Update costmap with obstacle positions (x, y, radius in meters). Inflate by obstacle_inflation_radius. Replaces previous obstacle-derived costs."""
        self._costmap.clear()
        for x, y, radius in new_obstacles:
            r_total = radius + self.obstacle_inflation_radius
            n_cells = max(1, int(r_total / self._grid_resolution))
            cx, cy = self._world_to_grid((x, y))
            for dx in range(-n_cells, n_cells + 1):
                for dy in range(-n_cells, n_cells + 1):
                    gx, gy = cx + dx, cy + dy
                    wx = gx * self._grid_resolution
                    wy = gy * self._grid_resolution
                    dist = math.sqrt((wx - x) ** 2 + (wy - y) ** 2)
                    if dist <= r_total:
                        self._costmap[(gx, gy)] = float("inf")

    def get_costmap_for_replan(self) -> Dict[Tuple[int, int], float]:
        """Return costmap as grid cell -> cost for D* Lite replanning."""
        return dict(self._costmap)

    def reset_planner(self):
        """Reset planner state and costmap."""
        self._costmap.clear()

    def shutdown(self):
        """Shutdown path planner"""
        # TODO: Clean shutdown
        # Save state if needed
        # Release resources

    def _create_navigation_graph(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        constraints: Optional[dict] = None,
    ) -> Optional[object]:
        """Create NetworkX graph for path planning."""
        if nx is None:
            return None
        graph = nx.Graph()
        margin = 10.0
        min_x = min(start[0], goal[0]) - margin
        max_x = max(start[0], goal[0]) + margin
        min_y = min(start[1], goal[1]) - margin
        max_y = max(start[1], goal[1]) + margin
        resolution = 1.0
        nodes = []
        x = min_x
        while x <= max_x:
            y = min_y
            while y <= max_y:
                if self.is_position_valid((x, y)):
                    nodes.append((x, y))
                y += resolution
            x += resolution
        for node in nodes:
            node_id = self._coord_to_node(node)
            graph.add_node(node_id, pos=node)
        for node in nodes:
            node_id = self._coord_to_node(node)
            for dx, dy in [
                (-1, 0),
                (1, 0),
                (0, -1),
                (0, 1),
                (-1, -1),
                (-1, 1),
                (1, -1),
                (1, 1),
            ]:
                neighbor = (node[0] + dx * resolution, node[1] + dy * resolution)
                if neighbor in nodes:
                    neighbor_id = self._coord_to_node(neighbor)
                    distance = ((dx * resolution) ** 2 + (dy * resolution) ** 2) ** 0.5
                    terrain_cost = (
                        self.get_terrain_cost(node) + self.get_terrain_cost(neighbor)
                    ) / 2.0
                    weight = distance * terrain_cost
                    graph.add_edge(node_id, neighbor_id, weight=weight)
        return graph

    def _coord_to_node(self, coord: Tuple[float, float]) -> str:
        """Convert coordinate to node ID."""
        return f"{coord[0]:.1f},{coord[1]:.1f}"

    def _node_to_coord(self, node_id: str) -> Tuple[float, float]:
        """Convert node ID to coordinate."""
        x, y = node_id.split(",")
        return (float(x), float(y))

    def _euclidean_heuristic(self, node1: str, node2: str) -> float:
        """Euclidean distance heuristic for A*."""
        coord1 = self._node_to_coord(node1)
        coord2 = self._node_to_coord(node2)
        return ((coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2) ** 0.5

    def _simple_path(
        self, start: Tuple[float, float], goal: Tuple[float, float]
    ) -> List[Tuple[float, float]]:
        """Fallback simple straight-line path."""
        return [start, goal]


# =============================================================================
# D* Lite Implementation for Dynamic Path Planning
# =============================================================================


@dataclass
class DStarNode:
    """Node for D* Lite priority queue"""

    position: Tuple[int, int]
    key: Tuple[float, float]  # (k1, k2) priority values

    def __lt__(self, other):
        return self.key < other.key


class DStarLite:
    """
    D* Lite algorithm for efficient replanning in dynamic environments.

    Key features:
    - Incremental replanning when map changes
    - Reverse search from goal to start
    - Maintains optimality while being computationally efficient
    """

    def __init__(self, grid_resolution: float = 0.5):
        # Priority queue for node updates
        self.U: List[DStarNode] = []

        # Cost estimates
        self.g: Dict[Tuple[int, int], float] = {}  # Current best cost from goal
        self.rhs: Dict[Tuple[int, int], float] = {}  # One-step lookahead cost

        # Grid parameters
        self.grid_resolution = grid_resolution
        self.obstacle_cost = float("inf")

        # Last start/goal for incremental updates
        self.s_start: Optional[Tuple[int, int]] = None
        self.s_goal: Optional[Tuple[int, int]] = None
        self.s_last: Optional[Tuple[int, int]] = None

        # Costmap (will be updated from SLAM)
        self.costmap: Dict[Tuple[int, int], float] = {}
        self._previous_costmap: Dict[Tuple[int, int], float] = {}
        self._changed_cells: List[Tuple[int, int]] = []

        # Movement cost (diagonal moves allowed)
        self.sqrt2 = math.sqrt(2)

    def initialize(self, start: Tuple[float, float], goal: Tuple[float, float]):
        """Initialize D* Lite for new planning task"""
        # Convert to grid coordinates
        self.s_start = self._world_to_grid(start)
        self.s_goal = self._world_to_grid(goal)
        self.s_last = self.s_start

        # Clear data structures
        self.U.clear()
        self.g.clear()
        self.rhs.clear()

        # Initialize goal
        self.rhs[self.s_goal] = 0
        self.g[self.s_goal] = float("inf")

        # Insert goal into priority queue
        goal_node = DStarNode(self.s_goal, self._calculate_key(self.s_goal))
        heappush(self.U, goal_node)

        self._compute_shortest_path()

    def update_start(self, start: Tuple[float, float]) -> None:
        """Update current start position (e.g. after robot moved) for next replan."""
        self.s_last = self.s_start
        self.s_start = self._world_to_grid(start)

    def replan(
        self, new_costmap: Dict[Tuple[int, int], float]
    ) -> List[Tuple[float, float]]:
        """
        Incremental replanning when costmap changes

        Args:
            new_costmap: Updated costmap from SLAM

        Returns:
            Updated path from current position to goal
        """
        self._previous_costmap = dict(self.costmap)
        self.costmap = dict(new_costmap)

        if self._costmap_changed():
            self._update_changed_vertices()
            try:
                self._compute_shortest_path()
            except (KeyError, IndexError):
                pass  # No path or invalid state

        return self._extract_path()

    def _costmap_changed(self) -> bool:
        """Check if costmap has significant changes requiring replan."""
        all_cells = set(self._previous_costmap.keys()) | set(self.costmap.keys())
        self._changed_cells = [
            c
            for c in all_cells
            if self._previous_costmap.get(c, 0.0) != self.costmap.get(c, 0.0)
        ]
        return len(self._changed_cells) > 0

    def _update_changed_vertices(self) -> None:
        """Update vertices affected by costmap changes (rhs and queue)."""
        for cell in self._changed_cells:
            self._update_vertex(cell)
            for neighbor in self._get_neighbors(cell):
                self._update_vertex(neighbor)

    def _compute_shortest_path(self):
        """Main D* Lite algorithm - compute shortest path."""
        while (
            self.U
            and self.s_start is not None
            and self.s_goal is not None
            and (
                self.U[0].key < self._calculate_key(self.s_start)
                or self.rhs.get(self.s_start, float("inf"))
                != self.g.get(self.s_start, float("inf"))
            )
        ):

            # Get node with lowest priority
            u = heappop(self.U)
            k_old = u.key
            k_new = self._calculate_key(u.position)

            if k_old < k_new:
                # Node priority increased, reinsert
                u.key = k_new
                heappush(self.U, u)
            elif self.g.get(u.position, float("inf")) > self.rhs.get(
                u.position, float("inf")
            ):
                # Locally overconsistent (rhs is better)
                self.g[u.position] = self.rhs[u.position]
                # Update all neighbors
                for neighbor in self._get_neighbors(u.position):
                    self._update_vertex(neighbor)
            else:
                # Locally underconsistent (g is better)
                self.g[u.position] = float("inf")
                # Update current node and all neighbors
                self._update_vertex(u.position)
                for neighbor in self._get_neighbors(u.position):
                    self._update_vertex(neighbor)

    def _update_vertex(self, u: Tuple[int, int]):
        """Update vertex in D* Lite algorithm"""
        if u != self.s_goal:
            # Calculate minimum cost through neighbors
            min_cost = float("inf")
            for neighbor in self._get_neighbors(u):
                cost = self.g.get(neighbor, float("inf")) + self._cost(u, neighbor)
                if cost < min_cost:
                    min_cost = cost
            self.rhs[u] = min_cost

        # Remove from priority queue if present
        self.U = [node for node in self.U if node.position != u]

        # Add to queue if locally inconsistent
        if self.g.get(u, float("inf")) != self.rhs.get(u, float("inf")):
            node = DStarNode(u, self._calculate_key(u))
            heappush(self.U, node)

    def _calculate_key(self, s: Tuple[int, int]) -> Tuple[float, float]:
        """Calculate priority key for node"""
        g_rhs_min = min(self.g.get(s, float("inf")), self.rhs.get(s, float("inf")))

        # Calculate heuristic from start to this node
        h_start = self._heuristic(self.s_start, s)

        return (
            g_rhs_min + h_start + self._heuristic(self.s_last, self.s_start),
            g_rhs_min,
        )

    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Euclidean distance heuristic"""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return math.sqrt(dx * dx + dy * dy) * self.grid_resolution

    def _cost(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Cost of moving between adjacent cells"""
        # Check if either cell is an obstacle
        if self.costmap.get(a, 0) >= self.obstacle_cost:
            return float("inf")
        if self.costmap.get(b, 0) >= self.obstacle_cost:
            return float("inf")

        # Movement cost (diagonal moves cost more)
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])

        if dx == 1 and dy == 1:
            return self.sqrt2 * self.grid_resolution  # Diagonal
        else:
            return self.grid_resolution  # Cardinal

    def _get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get 8-connected neighbors"""
        x, y = pos
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = (x + dx, y + dy)
                neighbors.append(neighbor)
        return neighbors

    def _world_to_grid(self, world_pos: Tuple[float, float]) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        x = int(world_pos[0] / self.grid_resolution)
        y = int(world_pos[1] / self.grid_resolution)
        return (x, y)

    def _grid_to_world(self, grid_pos: Tuple[int, int]) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates"""
        x = grid_pos[0] * self.grid_resolution
        y = grid_pos[1] * self.grid_resolution
        return (x, y)

    def _extract_path(self) -> List[Tuple[float, float]]:
        """Extract path from current position to goal"""
        if self.g.get(self.s_start, float("inf")) == float("inf"):
            return []  # No path found

        path = []
        current = self.s_start

        while current != self.s_goal:
            path.append(self._grid_to_world(current))

            # Find best neighbor
            best_neighbor = None
            best_cost = float("inf")

            for neighbor in self._get_neighbors(current):
                neighbor_cost = self.g.get(neighbor, float("inf")) + self._cost(
                    current, neighbor
                )
                if neighbor_cost < best_cost:
                    best_cost = neighbor_cost
                    best_neighbor = neighbor

            if best_neighbor is None:
                break  # No valid path

            current = best_neighbor

        path.append(self._grid_to_world(self.s_goal))
        return path
