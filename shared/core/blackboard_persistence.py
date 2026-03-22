"""
Blackboard persistence: save/load a subset of blackboard keys to/from JSON.

Uses existing /blackboard/get_value and /blackboard/set_value services.
On load, system.emergency_stop is never overwritten with false (safety).
"""

import json
import os
from pathlib import Path
from typing import Any, Dict, List, Tuple

import rclpy
from rclpy.node import Node

from autonomy_interfaces.srv import GetBlackboardValue, SetBlackboardValue
from autonomy_interfaces.srv import SaveBlackboardSnapshot, LoadBlackboardSnapshot

try:
    from core.blackboard_keys import BlackboardKeys
except ImportError:
    BlackboardKeys = None

# Keys to persist (mission progress, mode, safety scalars). Exclude high-rate data.
PERSISTED_KEYS: List[Tuple[str, str]] = [
    ("mission_active", "bool"),
    ("samples_collected", "int"),
    ("waypoints_completed", "int"),
    ("current_mission_phase", "string"),
    ("current_waypoint_index", "int"),
    ("last_error", "string"),
]
if BlackboardKeys:
    PERSISTED_KEYS.extend(
        [
            (BlackboardKeys.Auto.MODE, "string"),
            (BlackboardKeys.System.BATTERY_PERCENT, "double"),
            (BlackboardKeys.System.LAST_ERROR, "string"),
        ]
    )

DEFAULT_SNAPSHOT_PATH = "data/blackboard_snapshot.json"


def _get_default_path(node: Node) -> str:
    """Resolve default path relative to cwd or workspace."""
    base = os.environ.get("URC_DATA_DIR", ".")
    path = os.path.join(base, "blackboard_snapshot.json")
    if base == ".":
        path = os.path.join("data", "blackboard_snapshot.json")
    return path


class BlackboardPersistenceNode(Node):
    """ROS2 node that provides SaveBlackboardSnapshot and LoadBlackboardSnapshot services."""

    def __init__(self):
        super().__init__("blackboard_persistence")
        self.get_client = self.create_client(
            GetBlackboardValue, "/blackboard/get_value"
        )
        self.set_client = self.create_client(
            SetBlackboardValue, "/blackboard/set_value"
        )
        self.save_srv = self.create_service(
            SaveBlackboardSnapshot,
            "/blackboard/save_snapshot",
            self.save_snapshot_callback,
        )
        self.load_srv = self.create_service(
            LoadBlackboardSnapshot,
            "/blackboard/load_snapshot",
            self.load_snapshot_callback,
        )
        self.get_logger().info("Blackboard persistence node started")

    def _get_value(self, key: str, value_type: str) -> Any:
        req = GetBlackboardValue.Request()
        req.key = key
        req.value_type = value_type
        future = self.get_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or not future.result().success:
            return None
        raw = future.result().value
        if value_type == "bool":
            return raw.lower() in ("true", "1")
        if value_type == "int":
            try:
                return int(raw)
            except (ValueError, TypeError):
                return 0
        if value_type == "double":
            try:
                return float(raw)
            except (ValueError, TypeError):
                return 0.0
        return raw

    def _set_value(self, key: str, value: Any, value_type: str) -> bool:
        req = SetBlackboardValue.Request()
        req.key = key
        req.value_type = value_type
        if value_type == "bool":
            req.value = "true" if value else "false"
        else:
            req.value = str(value)
        future = self.set_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            return False
        return future.result().success

    def save_snapshot_callback(
        self,
        request: SaveBlackboardSnapshot.Request,
        response: SaveBlackboardSnapshot.Response,
    ):
        path = request.path.strip() or _get_default_path(self)
        if not self.get_client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "Blackboard get_value service not available"
            return response
        data: Dict[str, Any] = {}
        for key, value_type in PERSISTED_KEYS:
            val = self._get_value(key, value_type)
            if val is not None:
                data[key] = val
        try:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            response.success = True
            response.message = f"Saved {len(data)} keys to {path}"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def load_snapshot_callback(
        self,
        request: LoadBlackboardSnapshot.Request,
        response: LoadBlackboardSnapshot.Response,
    ):
        path = request.path.strip()
        if not path or not os.path.isfile(path):
            response.success = False
            response.message = f"File not found: {path}"
            return response
        if not self.set_client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "Blackboard set_value service not available"
            return response
        try:
            with open(path) as f:
                data = json.load(f)
        except Exception as e:
            response.success = False
            response.message = str(e)
            return response
        # Never overwrite system.emergency_stop with false from file (safety)
        skip_keys = {"system.emergency_stop"}
        count = 0
        type_hints: Dict[str, str] = {k: t for k, t in PERSISTED_KEYS}
        for key, value in data.items():
            if key in skip_keys:
                continue
            value_type = type_hints.get(key, "string")
            if isinstance(value, bool):
                value_type = "bool"
            elif isinstance(value, int):
                value_type = "int"
            elif isinstance(value, (float, int)):
                value_type = "double"
            if self._set_value(key, value, value_type):
                count += 1
        response.success = True
        response.message = f"Loaded {count} keys from {path}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BlackboardPersistenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
