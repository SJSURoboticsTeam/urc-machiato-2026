# Launch directory (legacy / alternate layout)

These Python launch files target **older ROS package names** (for example
`autonomy_navigation`, `autonomy_slam`, `hardware_interface`, `autonomy_simulation`)
that are **not** the current `services/autonomy/autonomy_core` colcon package set.

## Canonical launches (use these first)

After `colcon build` and `source install/setup.bash`, prefer package-based launches:

| Goal | Command |
|------|---------|
| Main autonomy stack | `ros2 launch autonomy_core unified.launch.py mode:=simulation` |
| Gazebo / sim world | `ros2 launch autonomy_core simulation.launch.py` |
| SLAM pipeline | `ros2 launch autonomy_core slam.launch.py` |
| Competition-oriented stack | `ros2 launch autonomy_core competition_system.launch.py` |
| L2 PointCloud2 to `/scan` | `ros2 launch autonomy_core unitree_l2_pc2_to_scan.launch.py` |
| Rosbridge + rosapi (dashboard) | `ros2 launch autonomy_core rosbridge_stack.launch.py` (default port **9090**) |
| HIL: rosbridge + serial CAN | `ros2 launch autonomy_core hil_serial_bridges.launch.py` |

See [docs/operations/deployment.rst](../../docs/operations/deployment.rst) for `deploy.sh` modes and extras (BT orchestrator, rosbridge).

## Files in this directory (legacy)

- `integrated_system.launch.py` – full stack with old package names; requires a matching workspace overlay. Rosbridge is on port **9091** here, which **does not** match the dashboard default (`ws://localhost:9090`); prefer `autonomy_core/rosbridge_stack.launch.py` for new work.
- `mission_system.launch.py` – mission + rosbridge (port **9090**); same caveat.
- `rover_simulation.launch.py` – Gazebo rover sim; verify package names against your build.

There is **no** `slam_bridge.launch.py` in this repo; older docs that referenced it are obsolete.

## Running a file from disk (not installed)

If you maintain packages that match these launches:

```bash
ros2 launch /absolute/path/to/scripts/launch/mission_system.launch.py
```

Use `ros2 launch --show-args <file>` to inspect arguments.
