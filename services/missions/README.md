# Missions Service

The missions service implements the high-level behavior trees for the URC challenges (Delivery, Science, Autonomy, etc.).

## How to Run

Primary autonomy stack (colcon, after ``source install/setup.bash``):

```bash
ros2 launch autonomy_core unified.launch.py mode:=simulation
```

Legacy mission + rosbridge layout (requires matching ROS packages from that stack):

```bash
ros2 launch /path/to/repo/scripts/launch/mission_system.launch.py
```

## How to Test
```bash
python -m pytest tests/unit/missions/
```