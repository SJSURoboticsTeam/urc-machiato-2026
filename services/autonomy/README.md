# Autonomy Service

The autonomy service contains the core ROS2 packages responsible for perception, navigation, and control of the Mars rover.

## How to Run
```bash
colcon build --packages-up-to autonomy_core
source install/setup.bash
ros2 launch autonomy_core unified.launch.py
```

## How to Test
```bash
python -m pytest tests/unit/autonomy/
```
