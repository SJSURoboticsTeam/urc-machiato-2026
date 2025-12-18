"""
Hardware Interface Package

Provides ROS2 interface to physical STM32 control systems via CAN bus.
Connects autonomy stack to deployed control systems and teleoperation infrastructure.
"""

from .hardware_interface_node import HardwareInterfaceNode

__all__ = ['HardwareInterfaceNode']
