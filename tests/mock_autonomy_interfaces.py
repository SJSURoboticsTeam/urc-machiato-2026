#!/usr/bin/env python3
"""
Mock autonomy_interfaces module for integration testing.

Provides mock ROS2 message and service types when the real autonomy_interfaces
package is not available.
"""

from geometry_msgs.msg import Point, PoseStamped, Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import String as SystemState
from std_srvs.srv import SetBool as ChangeState
from std_srvs.srv import Trigger


# Mock message types
class FollowMeStatus:
    def __init__(self):
        self.status = ""
        self.target_distance = 0.0
        self.target_angle = 0.0

class ArucoDetection:
    def __init__(self):
        self.marker_id = 0
        self.pose = PoseStamped()
        self.confidence = 0.0

class TypingStatus:
    def __init__(self):
        self.is_typing = False
        self.current_letter = ""
        self.progress = 0.0

# Mock service types
class DetectAruco:
    class Request:
        def __init__(self):
            self.image = Image()
            self.marker_id = 0

    class Response:
        def __init__(self):
            self.detections = []
            self.success = False

class StartTyping:
    class Request:
        def __init__(self):
            self.text = ""

    class Response:
        def __init__(self):
            self.success = False
            self.message = ""

# Create the module structure
import sys
from types import ModuleType

# Create mock submodules
msg = ModuleType('msg')
srv = ModuleType('srv')
action = ModuleType('action')

# Add classes to submodules
msg.SystemState = SystemState
msg.ChangeState = ChangeState
msg.FollowMeStatus = FollowMeStatus
msg.ArucoDetection = ArucoDetection
msg.TypingStatus = TypingStatus

srv.ChangeState = ChangeState
srv.DetectAruco = DetectAruco
srv.StartTyping = StartTyping

# Mock action types
class ExecuteMission:
    class Goal:
        def __init__(self):
            self.mission_type = ""
            self.waypoints = []

    class Result:
        def __init__(self):
            self.success = False
            self.message = ""

    class Feedback:
        def __init__(self):
            self.progress = 0.0
            self.current_waypoint = 0

action.ExecuteMission = ExecuteMission

# Create the main module
autonomy_interfaces = ModuleType('autonomy_interfaces')
autonomy_interfaces.msg = msg
autonomy_interfaces.srv = srv
autonomy_interfaces.action = action

# Add to sys.modules so imports work
sys.modules['autonomy_interfaces'] = autonomy_interfaces
sys.modules['autonomy_interfaces.msg'] = msg
sys.modules['autonomy_interfaces.srv'] = srv
sys.modules['autonomy_interfaces.action'] = action
