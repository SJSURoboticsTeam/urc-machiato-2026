#!/usr/bin/env python3
"""
Adaptive State Machine - Runtime ROS2 State Machine.

This is the PRODUCTION runtime state machine for the URC 2026 rover.

Architecture:
- Runtime: Uses ROS2 LifecycleNode for production state management
- Service: /adaptive_state_machine/get_state (queried by BT orchestrator)
- Topic: /adaptive_state_machine/state (publishes state changes)
- Commands: /adaptive_state_machine/commands (receives JSON commands)

States: BOOT -> IDLE -> {AUTONOMOUS, TELEOPERATION} -> EMERGENCY_STOP/ERROR

Do NOT confuse with StateManager in state_management.py:
- StateManager = Dashboard/CLI state tracking (legacy/development)
- AdaptiveStateMachine = Production runtime state (ROS2)

Integration:
- BT orchestrator (C++) queries this node via service calls
- Dashboard should subscribe to /adaptive_state_machine/state for runtime status

Classes
-------
SystemState : Enum
    Enumeration of system states (boot, idle, autonomous, teleoperation, etc.).
AutonomousSubstate : Enum
    Enumeration of autonomous operation substates.
TeleoperationSubstate : Enum
    Enumeration of teleoperation substates.
AdaptiveStateMachine : LifecycleNode
    Main state machine class providing ROS2 service interface.

Author
------
URC 2026 State Machine Team
"""

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from enum import Enum
import json
from typing import Dict, Any, Optional
import time
import std_msgs.msg

# Import autonomy interfaces
try:
    import autonomy_interfaces
    from autonomy_interfaces.srv import GetSystemState
    from autonomy_interfaces.msg import SystemState as SystemStateMsg

    AUTONOMY_INTERFACES_AVAILABLE = True
except ImportError as e:
    AUTONOMY_INTERFACES_AVAILABLE = False
    GetSystemState = None
    SystemStateMsg = None
    print(f"Warning: autonomy_interfaces not available: {e}")


class SystemState(Enum):
    """System states matching URC requirements."""

    BOOT = "boot"
    IDLE = "idle"
    AUTONOMOUS = "autonomous"
    TELEOPERATION = "teleoperation"
    EMERGENCY_STOP = "emergency_stop"
    ERROR = "error"


class AutonomousSubstate(Enum):
    """Autonomous operation substates."""

    NONE = "none"
    WAYPOINT_NAVIGATION = "waypoint_navigation"
    SAMPLE_COLLECTION = "sample_collection"
    DELIVERY = "delivery"
    FOLLOW_ME = "follow_me"
    RETURN_TO_OPERATOR = "return_to_operator"


class TeleoperationSubstate(Enum):
    """Teleoperation substates."""

    NONE = "none"
    MANUAL_CONTROL = "manual_control"
    SEMI_AUTONOMOUS = "semi_autonomous"


class AdaptiveStateMachine(LifecycleNode):
    """
    Adaptive State Machine for URC 2026 rover.

    Provides ROS2 service interface for BT orchestrator to query system state
    and manages state transitions based on system conditions and commands.

    Attributes
    ----------
    current_state : SystemState
        Current system state (boot, idle, autonomous, teleoperation, etc.).
    current_substate : AutonomousSubstate
        Current autonomous operation substate.
    current_teleop_substate : TeleoperationSubstate
        Current teleoperation substate.
    state_metadata : dict
        Additional metadata associated with current state.
    last_transition_time : float
        Timestamp of last state transition.
    state_publisher : rclpy.publisher.Publisher
        Publisher for state change notifications.
    state_service : rclpy.service.Service
        Service server for state queries from BT orchestrator.
    """

    def __init__(self):
        super().__init__("adaptive_state_machine")

        # State management
        self.current_state = SystemState.BOOT
        self.current_substate = AutonomousSubstate.NONE
        self.current_teleop_substate = TeleoperationSubstate.NONE
        self.state_metadata = {}
        self.last_transition_time = time.time()

        # Hybrid control: mode request and flags (cleared after handling)
        self.mode_request: Optional[SystemState] = None
        self.human_override_active = False
        self.auto_assist_enabled = True
        self._battery_low_threshold = (
            15.0  # percent; do not switch to autonomous below this
        )

        # Optional blackboard client for syncing state and safety checks
        self._blackboard = None

        # Service clients for system monitoring
        self.system_monitor_client = None
        self.navigation_status_client = None

        # Publishers for state changes
        if AUTONOMY_INTERFACES_AVAILABLE and SystemStateMsg is not None:
            self.state_publisher = self.create_publisher(
                SystemStateMsg, "/adaptive_state_machine/state", 10
            )
        else:
            # Fallback to String if interfaces not available
            from std_msgs.msg import String

            self.state_publisher = self.create_publisher(
                String, "/adaptive_state_machine/state", 10
            )
            self.get_logger().warn(
                "Using String message type as fallback for state publishing"
            )

        self.get_logger().info("Adaptive State Machine initialized")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Configure the state machine.

        Sets up ROS2 services, subscriptions, and clients for state management.
        Called during lifecycle node configuration phase.

        Parameters
        ----------
        state : LifecycleState
            Current lifecycle state of the node.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS if configuration succeeded, FAILURE otherwise.

        Raises
        ------
        Exception
            If service or subscription creation fails.
        """
        self.get_logger().info("Configuring Adaptive State Machine...")

        try:
            # Create service server
            if AUTONOMY_INTERFACES_AVAILABLE and GetSystemState is not None:
                try:
                    # Check if service already exists (cleanup from previous run)
                    # If it exists, we'll skip creation but log a warning
                    self.state_service = self.create_service(
                        GetSystemState,
                        "/adaptive_state_machine/get_state",
                        self.get_state_callback,
                    )
                    self.get_logger().info("State service created successfully")
                except Exception as e:
                    error_msg = str(e)
                    if (
                        "existing" in error_msg.lower()
                        or "incompatible" in error_msg.lower()
                    ):
                        # Service might exist from previous run - try to continue
                        self.get_logger().warn(
                            f"Service may already exist: {e}. Continuing..."
                        )
                        self.state_service = None  # Will handle gracefully
                    else:
                        self.get_logger().error(f"Failed to create state service: {e}")
                        self.state_service = None
            else:
                self.get_logger().warn(
                    "GetSystemState service not available (autonomy_interfaces missing)"
                )
                self.state_service = None

            # Create subscription for state change commands
            try:
                self.state_command_sub = self.create_subscription(
                    std_msgs.msg.String,
                    "/adaptive_state_machine/commands",
                    self.state_command_callback,
                    10,
                )
            except Exception as e:
                self.get_logger().error(f"Failed to create command subscription: {e}")

            # Create clients for system monitoring
            if AUTONOMY_INTERFACES_AVAILABLE:
                try:
                    from autonomy_interfaces.srv import (
                        GetSystemHealth,
                        GetNavigationStatus,
                    )

                    self.system_monitor_client = self.create_client(
                        GetSystemHealth, "/system_monitor/get_health"
                    )
                    self.navigation_status_client = self.create_client(
                        GetNavigationStatus, "/navigation/get_status"
                    )
                except (ImportError, Exception) as e:
                    self.system_monitor_client = None
                    self.navigation_status_client = None
                    self.get_logger().warn(
                        f"System monitoring services not available: {e}"
                    )
            else:
                self.system_monitor_client = None
                self.navigation_status_client = None

            # Optional unified blackboard client for syncing mode and safety to BT
            try:
                import sys
                import os

                _src = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
                if _src not in sys.path:
                    sys.path.insert(0, _src)
                from core.unified_blackboard_client import UnifiedBlackboardClient
                from core.blackboard_keys import BlackboardKeys

                self._blackboard = UnifiedBlackboardClient(self)
                self._BlackboardKeys = BlackboardKeys
                self.get_logger().info("Blackboard client initialized for state sync")
            except Exception as e:
                self._blackboard = None
                self._BlackboardKeys = None
                self.get_logger().debug(f"Blackboard not available for state sync: {e}")

            self.get_logger().info("Adaptive State Machine configured")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error during configuration: {e}")
            import traceback

            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Activate the state machine.

        Transitions the state machine to IDLE state and begins normal operation.
        Called during lifecycle node activation phase.

        Parameters
        ----------
        state : LifecycleState
            Current lifecycle state of the node.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS if activation succeeded, FAILURE otherwise.
        """
        self.get_logger().info("Activating Adaptive State Machine...")

        try:
            # Transition to IDLE state
            self.transition_to(SystemState.IDLE)
            self.get_logger().info("Adaptive State Machine activated")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error during activation: {e}")
            import traceback

            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the state machine."""
        self.get_logger().info("Deactivating Adaptive State Machine...")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Clean up the state machine."""
        self.get_logger().info("Cleaning up Adaptive State Machine...")

        # Clean up service and subscriptions
        if hasattr(self, "state_service"):
            self.destroy_service(self.state_service)

        if hasattr(self, "state_command_sub"):
            self.destroy_subscription(self.state_command_sub)

        return TransitionCallbackReturn.SUCCESS

    def get_state_callback(self, request, response):
        """Handle get state service requests from BT orchestrator."""
        if not AUTONOMY_INTERFACES_AVAILABLE or GetSystemState is None:
            self.get_logger().error(
                "GetSystemState service called but interfaces not available"
            )
            return

        response.current_state = self.current_state.value
        response.current_substate = self.current_substate.value
        response.current_teleop_substate = self.current_teleop_substate.value
        meta = dict(self.state_metadata)
        meta["mode_request"] = self.mode_request.value if self.mode_request else None
        meta["human_override_active"] = self.human_override_active
        meta["auto_assist_enabled"] = self.auto_assist_enabled
        response.state_metadata = json.dumps(meta)
        response.last_transition_time = self.last_transition_time

        # Add system health information if available
        try:
            if self.system_monitor_client.service_is_ready():
                health_request = autonomy_interfaces.srv.GetSystemHealth.Request()
                future = self.system_monitor_client.call_async(health_request)
                # Note: In production, this should be handled asynchronously
                response.system_healthy = True  # Placeholder
            else:
                response.system_healthy = True  # Assume healthy if service unavailable
        except:
            response.system_healthy = True

        self.get_logger().debug(f"State query response: {response.current_state}")
        return response

    def state_command_callback(self, msg: std_msgs.msg.String) -> None:
        """
        Handle state change commands.

        Processes JSON-formatted commands received via ROS2 topic to change
        system state or update substates.

        Parameters
        ----------
        msg : std_msgs.msg.String
            ROS2 message containing JSON-encoded command.

        Notes
        -----
        Supported commands:
        - `{"action": "transition", "target_state": "...", "metadata": {...}}`
        - `{"action": "set_substate", "substate_type": "...", "substate": "..."}`

        Raises
        ------
        json.JSONDecodeError
            If message data is not valid JSON.
        """
        try:
            command = json.loads(msg.data)

            if command.get("action") == "transition":
                target_state = command.get("target_state")
                if target_state:
                    self.handle_state_transition(
                        target_state, command.get("metadata", {})
                    )

            elif command.get("action") == "set_mode_request":
                target_state = command.get("target_state")
                if target_state:
                    try:
                        self.mode_request = SystemState(target_state)
                    except ValueError:
                        self.get_logger().warn(
                            f"Invalid mode_request state: {target_state}"
                        )
                else:
                    self.mode_request = None

            elif command.get("action") == "set_human_override":
                self.human_override_active = bool(command.get("value", False))

            elif command.get("action") == "set_auto_assist":
                self.auto_assist_enabled = bool(command.get("value", True))

            elif command.get("action") == "set_substate":
                substate_type = command.get("substate_type", "autonomous")
                substate = command.get("substate")

                if substate_type == "autonomous":
                    try:
                        self.current_substate = AutonomousSubstate(substate)
                    except ValueError:
                        self.get_logger().warn(
                            f"Invalid autonomous substate: {substate}"
                        )
                elif substate_type == "teleoperation":
                    try:
                        self.current_teleop_substate = TeleoperationSubstate(substate)
                    except ValueError:
                        self.get_logger().warn(f"Invalid teleop substate: {substate}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid state command JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing state command: {e}")

    def handle_state_transition(self, target_state_str: str, metadata: Dict[str, Any]):
        """Handle state transition requests."""
        try:
            target_state = SystemState(target_state_str)

            # Validate transition
            if self.can_transition_to(target_state):
                self.transition_to(target_state, metadata)
            else:
                self.get_logger().warn(
                    f"Invalid transition from {self.current_state.value} to {target_state_str}"
                )

        except ValueError:
            self.get_logger().error(f"Unknown target state: {target_state_str}")

    def _is_safe_to_switch_to_autonomous(self) -> bool:
        """Return False if battery is low (do not switch to autonomous)."""
        if not self._blackboard or not self._BlackboardKeys:
            return True
        try:
            battery = self._blackboard.get(
                self._BlackboardKeys.System.BATTERY_PERCENT, 100.0, "double"
            )
            if battery is None:
                return True
            return float(battery) >= self._battery_low_threshold
        except Exception:
            return True

    def can_transition_to(self, target_state: SystemState) -> bool:
        """Check if transition to target state is allowed."""
        # Define valid transitions
        valid_transitions = {
            SystemState.BOOT: [SystemState.IDLE],
            SystemState.IDLE: [SystemState.AUTONOMOUS, SystemState.TELEOPERATION],
            SystemState.AUTONOMOUS: [SystemState.IDLE, SystemState.EMERGENCY_STOP],
            SystemState.TELEOPERATION: [SystemState.IDLE, SystemState.EMERGENCY_STOP],
            SystemState.EMERGENCY_STOP: [SystemState.IDLE],
            SystemState.ERROR: [SystemState.BOOT, SystemState.IDLE],
        }

        # Emergency stop can be triggered from any state
        if target_state == SystemState.EMERGENCY_STOP:
            return True

        # Do not switch to autonomous if battery is low
        if (
            target_state == SystemState.AUTONOMOUS
            and not self._is_safe_to_switch_to_autonomous()
        ):
            self.get_logger().warn(
                "Cannot switch to AUTONOMOUS: battery below threshold"
            )
            return False

        return target_state in valid_transitions.get(self.current_state, [])

    def _sync_state_to_blackboard(self) -> None:
        """Write current state and hybrid flags to blackboard for BT."""
        if not self._blackboard or not self._BlackboardKeys:
            return
        try:
            BK = self._BlackboardKeys
            mode_str = {
                SystemState.TELEOPERATION: "teleop",
                SystemState.AUTONOMOUS: "autonomous",
                SystemState.EMERGENCY_STOP: "emergency",
                SystemState.IDLE: "idle",
                SystemState.BOOT: "boot",
                SystemState.ERROR: "error",
            }.get(self.current_state, "idle")
            self._blackboard.set(BK.Auto.MODE, mode_str)
            self._blackboard.set(
                BK.Auto.HUMAN_OVERRIDE_ACTIVE, self.human_override_active
            )
            self._blackboard.set(BK.Auto.ASSIST_ENABLED, self.auto_assist_enabled)
            if self.current_state == SystemState.EMERGENCY_STOP:
                self._blackboard.set(BK.System.EMERGENCY_STOP, True)
            else:
                self._blackboard.set(BK.System.EMERGENCY_STOP, False)
        except Exception as e:
            self.get_logger().debug(f"Blackboard sync failed: {e}")

    def transition_to(
        self, new_state: SystemState, metadata: Optional[Dict[str, Any]] = None
    ) -> None:
        """Perform state transition."""
        old_state = self.current_state

        self.current_state = new_state
        self.last_transition_time = time.time()
        self.state_metadata.update(metadata or {})

        # Clear mode_request if we transitioned to the requested state
        if self.mode_request == new_state:
            self.mode_request = None

        # Reset substates when entering certain states
        if new_state == SystemState.IDLE:
            self.current_substate = AutonomousSubstate.NONE
            self.current_teleop_substate = TeleoperationSubstate.NONE
        elif new_state == SystemState.EMERGENCY_STOP:
            self.current_substate = AutonomousSubstate.NONE
            self.current_teleop_substate = TeleoperationSubstate.NONE

        # Sync mode and safety to blackboard so BT sees them
        self._sync_state_to_blackboard()

        # Publish state change
        try:
            if AUTONOMY_INTERFACES_AVAILABLE and SystemStateMsg is not None:
                state_msg = SystemStateMsg()
                # Map to actual message fields
                state_msg.current_state = self.current_state.value
                # Message uses 'substate' not 'current_substate'
                if hasattr(state_msg, "substate"):
                    state_msg.substate = self.current_substate.value
                elif hasattr(state_msg, "current_substate"):
                    state_msg.current_substate = self.current_substate.value
                # Message may use 'sub_substate' for teleop substate
                if hasattr(state_msg, "sub_substate"):
                    state_msg.sub_substate = self.current_teleop_substate.value
                elif hasattr(state_msg, "current_teleop_substate"):
                    state_msg.current_teleop_substate = (
                        self.current_teleop_substate.value
                    )
                # Store metadata in available fields
                if hasattr(state_msg, "state_reason"):
                    state_msg.state_reason = json.dumps(self.state_metadata)
                # Use time_in_state for timing
                if hasattr(state_msg, "time_in_state"):
                    state_msg.time_in_state = time.time() - self.last_transition_time
                # transition_timestamp is a builtin_interfaces/Time, skip for now to avoid crash
                # if hasattr(state_msg, 'transition_timestamp'):
                #     from builtin_interfaces.msg import Time
                #     state_msg.transition_timestamp = Time()
                self.state_publisher.publish(state_msg)
            else:
                # Fallback to String message
                from std_msgs.msg import String

                state_str = json.dumps(
                    {
                        "current_state": self.current_state.value,
                        "current_substate": self.current_substate.value,
                        "current_teleop_substate": self.current_teleop_substate.value,
                        "state_metadata": self.state_metadata,
                        "last_transition_time": self.last_transition_time,
                    }
                )
                state_msg = String()
                state_msg.data = state_str
                self.state_publisher.publish(state_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing state: {e}")
            import traceback

            self.get_logger().error(traceback.format_exc())

        self.get_logger().info(
            f"State transition: {old_state.value} -> {new_state.value}"
        )

    def update_substate(
        self, substate: AutonomousSubstate, metadata: Optional[Dict[str, Any]] = None
    ):
        """Update autonomous substate."""
        if self.current_state == SystemState.AUTONOMOUS:
            self.current_substate = substate
            self.state_metadata.update(metadata or {})

            # Publish updated state
            try:
                if AUTONOMY_INTERFACES_AVAILABLE and SystemStateMsg is not None:
                    state_msg = SystemStateMsg()
                    state_msg.current_state = self.current_state.value
                    # Map to actual message fields
                    if hasattr(state_msg, "substate"):
                        state_msg.substate = self.current_substate.value
                    elif hasattr(state_msg, "current_substate"):
                        state_msg.current_substate = self.current_substate.value
                    if hasattr(state_msg, "sub_substate"):
                        state_msg.sub_substate = self.current_teleop_substate.value
                    elif hasattr(state_msg, "current_teleop_substate"):
                        state_msg.current_teleop_substate = (
                            self.current_teleop_substate.value
                        )
                    if hasattr(state_msg, "state_reason"):
                        state_msg.state_reason = json.dumps(self.state_metadata)
                    if hasattr(state_msg, "time_in_state"):
                        state_msg.time_in_state = (
                            time.time() - self.last_transition_time
                        )
                    # transition_timestamp is a builtin_interfaces/Time, skip for now
                    # if hasattr(state_msg, 'transition_timestamp'):
                    #     from builtin_interfaces.msg import Time
                    #     state_msg.transition_timestamp = Time()
                    self.state_publisher.publish(state_msg)
                else:
                    # Fallback to String message
                    from std_msgs.msg import String

                    state_str = json.dumps(
                        {
                            "current_state": self.current_state.value,
                            "current_substate": self.current_substate.value,
                            "current_teleop_substate": self.current_teleop_substate.value,
                            "state_metadata": self.state_metadata,
                            "last_transition_time": self.last_transition_time,
                        }
                    )
                    state_msg = String()
                    state_msg.data = state_str
                    self.state_publisher.publish(state_msg)
            except Exception as e:
                self.get_logger().error(f"Error publishing substate: {e}")
                import traceback

                self.get_logger().error(traceback.format_exc())

            self.get_logger().info(f"Substate updated: {substate.value}")

    def get_current_state_info(self) -> Dict[str, Any]:
        """Get comprehensive current state information."""
        return {
            "state": self.current_state.value,
            "substate": self.current_substate.value,
            "teleop_substate": self.current_teleop_substate.value,
            "metadata": self.state_metadata,
            "last_transition": self.last_transition_time,
            "time_in_state": time.time() - self.last_transition_time,
        }


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    # Create lifecycle node
    node = AdaptiveStateMachine()

    # Use lifecycle manager
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        # Configure and activate the node
        ret = node.trigger_configure()
        if ret == TransitionCallbackReturn.SUCCESS:
            ret = node.trigger_activate()
            if ret == TransitionCallbackReturn.SUCCESS:
                executor.spin()
            else:
                node.get_logger().error("Failed to activate node")
        else:
            node.get_logger().error("Failed to configure node")
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        # Context already shut down (e.g. SIGTERM); do not call rclpy.shutdown() again
        pass
    finally:
        # Clean shutdown - only if node was successfully configured
        try:
            current_state = node.get_current_state()
            if current_state.id == 2:  # inactive state
                node.trigger_deactivate()
            if current_state.id >= 1:  # configured or above
                node.trigger_cleanup()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
