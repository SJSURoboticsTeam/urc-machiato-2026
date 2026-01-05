#!/usr/bin/env python3
"""
Unified State Management System - Single Interface for All State Handling

Consolidates all state management functionality:
- Hierarchical state machines (from state_machine.py)
- Behavior trees (from behavior_tree.py)
- State synchronization (from state_synchronization_manager.py)
- Autonomy state management (from autonomy/core/state_management/)

Features:
- Unified API for different state management paradigms
- Hierarchical state machines with transitions
- Behavior trees for complex decision making
- State synchronization across components
- Real-time state monitoring and persistence
- Configurable state validation and constraints

Author: URC 2026 Unified State Management Team
"""

import time
import threading
import json
from typing import Dict, List, Any, Optional, Callable, Union, Type, Set
from dataclasses import dataclass, field
from enum import Enum
import logging
import weakref

# State management libraries with fallbacks
try:
    from transitions import Machine, State
    from transitions.extensions import HierarchicalMachine, GraphMachine, LockedMachine
    try:
        import graphviz
        GRAPHVIZ_AVAILABLE = True
    except ImportError:
        GRAPHVIZ_AVAILABLE = False
    TRANSITIONS_AVAILABLE = True
except ImportError:
    TRANSITIONS_AVAILABLE = False
    GRAPHVIZ_AVAILABLE = False
    # Fallback implementations
    class Machine:
        def __init__(self, *args, **kwargs): pass
        def add_transition(self, *args, **kwargs): pass
        def to_state(self, *args): pass
    class State: pass
    class HierarchicalMachine: pass
    class GraphMachine: pass

try:
    import py_trees
    from py_trees.behaviour import Behaviour
    from py_trees.composites import Sequence, Selector, Parallel
    from py_trees.trees import BehaviourTree
    from py_trees.blackboard import Blackboard
    from py_trees.common import Status
    from py_trees.display import render_dot_tree
    PY_TREES_AVAILABLE = True
except ImportError:
    PY_TREES_AVAILABLE = False
    # Fallback implementations
    class Behaviour:
        def __init__(self, name): self.name = name
        def update(self): return "SUCCESS"
        def tick(self): pass
    class Status:
        SUCCESS = "SUCCESS"
        FAILURE = "FAILURE"
        RUNNING = "RUNNING"
    class Sequence: pass
    class Selector: pass
    class Parallel: pass
    class BehaviourTree: pass
    class Blackboard: pass

logger = logging.getLogger(__name__)


class StateType(Enum):
    """Types of state management systems."""
    HIERARCHICAL_STATE_MACHINE = "hierarchical_state_machine"
    BEHAVIOR_TREE = "behavior_tree"
    FINITE_STATE_MACHINE = "finite_state_machine"
    CUSTOM = "custom"


class StateStatus(Enum):
    """State execution status."""
    IDLE = "idle"
    RUNNING = "running"
    SUCCESS = "success"
    FAILURE = "failure"
    ERROR = "error"
    PAUSED = "paused"
    STOPPED = "stopped"


@dataclass
class StateDefinition:
    """Definition of a state."""
    name: str
    type: StateType
    description: str = ""
    entry_actions: List[Callable] = field(default_factory=list)
    exit_actions: List[Callable] = field(default_factory=list)
    transitions: Dict[str, str] = field(default_factory=dict)  # event -> target_state
    data: Dict[str, Any] = field(default_factory=dict)
    parent: Optional[str] = None  # For hierarchical states
    children: List[str] = field(default_factory=list)


@dataclass
class StateInstance:
    """Runtime instance of a state."""
    definition: StateDefinition
    status: StateStatus = StateStatus.IDLE
    entered_at: Optional[float] = None
    exited_at: Optional[float] = None
    execution_count: int = 0
    last_error: Optional[str] = None
    data: Dict[str, Any] = field(default_factory=dict)


@dataclass
class TransitionEvent:
    """State transition event."""
    from_state: str
    to_state: str
    event: str
    timestamp: float = field(default_factory=time.time)
    data: Dict[str, Any] = field(default_factory=dict)
    success: bool = True


class HierarchicalStateMachine:
    """
    Hierarchical state machine implementation.
    Consolidates functionality from state_machine.py.
    """

    def __init__(self, name: str, initial_state: str = "idle"):
        self.name = name
        self.states: Dict[str, StateDefinition] = {}
        self.current_state: Optional[str] = None
        self.state_stack: List[str] = []  # For hierarchical navigation
        self.transition_history: List[TransitionEvent] = []
        self.event_handlers: Dict[str, List[Callable]] = {}
        self.validators: Dict[str, Callable] = {}

        # Create machine if available
        if TRANSITIONS_AVAILABLE:
            self.machine = HierarchicalMachine(
                model=self,
                states=[],
                initial=initial_state,
                model_attribute='current_state'
            )
        else:
            self.machine = None

        # Add initial idle state
        self.add_state("idle", "Idle state")
        self.current_state = initial_state

    def add_state(self, name: str, description: str = "",
                  entry_actions: Optional[List[Callable]] = None,
                  exit_actions: Optional[List[Callable]] = None,
                  parent: Optional[str] = None) -> StateDefinition:
        """Add a state to the machine."""

        state_def = StateDefinition(
            name=name,
            type=StateType.HIERARCHICAL_STATE_MACHINE,
            description=description,
            entry_actions=entry_actions or [],
            exit_actions=exit_actions or [],
            parent=parent
        )

        self.states[name] = state_def

        # Add to parent if hierarchical
        if parent and parent in self.states:
            self.states[parent].children.append(name)

        # Add to transitions machine if available
        if self.machine:
            try:
                self.machine.add_state(name, on_enter=entry_actions, on_exit=exit_actions)
            except Exception as e:
                logger.warning(f"Failed to add state to transitions machine: {e}")

        return state_def

    def add_transition(self, event: str, source: str, target: str,
                      conditions: Optional[List[Callable]] = None,
                      before: Optional[List[Callable]] = None,
                      after: Optional[List[Callable]] = None):
        """Add a transition between states."""

        if source in self.states:
            self.states[source].transitions[event] = target

        # Add to transitions machine if available
        if self.machine:
            try:
                self.machine.add_transition(event, source, target,
                                          conditions=conditions,
                                          before=before,
                                          after=after)
            except Exception as e:
                logger.warning(f"Failed to add transition to machine: {e}")

        # Register event handler
        if event not in self.event_handlers:
            self.event_handlers[event] = []
        if after:
            self.event_handlers[event].extend(after)

    def add_validator(self, state: str, validator: Callable):
        """Add a state transition validator."""
        self.validators[state] = validator

    def trigger_event(self, event: str, **kwargs) -> bool:
        """Trigger a state transition event."""
        if self.machine:
            try:
                # Use transitions library
                getattr(self.machine, event)(**kwargs)
                success = True
            except Exception as e:
                logger.error(f"State transition failed: {e}")
                success = False
        else:
            # Manual transition logic
            success = self._manual_transition(event, **kwargs)

        # Record transition
        if success and self.current_state:
            transition = TransitionEvent(
                from_state=self.current_state,
                to_state=self.current_state,  # Will be updated by transition
                event=event,
                data=kwargs,
                success=success
            )
            self.transition_history.append(transition)

        # Call event handlers
        if event in self.event_handlers:
            for handler in self.event_handlers[event]:
                try:
                    handler(**kwargs)
                except Exception as e:
                    logger.error(f"Event handler error: {e}")

        return success

    def _manual_transition(self, event: str, **kwargs) -> bool:
        """Manual state transition logic."""
        if not self.current_state or self.current_state not in self.states:
            return False

        current_def = self.states[self.current_state]
        target_state = current_def.transitions.get(event)

        if not target_state or target_state not in self.states:
            return False

        # Validate transition
        if target_state in self.validators:
            if not self.validators[target_state](**kwargs):
                return False

        # Execute exit actions
        for action in current_def.exit_actions:
            try:
                action()
            except Exception as e:
                logger.error(f"Exit action error: {e}")

        # Change state
        old_state = self.current_state
        self.current_state = target_state

        # Execute entry actions
        target_def = self.states[target_state]
        for action in target_def.entry_actions:
            try:
                action()
            except Exception as e:
                logger.error(f"Entry action error: {e}")

        logger.info(f"State transition: {old_state} -> {target_state} (event: {event})")
        return True

    def get_current_state(self) -> Optional[str]:
        """Get current state name."""
        return self.current_state

    def get_state_definition(self, name: str) -> Optional[StateDefinition]:
        """Get state definition."""
        return self.states.get(name)

    def get_transition_history(self, limit: int = 50) -> List[TransitionEvent]:
        """Get recent transition history."""
        return self.transition_history[-limit:]

    def to_graphviz(self) -> Optional[str]:
        """Generate GraphViz dot representation."""
        if not GRAPHVIZ_AVAILABLE or not self.machine:
            return None

        try:
            return self.machine.to_graphviz()
        except Exception as e:
            logger.error(f"GraphViz generation failed: {e}")
            return None


class BehaviorTree:
    """
    Behavior tree implementation.
    Consolidates functionality from behavior_tree.py.
    """

    def __init__(self, name: str, root: Optional[Any] = None):
        self.name = name
        self.root = root
        self.blackboard = Blackboard() if PY_TREES_AVAILABLE else {}
        self.tree: Optional[BehaviourTree] = None
        self.status = Status.INVALID
        self.execution_count = 0
        self.last_execution_time = 0.0

        if PY_TREES_AVAILABLE and root:
            self.tree = BehaviourTree(root)

    def set_root(self, root: Any):
        """Set the root behavior."""
        self.root = root
        if PY_TREES_AVAILABLE:
            self.tree = BehaviourTree(root)

    def add_behavior(self, behavior: Any, parent: Optional[Any] = None):
        """Add a behavior to the tree."""
        if parent is None:
            self.set_root(behavior)
        # For more complex trees, would need to implement proper insertion

    def tick(self) -> str:
        """Execute one tick of the behavior tree."""
        self.execution_count += 1
        start_time = time.time()

        if PY_TREES_AVAILABLE and self.tree:
            self.status = self.tree.tick()
            self.last_execution_time = time.time() - start_time
            return self.status.name
        else:
            # Fallback execution
            if self.root and hasattr(self.root, 'update'):
                try:
                    result = self.root.update()
                    self.status = getattr(Status, result, Status.INVALID)
                    self.last_execution_time = time.time() - start_time
                    return result
                except Exception as e:
                    logger.error(f"Behavior tree execution error: {e}")
                    return "ERROR"

        return "INVALID"

    def get_status(self) -> str:
        """Get current tree status."""
        return self.status.name if hasattr(self.status, 'name') else str(self.status)

    def get_blackboard_value(self, key: str) -> Any:
        """Get value from blackboard."""
        if PY_TREES_AVAILABLE:
            return self.blackboard.get(key)
        elif isinstance(self.blackboard, dict):
            return self.blackboard.get(key)
        return None

    def set_blackboard_value(self, key: str, value: Any):
        """Set value in blackboard."""
        if PY_TREES_AVAILABLE:
            self.blackboard.set(key, value)
        elif isinstance(self.blackboard, dict):
            self.blackboard[key] = value

    def get_execution_stats(self) -> Dict[str, Any]:
        """Get execution statistics."""
        return {
            "execution_count": self.execution_count,
            "last_execution_time": self.last_execution_time,
            "average_execution_time": self.last_execution_time,  # Simplified
            "status": self.get_status()
        }


class StateSynchronizationManager:
    """
    State synchronization across components.
    Consolidates functionality from state_synchronization_manager.py.
    """

    def __init__(self):
        self.states: Dict[str, Any] = {}
        self.subscribers: Dict[str, List[Callable]] = {}
        self.sync_lock = threading.Lock()
        self.last_sync_time = 0.0
        self.sync_interval = 1.0  # seconds

    def register_state(self, name: str, initial_value: Any = None):
        """Register a state for synchronization."""
        with self.sync_lock:
            self.states[name] = initial_value

    def update_state(self, name: str, value: Any) -> bool:
        """Update a state value and notify subscribers."""
        with self.sync_lock:
            if name not in self.states:
                return False

            old_value = self.states[name]
            self.states[name] = value
            self.last_sync_time = time.time()

            # Notify subscribers
            if name in self.subscribers:
                for subscriber in self.subscribers[name]:
                    try:
                        subscriber(name, old_value, value)
                    except Exception as e:
                        logger.error(f"State subscriber error: {e}")

            return True

    def get_state(self, name: str) -> Any:
        """Get current state value."""
        with self.sync_lock:
            return self.states.get(name)

    def subscribe_to_state(self, name: str, callback: Callable):
        """Subscribe to state changes."""
        with self.sync_lock:
            if name not in self.subscribers:
                self.subscribers[name] = []
            self.subscribers[name].append(callback)

    def unsubscribe_from_state(self, name: str, callback: Callable):
        """Unsubscribe from state changes."""
        with self.sync_lock:
            if name in self.subscribers and callback in self.subscribers[name]:
                self.subscribers[name].remove(callback)

    def get_all_states(self) -> Dict[str, Any]:
        """Get all current states."""
        with self.sync_lock:
            return self.states.copy()

    def sync_states(self, external_states: Dict[str, Any]):
        """Synchronize with external state source."""
        with self.sync_lock:
            for name, value in external_states.items():
                if name in self.states:
                    self.update_state(name, value)


class StateManager:
    """
    Unified State Manager - Single interface for all state management paradigms.

    Consolidates:
    - Hierarchical state machines (from state_machine.py)
    - Behavior trees (from behavior_tree.py)
    - State synchronization (from state_synchronization_manager.py)
    - Component state management

    Features:
    - Multiple state management paradigms in one system
    - Real-time state monitoring and synchronization
    - State persistence and recovery
    - Performance profiling and optimization
    - Event-driven state transitions
    """

    def __init__(self):
        self.state_machines: Dict[str, HierarchicalStateMachine] = {}
        self.behavior_trees: Dict[str, BehaviorTree] = {}
        self.sync_manager = StateSynchronizationManager()
        self.state_instances: Dict[str, StateInstance] = {}
        self.event_handlers: Dict[str, List[Callable]] = {}
        self.persistence_enabled = False
        self.persistence_file: Optional[str] = None

        # Monitoring
        self.transition_count = 0
        self.error_count = 0
        self.last_activity = time.time()

    def create_state_machine(self, name: str, initial_state: str = "idle") -> HierarchicalStateMachine:
        """Create a new hierarchical state machine."""
        machine = HierarchicalStateMachine(name, initial_state)
        self.state_machines[name] = machine
        logger.info(f"Created state machine: {name}")
        return machine

    def create_behavior_tree(self, name: str, root: Optional[Any] = None) -> BehaviorTree:
        """Create a new behavior tree."""
        tree = BehaviorTree(name, root)
        self.behavior_trees[name] = tree
        logger.info(f"Created behavior tree: {name}")
        return tree

    def register_state_instance(self, name: str, definition: StateDefinition) -> StateInstance:
        """Register a state instance."""
        instance = StateInstance(definition=definition)
        self.state_instances[name] = instance
        return instance

    def execute_state_machine(self, name: str, event: str, **kwargs) -> bool:
        """Execute an event on a state machine."""
        if name not in self.state_machines:
            logger.error(f"State machine not found: {name}")
            return False

        machine = self.state_machines[name]
        success = machine.trigger_event(event, **kwargs)

        if success:
            self.transition_count += 1
            self.last_activity = time.time()
        else:
            self.error_count += 1

        return success

    def execute_behavior_tree(self, name: str) -> str:
        """Execute one tick of a behavior tree."""
        if name not in self.behavior_trees:
            logger.error(f"Behavior tree not found: {name}")
            return "ERROR"

        tree = self.behavior_trees[name]
        status = tree.tick()
        self.last_activity = time.time()
        return status

    def update_state(self, name: str, value: Any) -> bool:
        """Update a synchronized state."""
        return self.sync_manager.update_state(name, value)

    def get_state(self, name: str) -> Any:
        """Get a synchronized state value."""
        return self.sync_manager.get_state(name)

    def subscribe_to_state(self, state_name: str, callback: Callable):
        """Subscribe to state changes."""
        self.sync_manager.subscribe_to_state(state_name, callback)

    def add_event_handler(self, event: str, handler: Callable):
        """Add a global event handler."""
        if event not in self.event_handlers:
            self.event_handlers[event] = []
        self.event_handlers[event].append(handler)

    def trigger_global_event(self, event: str, **kwargs):
        """Trigger a global event to all handlers."""
        if event in self.event_handlers:
            for handler in self.event_handlers[event]:
                try:
                    handler(**kwargs)
                except Exception as e:
                    logger.error(f"Global event handler error: {e}")

    def enable_persistence(self, file_path: str):
        """Enable state persistence."""
        self.persistence_enabled = True
        self.persistence_file = file_path
        self._load_persistent_state()

    def disable_persistence(self):
        """Disable state persistence."""
        self.persistence_enabled = False
        self.persistence_file = None

    def save_state(self):
        """Save current state to persistent storage."""
        if not self.persistence_enabled or not self.persistence_file:
            return

        try:
            state_data = {
                "timestamp": time.time(),
                "state_machines": {
                    name: machine.get_current_state()
                    for name, machine in self.state_machines.items()
                },
                "synchronized_states": self.sync_manager.get_all_states(),
                "transition_count": self.transition_count,
                "error_count": self.error_count
            }

            with open(self.persistence_file, 'w') as f:
                json.dump(state_data, f, indent=2, default=str)

        except Exception as e:
            logger.error(f"State persistence save failed: {e}")

    def load_state(self):
        """Load state from persistent storage."""
        if not self.persistence_enabled or not self.persistence_file:
            return

        try:
            with open(self.persistence_file, 'r') as f:
                state_data = json.load(f)

            # Restore synchronized states
            if "synchronized_states" in state_data:
                self.sync_manager.sync_states(state_data["synchronized_states"])

            # Restore state machine states (simplified)
            if "state_machines" in state_data:
                for name, state in state_data["state_machines"].items():
                    if name in self.state_machines:
                        try:
                            # This is simplified - real restoration would be more complex
                            machine = self.state_machines[name]
                            if hasattr(machine, 'to_state'):
                                machine.to_state(state)
                        except Exception as e:
                            logger.error(f"Failed to restore state machine {name}: {e}")

        except FileNotFoundError:
            logger.info("No persistent state file found, starting fresh")
        except Exception as e:
            logger.error(f"State persistence load failed: {e}")

    def _load_persistent_state(self):
        """Load persistent state on startup."""
        self.load_state()

    def get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive state management system status."""
        return {
            "timestamp": time.time(),
            "state_machines": {
                name: {
                    "current_state": machine.get_current_state(),
                    "states_count": len(machine.states),
                    "transitions_count": len(machine.transition_history)
                }
                for name, machine in self.state_machines.items()
            },
            "behavior_trees": {
                name: tree.get_execution_stats()
                for name, tree in self.behavior_trees.items()
            },
            "synchronized_states": self.sync_manager.get_all_states(),
            "state_instances": len(self.state_instances),
            "event_handlers": len(self.event_handlers),
            "persistence": {
                "enabled": self.persistence_enabled,
                "file": self.persistence_file
            },
            "metrics": {
                "transition_count": self.transition_count,
                "error_count": self.error_count,
                "last_activity": self.last_activity,
                "uptime_seconds": time.time() - self.last_activity if self.last_activity else 0
            }
        }

    def shutdown(self):
        """Shutdown the state management system."""
        # Save state if persistence is enabled
        if self.persistence_enabled:
            self.save_state()

        logger.info("State management system shutdown")


# Global state manager instance
_state_manager = None

def get_state_manager() -> StateManager:
    """Get global state manager instance."""
    global _state_manager
    if _state_manager is None:
        _state_manager = StateManager()
    return _state_manager

# Convenience functions
def create_state_machine(name: str, initial_state: str = "idle") -> HierarchicalStateMachine:
    """Create a state machine."""
    return get_state_manager().create_state_machine(name, initial_state)

def create_behavior_tree(name: str, root: Optional[Any] = None) -> BehaviorTree:
    """Create a behavior tree."""
    return get_state_manager().create_behavior_tree(name, root)

def update_state(name: str, value: Any) -> bool:
    """Update a synchronized state."""
    return get_state_manager().update_state(name, value)

def get_state(name: str) -> Any:
    """Get a synchronized state."""
    return get_state_manager().get_state(name)

# Export key components
__all__ = [
    'StateManager',
    'HierarchicalStateMachine',
    'BehaviorTree',
    'StateSynchronizationManager',
    'StateType',
    'StateStatus',
    'StateDefinition',
    'StateInstance',
    'TransitionEvent',
    'get_state_manager',
    'create_state_machine',
    'create_behavior_tree',
    'update_state',
    'get_state'
]

