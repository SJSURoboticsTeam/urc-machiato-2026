// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/SystemState.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__SYSTEM_STATE__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__SYSTEM_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'current_state'
// Member 'substate'
// Member 'sub_substate'
// Member 'previous_state'
// Member 'active_subsystems'
// Member 'failed_subsystems'
// Member 'mission_phase'
// Member 'operator_id'
// Member 'state_reason'
#include "rosidl_runtime_c/string.h"
// Member 'transition_timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/SystemState in the package autonomy_interfaces.
/**
  * System State Message - URC 2026 Mars Rover Autonomy
  *
  * This message provides comprehensive state information for the rover's hierarchical
  * state machine. It tracks the current operational state, subsystem status, and
  * mission context to enable coordinated autonomous operations.
  *
  * The state machine uses a three-level hierarchy:
  * - current_state: Top-level operational mode (BOOT, CALIBRATION, IDLE, etc.)
  * - substate: Mission-specific context (SCIENCE, DELIVERY, etc.)
  * - sub_substate: Detailed operational state (SAMPLE_DELIVERY, etc.)
  *
  * This design allows for flexible mission execution while maintaining clear
  * operational boundaries and safety constraints.
 */
typedef struct autonomy_interfaces__msg__SystemState
{
  std_msgs__msg__Header header;
  /// Primary state information
  /// Top-level state (BOOT, CALIBRATION, IDLE, NAVIGATION, EXECUTION, RECOVERY, SHUTDOWN)
  rosidl_runtime_c__String current_state;
  /// Mission-specific substate (SCIENCE, DELIVERY, MAINTENANCE, etc.)
  rosidl_runtime_c__String substate;
  /// Detailed substate (SAMPLE_COLLECTION, DELIVERY_SETUP, DIAGNOSTIC_CHECK, etc.)
  rosidl_runtime_c__String sub_substate;
  /// State metadata
  /// Seconds spent in current state (resets on transition)
  double time_in_state;
  /// Maximum allowed time in state (0.0 = no timeout, seconds)
  double state_timeout;
  /// Last state before current transition
  rosidl_runtime_c__String previous_state;
  /// UTC timestamp when current state was entered
  builtin_interfaces__msg__Time transition_timestamp;
  /// System status flags
  /// True during state transitions (prevents conflicting operations)
  bool is_transitioning;
  /// True when all autonomous operation prerequisites are satisfied
  bool preconditions_met;
  /// List of currently active subsystems (navigation, vision, slam, etc.)
  rosidl_runtime_c__String__Sequence active_subsystems;
  /// List of subsystems currently in error state
  rosidl_runtime_c__String__Sequence failed_subsystems;
  /// Mission and operational context
  /// Competition mission context (setup, terrain, equipment_service, science, etc.)
  rosidl_runtime_c__String mission_phase;
  /// Identifier of who/what initiated current state (auto, manual, safety_system)
  rosidl_runtime_c__String operator_id;
  /// Human-readable explanation for current state and any restrictions
  rosidl_runtime_c__String state_reason;
} autonomy_interfaces__msg__SystemState;

// Struct for a sequence of autonomy_interfaces__msg__SystemState.
typedef struct autonomy_interfaces__msg__SystemState__Sequence
{
  autonomy_interfaces__msg__SystemState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__SystemState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__SYSTEM_STATE__STRUCT_H_
