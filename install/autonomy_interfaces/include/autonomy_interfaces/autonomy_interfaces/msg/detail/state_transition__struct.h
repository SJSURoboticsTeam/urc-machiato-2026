// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/StateTransition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/state_transition.h"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__STATE_TRANSITION__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__STATE_TRANSITION__STRUCT_H_

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
// Member 'from_state'
// Member 'to_state'
// Member 'reason'
// Member 'initiated_by'
// Member 'failure_reason'
// Member 'preconditions_checked'
// Member 'entry_actions_executed'
// Member 'exit_actions_executed'
#include "rosidl_runtime_c/string.h"
// Member 'start_time'
// Member 'end_time'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/StateTransition in the package autonomy_interfaces.
/**
  * State Transition Event Message
  * Records state transitions for logging and debugging
 */
typedef struct autonomy_interfaces__msg__StateTransition
{
  std_msgs__msg__Header header;
  /// Transition details
  /// State transitioning from
  rosidl_runtime_c__String from_state;
  /// State transitioning to
  rosidl_runtime_c__String to_state;
  /// When transition started
  builtin_interfaces__msg__Time start_time;
  /// When transition completed
  builtin_interfaces__msg__Time end_time;
  /// Duration in seconds
  double transition_duration;
  /// Transition metadata
  /// True if transition completed successfully
  bool success;
  /// Reason for transition
  rosidl_runtime_c__String reason;
  /// Source of transition request (frontend, safety, auto)
  rosidl_runtime_c__String initiated_by;
  /// If failed, why it failed
  rosidl_runtime_c__String failure_reason;
  /// Context information
  /// List of preconditions that were validated
  rosidl_runtime_c__String__Sequence preconditions_checked;
  /// Actions executed on entering new state
  rosidl_runtime_c__String__Sequence entry_actions_executed;
  /// Actions executed on leaving old state
  rosidl_runtime_c__String__Sequence exit_actions_executed;
} autonomy_interfaces__msg__StateTransition;

// Struct for a sequence of autonomy_interfaces__msg__StateTransition.
typedef struct autonomy_interfaces__msg__StateTransition__Sequence
{
  autonomy_interfaces__msg__StateTransition * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__StateTransition__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__STATE_TRANSITION__STRUCT_H_
