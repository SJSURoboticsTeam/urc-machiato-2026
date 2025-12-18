// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/ChangeState.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__CHANGE_STATE__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__CHANGE_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'desired_state'
// Member 'desired_substate'
// Member 'desired_calibration_substate'
// Member 'reason'
// Member 'operator_id'
// Member 'metadata'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ChangeState in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__ChangeState_Request
{
  /// Request
  /// Target state to transition to
  rosidl_runtime_c__String desired_state;
  /// Optional: target substate (empty string if not applicable)
  rosidl_runtime_c__String desired_substate;
  /// Optional: target calibration substate (empty string if not applicable)
  rosidl_runtime_c__String desired_calibration_substate;
  /// Human-readable reason for state change
  rosidl_runtime_c__String reason;
  /// ID of operator/system requesting change
  rosidl_runtime_c__String operator_id;
  /// If true, skip some validation checks (use with caution)
  bool force;
  /// Additional key=value metadata pairs
  rosidl_runtime_c__String__Sequence metadata;
} autonomy_interfaces__srv__ChangeState_Request;

// Struct for a sequence of autonomy_interfaces__srv__ChangeState_Request.
typedef struct autonomy_interfaces__srv__ChangeState_Request__Sequence
{
  autonomy_interfaces__srv__ChangeState_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__ChangeState_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'actual_state'
// Member 'actual_substate'
// Member 'actual_calibration_substate'
// Member 'message'
// Member 'failed_preconditions'
// Member 'warnings'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ChangeState in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__ChangeState_Response
{
  /// True if transition successful
  bool success;
  /// Actual state after transition attempt
  rosidl_runtime_c__String actual_state;
  /// Actual substate after transition
  rosidl_runtime_c__String actual_substate;
  /// Actual calibration substate after transition
  rosidl_runtime_c__String actual_calibration_substate;
  /// Time taken to complete transition (seconds)
  double transition_time;
  /// Human-readable status message
  rosidl_runtime_c__String message;
  /// Validation details
  /// Were all preconditions satisfied
  bool preconditions_met;
  /// List of preconditions that failed
  rosidl_runtime_c__String__Sequence failed_preconditions;
  /// Non-critical warnings about the transition
  rosidl_runtime_c__String__Sequence warnings;
} autonomy_interfaces__srv__ChangeState_Response;

// Struct for a sequence of autonomy_interfaces__srv__ChangeState_Response.
typedef struct autonomy_interfaces__srv__ChangeState_Response__Sequence
{
  autonomy_interfaces__srv__ChangeState_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__ChangeState_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__CHANGE_STATE__STRUCT_H_
