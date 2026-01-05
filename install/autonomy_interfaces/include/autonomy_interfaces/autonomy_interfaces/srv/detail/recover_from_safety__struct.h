// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/RecoverFromSafety.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/recover_from_safety.h"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__RECOVER_FROM_SAFETY__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__RECOVER_FROM_SAFETY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'recovery_method'
// Member 'operator_id'
// Member 'completed_steps'
// Member 'notes'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RecoverFromSafety in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__RecoverFromSafety_Request
{
  /// Request
  /// "AUTO", "MANUAL_GUIDED", "FULL_RESET"
  rosidl_runtime_c__String recovery_method;
  /// ID of operator initiating recovery
  rosidl_runtime_c__String operator_id;
  /// Operator acknowledges recovery risks
  bool acknowledge_risks;
  /// Manual steps already completed (for verification)
  rosidl_runtime_c__String__Sequence completed_steps;
  /// Additional notes about recovery conditions
  rosidl_runtime_c__String notes;
} autonomy_interfaces__srv__RecoverFromSafety_Request;

// Struct for a sequence of autonomy_interfaces__srv__RecoverFromSafety_Request.
typedef struct autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence
{
  autonomy_interfaces__srv__RecoverFromSafety_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// Member 'recovery_state'
// Member 'remaining_steps'
// Member 'verified_systems'
// Member 'failed_systems'
// Member 'recommended_next_state'
// Member 'restrictions'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RecoverFromSafety in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__RecoverFromSafety_Response
{
  /// True if recovery initiated/completed
  bool success;
  /// Human-readable status message
  rosidl_runtime_c__String message;
  /// "IN_PROGRESS", "COMPLETED", "FAILED", "REQUIRES_ACTION"
  rosidl_runtime_c__String recovery_state;
  /// Recovery details
  /// True if safe to continue operations
  bool is_safe_to_proceed;
  /// Steps still needed for full recovery
  rosidl_runtime_c__String__Sequence remaining_steps;
  /// Systems that passed verification
  rosidl_runtime_c__String__Sequence verified_systems;
  /// Systems that failed verification
  rosidl_runtime_c__String__Sequence failed_systems;
  /// Estimated time to complete recovery (seconds)
  double estimated_time;
  /// Next state information
  /// Recommended state after recovery
  rosidl_runtime_c__String recommended_next_state;
  /// Operational restrictions after recovery
  rosidl_runtime_c__String__Sequence restrictions;
} autonomy_interfaces__srv__RecoverFromSafety_Response;

// Struct for a sequence of autonomy_interfaces__srv__RecoverFromSafety_Response.
typedef struct autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence
{
  autonomy_interfaces__srv__RecoverFromSafety_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  autonomy_interfaces__srv__RecoverFromSafety_Event__request__MAX_SIZE = 1
};
// response
enum
{
  autonomy_interfaces__srv__RecoverFromSafety_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/RecoverFromSafety in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__RecoverFromSafety_Event
{
  service_msgs__msg__ServiceEventInfo info;
  autonomy_interfaces__srv__RecoverFromSafety_Request__Sequence request;
  autonomy_interfaces__srv__RecoverFromSafety_Response__Sequence response;
} autonomy_interfaces__srv__RecoverFromSafety_Event;

// Struct for a sequence of autonomy_interfaces__srv__RecoverFromSafety_Event.
typedef struct autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence
{
  autonomy_interfaces__srv__RecoverFromSafety_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__RecoverFromSafety_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__RECOVER_FROM_SAFETY__STRUCT_H_
