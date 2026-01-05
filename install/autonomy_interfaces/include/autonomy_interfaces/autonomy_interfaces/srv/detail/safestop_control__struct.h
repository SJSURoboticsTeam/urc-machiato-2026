// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/SafestopControl.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/safestop_control.h"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__SAFESTOP_CONTROL__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__SAFESTOP_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
// Member 'operator_id'
// Member 'reason'
// Member 'affected_subsystems'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SafestopControl in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__SafestopControl_Request
{
  /// Request
  /// "ENGAGE", "DISENGAGE", "TOGGLE"
  rosidl_runtime_c__String command;
  /// ID of operator initiating the command
  rosidl_runtime_c__String operator_id;
  /// Operator acknowledges any risks
  bool acknowledge_risks;
  /// Optional reason for the command
  rosidl_runtime_c__String reason;
  /// Specific subsystems to control (empty = all)
  rosidl_runtime_c__String__Sequence affected_subsystems;
} autonomy_interfaces__srv__SafestopControl_Request;

// Struct for a sequence of autonomy_interfaces__srv__SafestopControl_Request.
typedef struct autonomy_interfaces__srv__SafestopControl_Request__Sequence
{
  autonomy_interfaces__srv__SafestopControl_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__SafestopControl_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// Member 'current_state'
// Member 'affected_subsystems'
// Member 'operator_id'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SafestopControl in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__SafestopControl_Response
{
  /// Command executed successfully
  bool success;
  /// Human-readable status message
  rosidl_runtime_c__String message;
  /// "ENGAGED", "DISENGAGED", "IN_PROGRESS", "FAILED"
  rosidl_runtime_c__String current_state;
  /// Subsystems that were affected
  rosidl_runtime_c__String__Sequence affected_subsystems;
  /// When command was processed
  double timestamp;
  /// Operator who executed the command
  rosidl_runtime_c__String operator_id;
} autonomy_interfaces__srv__SafestopControl_Response;

// Struct for a sequence of autonomy_interfaces__srv__SafestopControl_Response.
typedef struct autonomy_interfaces__srv__SafestopControl_Response__Sequence
{
  autonomy_interfaces__srv__SafestopControl_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__SafestopControl_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  autonomy_interfaces__srv__SafestopControl_Event__request__MAX_SIZE = 1
};
// response
enum
{
  autonomy_interfaces__srv__SafestopControl_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/SafestopControl in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__SafestopControl_Event
{
  service_msgs__msg__ServiceEventInfo info;
  autonomy_interfaces__srv__SafestopControl_Request__Sequence request;
  autonomy_interfaces__srv__SafestopControl_Response__Sequence response;
} autonomy_interfaces__srv__SafestopControl_Event;

// Struct for a sequence of autonomy_interfaces__srv__SafestopControl_Event.
typedef struct autonomy_interfaces__srv__SafestopControl_Event__Sequence
{
  autonomy_interfaces__srv__SafestopControl_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__SafestopControl_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__SAFESTOP_CONTROL__STRUCT_H_
