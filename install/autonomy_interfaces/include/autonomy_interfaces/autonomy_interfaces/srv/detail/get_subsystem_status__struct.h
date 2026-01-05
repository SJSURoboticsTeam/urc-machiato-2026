// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/GetSubsystemStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/get_subsystem_status.h"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_SUBSYSTEM_STATUS__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_SUBSYSTEM_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'subsystem_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetSubsystemStatus in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__GetSubsystemStatus_Request
{
  /// "navigation", "slam", "computer_vision", "autonomous_typing", "led_status", "all"
  rosidl_runtime_c__String subsystem_name;
} autonomy_interfaces__srv__GetSubsystemStatus_Request;

// Struct for a sequence of autonomy_interfaces__srv__GetSubsystemStatus_Request.
typedef struct autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence
{
  autonomy_interfaces__srv__GetSubsystemStatus_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'error_message'
// Member 'subsystem_names'
// Member 'subsystem_states'
// Member 'status_messages'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'subsystem_health'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/GetSubsystemStatus in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__GetSubsystemStatus_Response
{
  bool success;
  rosidl_runtime_c__String error_message;
  rosidl_runtime_c__String__Sequence subsystem_names;
  rosidl_runtime_c__String__Sequence subsystem_states;
  /// 0.0 to 1.0 for each subsystem
  rosidl_runtime_c__float__Sequence subsystem_health;
  rosidl_runtime_c__String__Sequence status_messages;
} autonomy_interfaces__srv__GetSubsystemStatus_Response;

// Struct for a sequence of autonomy_interfaces__srv__GetSubsystemStatus_Response.
typedef struct autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence
{
  autonomy_interfaces__srv__GetSubsystemStatus_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  autonomy_interfaces__srv__GetSubsystemStatus_Event__request__MAX_SIZE = 1
};
// response
enum
{
  autonomy_interfaces__srv__GetSubsystemStatus_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetSubsystemStatus in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__GetSubsystemStatus_Event
{
  service_msgs__msg__ServiceEventInfo info;
  autonomy_interfaces__srv__GetSubsystemStatus_Request__Sequence request;
  autonomy_interfaces__srv__GetSubsystemStatus_Response__Sequence response;
} autonomy_interfaces__srv__GetSubsystemStatus_Event;

// Struct for a sequence of autonomy_interfaces__srv__GetSubsystemStatus_Event.
typedef struct autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence
{
  autonomy_interfaces__srv__GetSubsystemStatus_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__GetSubsystemStatus_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_SUBSYSTEM_STATUS__STRUCT_H_
