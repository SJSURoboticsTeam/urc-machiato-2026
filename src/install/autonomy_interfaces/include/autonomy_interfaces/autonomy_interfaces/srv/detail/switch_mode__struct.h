// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/SwitchMode.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__SWITCH_MODE__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__SWITCH_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'requested_mode'
// Member 'reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SwitchMode in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__SwitchMode_Request
{
  /// "autonomous", "teleoperation", "manual_override", "idle"
  rosidl_runtime_c__String requested_mode;
  /// Human-readable reason for mode switch
  rosidl_runtime_c__String reason;
} autonomy_interfaces__srv__SwitchMode_Request;

// Struct for a sequence of autonomy_interfaces__srv__SwitchMode_Request.
typedef struct autonomy_interfaces__srv__SwitchMode_Request__Sequence
{
  autonomy_interfaces__srv__SwitchMode_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__SwitchMode_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// Member 'actual_mode'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SwitchMode in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__SwitchMode_Response
{
  bool success;
  rosidl_runtime_c__String message;
  /// Mode system is actually in after request
  rosidl_runtime_c__String actual_mode;
  /// Seconds taken to complete transition
  float transition_time;
} autonomy_interfaces__srv__SwitchMode_Response;

// Struct for a sequence of autonomy_interfaces__srv__SwitchMode_Response.
typedef struct autonomy_interfaces__srv__SwitchMode_Response__Sequence
{
  autonomy_interfaces__srv__SwitchMode_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__SwitchMode_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__SWITCH_MODE__STRUCT_H_
