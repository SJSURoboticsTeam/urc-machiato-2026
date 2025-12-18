// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/SoftwareEstop.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__SOFTWARE_ESTOP__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__SOFTWARE_ESTOP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'operator_id'
// Member 'reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SoftwareEstop in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__SoftwareEstop_Request
{
  /// Request
  /// ID of operator triggering software ESTOP
  rosidl_runtime_c__String operator_id;
  /// Reason for triggering software ESTOP
  rosidl_runtime_c__String reason;
  /// Operator acknowledges this is critical action
  bool acknowledge_criticality;
  /// Force immediate shutdown (no coordination)
  bool force_immediate;
} autonomy_interfaces__srv__SoftwareEstop_Request;

// Struct for a sequence of autonomy_interfaces__srv__SoftwareEstop_Request.
typedef struct autonomy_interfaces__srv__SoftwareEstop_Request__Sequence
{
  autonomy_interfaces__srv__SoftwareEstop_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__SoftwareEstop_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// Member 'estop_id'
// Member 'triggered_by'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SoftwareEstop in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__SoftwareEstop_Response
{
  /// True if software ESTOP was triggered
  bool success;
  /// Status message
  rosidl_runtime_c__String message;
  /// Unique ID for this ESTOP event
  rosidl_runtime_c__String estop_id;
  /// When ESTOP was triggered
  double timestamp;
  /// Operator who triggered it
  rosidl_runtime_c__String triggered_by;
  /// Whether subsystem coordination began
  bool coordination_started;
} autonomy_interfaces__srv__SoftwareEstop_Response;

// Struct for a sequence of autonomy_interfaces__srv__SoftwareEstop_Response.
typedef struct autonomy_interfaces__srv__SoftwareEstop_Response__Sequence
{
  autonomy_interfaces__srv__SoftwareEstop_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__SoftwareEstop_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__SOFTWARE_ESTOP__STRUCT_H_
