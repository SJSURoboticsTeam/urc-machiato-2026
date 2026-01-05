// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/GetSafetyStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/get_safety_status.h"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_SAFETY_STATUS__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_SAFETY_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetSafetyStatus in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__GetSafetyStatus_Request
{
  uint8_t structure_needs_at_least_one_member;
} autonomy_interfaces__srv__GetSafetyStatus_Request;

// Struct for a sequence of autonomy_interfaces__srv__GetSafetyStatus_Request.
typedef struct autonomy_interfaces__srv__GetSafetyStatus_Request__Sequence
{
  autonomy_interfaces__srv__GetSafetyStatus_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__GetSafetyStatus_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'overall_safety'
#include "rosidl_runtime_c/string.h"
// Member 'active_alerts'
#include "autonomy_interfaces/msg/detail/safety_alert__struct.h"
// Member 'monitoring_stats'
#include "autonomy_interfaces/msg/detail/monitoring_stats__struct.h"

/// Struct defined in srv/GetSafetyStatus in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__GetSafetyStatus_Response
{
  rosidl_runtime_c__String overall_safety;
  autonomy_interfaces__msg__SafetyAlert__Sequence active_alerts;
  autonomy_interfaces__msg__MonitoringStats monitoring_stats;
} autonomy_interfaces__srv__GetSafetyStatus_Response;

// Struct for a sequence of autonomy_interfaces__srv__GetSafetyStatus_Response.
typedef struct autonomy_interfaces__srv__GetSafetyStatus_Response__Sequence
{
  autonomy_interfaces__srv__GetSafetyStatus_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__GetSafetyStatus_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  autonomy_interfaces__srv__GetSafetyStatus_Event__request__MAX_SIZE = 1
};
// response
enum
{
  autonomy_interfaces__srv__GetSafetyStatus_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetSafetyStatus in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__GetSafetyStatus_Event
{
  service_msgs__msg__ServiceEventInfo info;
  autonomy_interfaces__srv__GetSafetyStatus_Request__Sequence request;
  autonomy_interfaces__srv__GetSafetyStatus_Response__Sequence response;
} autonomy_interfaces__srv__GetSafetyStatus_Event;

// Struct for a sequence of autonomy_interfaces__srv__GetSafetyStatus_Event.
typedef struct autonomy_interfaces__srv__GetSafetyStatus_Event__Sequence
{
  autonomy_interfaces__srv__GetSafetyStatus_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__GetSafetyStatus_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_SAFETY_STATUS__STRUCT_H_
