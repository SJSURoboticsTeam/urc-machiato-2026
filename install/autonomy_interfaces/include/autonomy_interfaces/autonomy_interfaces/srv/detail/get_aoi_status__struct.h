// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/GetAOIStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/get_aoi_status.h"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_AOI_STATUS__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_AOI_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'sensor_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetAOIStatus in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__GetAOIStatus_Request
{
  /// Request
  /// Specific sensor to query (empty for all)
  rosidl_runtime_c__String sensor_name;
  /// Whether to include historical data
  bool include_history;
  /// Number of historical samples to return (0 = current only)
  uint32_t history_samples;
} autonomy_interfaces__srv__GetAOIStatus_Request;

// Struct for a sequence of autonomy_interfaces__srv__GetAOIStatus_Request.
typedef struct autonomy_interfaces__srv__GetAOIStatus_Request__Sequence
{
  autonomy_interfaces__srv__GetAOIStatus_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__GetAOIStatus_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'sensor_status'
#include "autonomy_interfaces/msg/detail/aoi_status__struct.h"
// Member 'aoi_history'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'timestamp_history'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'system_metrics'
#include "autonomy_interfaces/msg/detail/aoi_metrics__struct.h"

/// Struct defined in srv/GetAOIStatus in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__GetAOIStatus_Response
{
  /// Whether query was successful
  bool success;
  /// Status message
  rosidl_runtime_c__String message;
  /// Current status
  /// Status for each sensor
  autonomy_interfaces__msg__AOIStatus__Sequence sensor_status;
  /// Historical data (if requested)
  /// Historical AoI values
  rosidl_runtime_c__double__Sequence aoi_history;
  /// Corresponding timestamps
  builtin_interfaces__msg__Time__Sequence timestamp_history;
  /// System summary
  /// Overall system AoI metrics
  autonomy_interfaces__msg__AOIMetrics system_metrics;
} autonomy_interfaces__srv__GetAOIStatus_Response;

// Struct for a sequence of autonomy_interfaces__srv__GetAOIStatus_Response.
typedef struct autonomy_interfaces__srv__GetAOIStatus_Response__Sequence
{
  autonomy_interfaces__srv__GetAOIStatus_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__GetAOIStatus_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  autonomy_interfaces__srv__GetAOIStatus_Event__request__MAX_SIZE = 1
};
// response
enum
{
  autonomy_interfaces__srv__GetAOIStatus_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetAOIStatus in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__GetAOIStatus_Event
{
  service_msgs__msg__ServiceEventInfo info;
  autonomy_interfaces__srv__GetAOIStatus_Request__Sequence request;
  autonomy_interfaces__srv__GetAOIStatus_Response__Sequence response;
} autonomy_interfaces__srv__GetAOIStatus_Event;

// Struct for a sequence of autonomy_interfaces__srv__GetAOIStatus_Event.
typedef struct autonomy_interfaces__srv__GetAOIStatus_Event__Sequence
{
  autonomy_interfaces__srv__GetAOIStatus_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__GetAOIStatus_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_AOI_STATUS__STRUCT_H_
