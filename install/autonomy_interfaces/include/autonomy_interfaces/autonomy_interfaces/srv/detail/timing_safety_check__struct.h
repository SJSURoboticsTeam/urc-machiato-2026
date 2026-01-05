// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/TimingSafetyCheck.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/timing_safety_check.h"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__TIMING_SAFETY_CHECK__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__TIMING_SAFETY_CHECK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'monitored_components'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/TimingSafetyCheck in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__TimingSafetyCheck_Request
{
  /// Request
  /// Check real-time performance
  bool real_time_check;
  /// Time window to analyze (seconds)
  double time_window;
  /// Specific components to check timing for
  rosidl_runtime_c__String__Sequence monitored_components;
} autonomy_interfaces__srv__TimingSafetyCheck_Request;

// Struct for a sequence of autonomy_interfaces__srv__TimingSafetyCheck_Request.
typedef struct autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence
{
  autonomy_interfaces__srv__TimingSafetyCheck_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'timing_status'
// Member 'components_checked'
// Member 'timing_recommendations'
// Member 'timestamp'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'component_avg_times'
// Member 'component_deadlines_missed'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/TimingSafetyCheck in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__TimingSafetyCheck_Response
{
  /// Overall timing safety status
  bool timing_safe;
  /// "NOMINAL", "WARNING", "CRITICAL", "FAILED"
  rosidl_runtime_c__String timing_status;
  /// Timing metrics
  /// Average response time (ms)
  double avg_response_time;
  /// Maximum response time (ms)
  double max_response_time;
  /// Minimum response time (ms)
  double min_response_time;
  /// Response time jitter (ms)
  double jitter;
  /// Number of deadline misses in time window
  int32_t deadline_misses;
  /// Rate of deadline misses (per second)
  double deadline_miss_rate;
  /// Component timing
  /// Components that were checked
  rosidl_runtime_c__String__Sequence components_checked;
  /// Average response times per component
  rosidl_runtime_c__double__Sequence component_avg_times;
  /// Deadline misses per component
  rosidl_runtime_c__int32__Sequence component_deadlines_missed;
  /// System health
  /// Current CPU utilization (0.0-1.0)
  double cpu_utilization;
  /// Current memory utilization (0.0-1.0)
  double memory_utilization;
  /// Current number of active threads
  int32_t thread_count;
  /// Whether real-time scheduling is active
  bool real_time_scheduling;
  /// Recommendations
  /// Actions to improve timing if needed
  rosidl_runtime_c__String__Sequence timing_recommendations;
  /// When check was performed
  rosidl_runtime_c__String timestamp;
} autonomy_interfaces__srv__TimingSafetyCheck_Response;

// Struct for a sequence of autonomy_interfaces__srv__TimingSafetyCheck_Response.
typedef struct autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence
{
  autonomy_interfaces__srv__TimingSafetyCheck_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  autonomy_interfaces__srv__TimingSafetyCheck_Event__request__MAX_SIZE = 1
};
// response
enum
{
  autonomy_interfaces__srv__TimingSafetyCheck_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/TimingSafetyCheck in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__TimingSafetyCheck_Event
{
  service_msgs__msg__ServiceEventInfo info;
  autonomy_interfaces__srv__TimingSafetyCheck_Request__Sequence request;
  autonomy_interfaces__srv__TimingSafetyCheck_Response__Sequence response;
} autonomy_interfaces__srv__TimingSafetyCheck_Event;

// Struct for a sequence of autonomy_interfaces__srv__TimingSafetyCheck_Event.
typedef struct autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence
{
  autonomy_interfaces__srv__TimingSafetyCheck_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__TimingSafetyCheck_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__TIMING_SAFETY_CHECK__STRUCT_H_
