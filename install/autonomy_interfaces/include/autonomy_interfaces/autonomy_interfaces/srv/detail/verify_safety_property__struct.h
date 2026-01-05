// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/VerifySafetyProperty.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/verify_safety_property.h"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__VERIFY_SAFETY_PROPERTY__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__VERIFY_SAFETY_PROPERTY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'property_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/VerifySafetyProperty in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__VerifySafetyProperty_Request
{
  rosidl_runtime_c__String property_name;
} autonomy_interfaces__srv__VerifySafetyProperty_Request;

// Struct for a sequence of autonomy_interfaces__srv__VerifySafetyProperty_Request.
typedef struct autonomy_interfaces__srv__VerifySafetyProperty_Request__Sequence
{
  autonomy_interfaces__srv__VerifySafetyProperty_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__VerifySafetyProperty_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'property_name'
// Member 'details'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/VerifySafetyProperty in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__VerifySafetyProperty_Response
{
  rosidl_runtime_c__String property_name;
  bool satisfied;
  rosidl_runtime_c__String details;
  int32_t violation_count;
  double last_violation;
} autonomy_interfaces__srv__VerifySafetyProperty_Response;

// Struct for a sequence of autonomy_interfaces__srv__VerifySafetyProperty_Response.
typedef struct autonomy_interfaces__srv__VerifySafetyProperty_Response__Sequence
{
  autonomy_interfaces__srv__VerifySafetyProperty_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__VerifySafetyProperty_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  autonomy_interfaces__srv__VerifySafetyProperty_Event__request__MAX_SIZE = 1
};
// response
enum
{
  autonomy_interfaces__srv__VerifySafetyProperty_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/VerifySafetyProperty in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__VerifySafetyProperty_Event
{
  service_msgs__msg__ServiceEventInfo info;
  autonomy_interfaces__srv__VerifySafetyProperty_Request__Sequence request;
  autonomy_interfaces__srv__VerifySafetyProperty_Response__Sequence response;
} autonomy_interfaces__srv__VerifySafetyProperty_Event;

// Struct for a sequence of autonomy_interfaces__srv__VerifySafetyProperty_Event.
typedef struct autonomy_interfaces__srv__VerifySafetyProperty_Event__Sequence
{
  autonomy_interfaces__srv__VerifySafetyProperty_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__VerifySafetyProperty_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__VERIFY_SAFETY_PROPERTY__STRUCT_H_
