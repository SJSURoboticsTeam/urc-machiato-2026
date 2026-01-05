// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/ValidateCalibration.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/srv/validate_calibration.h"


#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__VALIDATE_CALIBRATION__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__VALIDATE_CALIBRATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'calibration_file'
// Member 'test_images'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ValidateCalibration in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__ValidateCalibration_Request
{
  rosidl_runtime_c__String calibration_file;
  rosidl_runtime_c__String__Sequence test_images;
} autonomy_interfaces__srv__ValidateCalibration_Request;

// Struct for a sequence of autonomy_interfaces__srv__ValidateCalibration_Request.
typedef struct autonomy_interfaces__srv__ValidateCalibration_Request__Sequence
{
  autonomy_interfaces__srv__ValidateCalibration_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__ValidateCalibration_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'quality_assessment'
// Member 'recommendations'
// Member 'error_message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ValidateCalibration in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__ValidateCalibration_Response
{
  bool success;
  float reprojection_error;
  rosidl_runtime_c__String quality_assessment;
  rosidl_runtime_c__String recommendations;
  rosidl_runtime_c__String error_message;
} autonomy_interfaces__srv__ValidateCalibration_Response;

// Struct for a sequence of autonomy_interfaces__srv__ValidateCalibration_Response.
typedef struct autonomy_interfaces__srv__ValidateCalibration_Response__Sequence
{
  autonomy_interfaces__srv__ValidateCalibration_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__ValidateCalibration_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  autonomy_interfaces__srv__ValidateCalibration_Event__request__MAX_SIZE = 1
};
// response
enum
{
  autonomy_interfaces__srv__ValidateCalibration_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/ValidateCalibration in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__ValidateCalibration_Event
{
  service_msgs__msg__ServiceEventInfo info;
  autonomy_interfaces__srv__ValidateCalibration_Request__Sequence request;
  autonomy_interfaces__srv__ValidateCalibration_Response__Sequence response;
} autonomy_interfaces__srv__ValidateCalibration_Event;

// Struct for a sequence of autonomy_interfaces__srv__ValidateCalibration_Event.
typedef struct autonomy_interfaces__srv__ValidateCalibration_Event__Sequence
{
  autonomy_interfaces__srv__ValidateCalibration_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__ValidateCalibration_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__VALIDATE_CALIBRATION__STRUCT_H_
