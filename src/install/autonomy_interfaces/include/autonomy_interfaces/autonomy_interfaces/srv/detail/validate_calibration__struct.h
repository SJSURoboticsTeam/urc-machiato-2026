// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/ValidateCalibration.idl
// generated code does not contain a copyright notice

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

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__VALIDATE_CALIBRATION__STRUCT_H_
